/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <sys/cdefs.h>
#include <stdbool.h>
#include <sdkconfig.h>
#include <math.h>
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/biquad_q16.h"
#include "espFoC/driver_iq31_local.h"
#include "espFoC/current_sensor_adc.h"
#include "hal/adc_hal.h"
#include "esp_log.h"

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define ADC_GET_CHANNEL(p_data)     ((p_data)->type1.channel)
#define ADC_GET_DATA(p_data)        ((p_data)->type1.data)
#else
#define ADC_GET_CHANNEL(p_data)     ((p_data)->type2.channel)
#define ADC_GET_DATA(p_data)        ((p_data)->type2.data)
#endif

static const char *TAG = "ESP_FOC_ISENSOR";

typedef struct {
    adc_channel_t channels[4];
    adc_unit_t units[4];
    adc_continuous_handle_t handle;
    float adc_to_current_scale;
    q16_t adc_to_current_scale_q16;
    /* latest_raw is the most recent unfiltered ADC sample per channel.
     * Used by calibration (we want the raw DC at no-current). */
    int32_t latest_raw[4];
    /* filtered_count is bq[i] output mapped back into ADC count units
     * (still int32 because the rest of the pipeline subtracts an int
     * offset before scaling). */
    int32_t filtered_count[4];
    esp_foc_biquad_q16_t bq[4];
    float offsets[4];
    esp_foc_isensor_t interface;
    int number_of_channels;
    isensor_callback_t callback;
    void *user_data;
}isensor_adc_t;

static adc_continuous_handle_cfg_t adc_config = {
    .max_store_buf_size = 2 * SOC_ADC_DIGI_RESULT_BYTES,
    .conv_frame_size = 2 * SOC_ADC_DIGI_RESULT_BYTES,
    .flags = {
        .flush_pool = 1,
    },
};

static adc_continuous_config_t dig_cfg = {
#if CONFIG_IDF_TARGET_ESP32
    .sample_freq_hz = 80000,
#else
    .sample_freq_hz = 80000,
#endif
    .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    .pattern_num = 2,
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
    .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
#else
    .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
#endif
};

static adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};

DRAM_ATTR static isensor_adc_t isensor_adc;
static bool adc_initialized = false;
static const float adc_to_volts = ((3.1f)/ 4096.0f);

static bool isensor_adc_done_callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    adc_hal_digi_enable(false);
    adc_hal_digi_connect(false);

    isensor_adc_t *isensor = (isensor_adc_t *)user_data;
    adc_digi_output_data_t *p = (adc_digi_output_data_t *)edata->conv_frame_buffer;

    /* Push every fresh ADC sample straight through its per-channel
     * biquad. Raw counts (12-bit) shift-left into Q16 with plenty of
     * headroom (4095 << 16 = 268 M, well below INT32_MAX). The output
     * shifts back to count units; the rest of the pipeline never sees
     * the q16 representation. */
    for(int i = 0; i < isensor->number_of_channels; i++) {
        int32_t raw = (int32_t)ADC_GET_DATA(p);
        isensor->latest_raw[i] = raw;
        q16_t y = esp_foc_biquad_q16_update(&isensor->bq[i],
                                            (q16_t)(raw << 16));
        isensor->filtered_count[i] = y >> 16;
        p++;
    }

    if(isensor->callback != NULL) {
        isensor->callback(isensor->user_data);
    }

    return false;
}

static const adc_continuous_evt_cbs_t cbs = {
    .on_conv_done = isensor_adc_done_callback,
};

static void continuous_adc_init(isensor_adc_t *isensor)
{
    adc_config.conv_frame_size = isensor->number_of_channels * SOC_ADC_DIGI_RESULT_BYTES;
    adc_continuous_new_handle(&adc_config, &isensor->handle);

    for (int i = 0; i < isensor->number_of_channels; i++) {
        adc_pattern[i].atten = ADC_ATTEN_DB_12;
        adc_pattern[i].channel = isensor->channels[i] & 0x7;
        adc_pattern[i].unit = isensor_adc.units[0];
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
    }

    dig_cfg.adc_pattern = adc_pattern;
    dig_cfg.pattern_num =  isensor->number_of_channels;
    adc_continuous_config(isensor->handle, &dig_cfg);
    adc_continuous_register_event_callbacks(isensor->handle, &cbs, isensor);
    adc_continuous_start(isensor->handle);
    esp_foc_sleep_ms(10);
}

static void fetch_isensors(esp_foc_isensor_t *self, isensor_values_t *values)
{
    isensor_adc_t *obj = __containerof(self, isensor_adc_t, interface);
    const int32_t adc_rng = 2048;

    /* Pull the latest filtered counts produced by the per-channel
     * biquad in the ADC ISR. */
    int32_t f0 = obj->filtered_count[0];
    int32_t f1 = obj->filtered_count[1];
    int32_t f2 = obj->filtered_count[2];
    int32_t f3 = obj->filtered_count[3];

    int32_t d0 = f0 - (int32_t)lroundf(obj->offsets[0]);
    int32_t d1 = f1 - (int32_t)lroundf(obj->offsets[1]);
    int32_t d2 = f2 - (int32_t)lroundf(obj->offsets[2]);
    int32_t d3 = f3 - (int32_t)lroundf(obj->offsets[3]);

    d0 = esp_foc_clamp_int32(d0, -adc_rng, adc_rng);
    d1 = esp_foc_clamp_int32(d1, -adc_rng, adc_rng);
    d2 = esp_foc_clamp_int32(d2, -adc_rng, adc_rng);
    d3 = esp_foc_clamp_int32(d3, -adc_rng, adc_rng);

    q16_t iu0 = q16_mul(esp_foc_q16_from_adc_diff_clamped(d0, adc_rng), obj->adc_to_current_scale_q16);
    q16_t iv0 = q16_mul(esp_foc_q16_from_adc_diff_clamped(d1, adc_rng), obj->adc_to_current_scale_q16);
    q16_t iu1 = q16_mul(esp_foc_q16_from_adc_diff_clamped(d2, adc_rng), obj->adc_to_current_scale_q16);
    q16_t iv1 = q16_mul(esp_foc_q16_from_adc_diff_clamped(d3, adc_rng), obj->adc_to_current_scale_q16);

    q16_t zero = 0;
    q16_t iw0 = q16_sub(zero, q16_add(iu0, iv0));
    q16_t iw1 = q16_sub(zero, q16_add(iu1, iv1));

    values->iu_axis_0 = iu0;
    values->iv_axis_0 = iv0;
    values->iw_axis_0 = iw0;
    values->iu_axis_1 = iu1;
    values->iv_axis_1 = iv1;
    values->iw_axis_1 = iw1;
}

static void sample_isensors(esp_foc_isensor_t *self)
{
    adc_hal_digi_connect(true);
    adc_hal_digi_enable(true);
}

static void calibrate_isensors (esp_foc_isensor_t *self, int calibration_rounds)
{
    isensor_values_t val;
    isensor_adc_t *obj =
        __containerof(self, isensor_adc_t, interface);

    esp_foc_sleep_ms(100);

    obj->offsets[0] = 0.0f;
    obj->offsets[1] = 0.0f;
    obj->offsets[2] = 0.0f;
    obj->offsets[3] = 0.0f;

    for(int i = 0; i < calibration_rounds; i++) {
        self->sample_isensors(self);
        esp_foc_sleep_ms(10);

        /* Use the unfiltered latest_raw value here on purpose: we want
         * the genuine zero-current DC level, not whatever the biquad
         * happens to be settling towards (the loop is still cold and
         * the filter has not had time to converge across rounds). */
        obj->offsets[0] += ((float)obj->latest_raw[0]);
        obj->offsets[1] += ((float)obj->latest_raw[1]);
        obj->offsets[2] += ((float)obj->latest_raw[2]);
        obj->offsets[3] += ((float)obj->latest_raw[3]);
    }

    obj->offsets[0] /= calibration_rounds;
    obj->offsets[1] /= calibration_rounds;
    obj->offsets[2] /= calibration_rounds;
    obj->offsets[3] /= calibration_rounds;

    /* Clear filter state so the post-calibration steady state lines up
     * with the freshly-measured offsets. */
    for (int i = 0; i < 4; ++i) {
        esp_foc_biquad_q16_reset(&obj->bq[i]);
        obj->filtered_count[i] = obj->latest_raw[i];
    }

    ESP_LOGI(TAG, "ADC calibrated, phase current offsets are: %f, %f, %f, %f",
            obj->offsets[0], obj->offsets[1], obj->offsets[2], obj->offsets[3]);
    esp_foc_sleep_ms(100);

    /* Dummy read to check reading when no current is flowing*/
    self->sample_isensors(self);
    esp_foc_sleep_ms(10);
    self->fetch_isensors(self, &val);

    ESP_LOGI(TAG, "No current flow isensor test read: %f, %f, %f, %f, %f, %f",
            q16_to_float(val.iu_axis_0), q16_to_float(val.iv_axis_0), q16_to_float(val.iw_axis_0),
            q16_to_float(val.iu_axis_1), q16_to_float(val.iv_axis_1), q16_to_float(val.iw_axis_1));
    esp_foc_sleep_ms(100);
}

static void set_callback(esp_foc_isensor_t *self, isensor_callback_t cb, void *arg)
{
    isensor_adc_t *obj =
        __containerof(self, isensor_adc_t, interface);

    esp_foc_critical_enter();
    obj->callback = cb;
    obj->user_data = arg;
    esp_foc_critical_leave();
}

static void set_filter_cutoff(esp_foc_isensor_t *self, float fc_hz, float fs_hz)
{
    isensor_adc_t *obj =
        __containerof(self, isensor_adc_t, interface);

    /* Designer is float-heavy; do it under critical section so the ADC
     * ISR cannot land mid-redesign and use partially-updated coefs. */
    esp_foc_critical_enter();
    for (int i = 0; i < 4; ++i) {
        esp_foc_biquad_butterworth_lpf_design_q16(&obj->bq[i], fc_hz, fs_hz);
    }
    esp_foc_critical_leave();
}


esp_foc_isensor_t *isensor_adc_new(esp_foc_isensor_adc_config_t *config)
{

    if(adc_initialized == true) {
        return &isensor_adc.interface;
    }

    isensor_adc.adc_to_current_scale = adc_to_volts * (1.0f / (config->amp_gain * config->shunt_resistance));
    isensor_adc.adc_to_current_scale_q16 = q16_from_float(2048.0f * isensor_adc.adc_to_current_scale);

    isensor_adc.interface.fetch_isensors = fetch_isensors;
    isensor_adc.interface.sample_isensors = sample_isensors;
    isensor_adc.interface.calibrate_isensors = calibrate_isensors;
    isensor_adc.interface.set_isensor_callback = set_callback;
    isensor_adc.interface.set_filter_cutoff = set_filter_cutoff;
    /* Default to bypass so the driver works correctly even if the
     * caller (axis init / tuner) never invokes set_filter_cutoff. */
    for (int i = 0; i < 4; ++i) {
        esp_foc_biquad_q16_set_bypass(&isensor_adc.bq[i]);
    }
    isensor_adc.number_of_channels = config->number_of_channels;
    isensor_adc.channels[0] = config->axis_channels[0];
    isensor_adc.channels[1] = config->axis_channels[1];
    isensor_adc.channels[2] = config->axis_channels[2];
    isensor_adc.channels[3] = config->axis_channels[3];
    isensor_adc.units[0] = config->units[0];
    isensor_adc.units[1] = config->units[1];
    isensor_adc.units[2] = config->units[2];
    isensor_adc.units[3] = config->units[3];
    isensor_adc.offsets[0] = 0.0f;
    isensor_adc.offsets[1] = 0.0f;
    isensor_adc.offsets[2] = 0.0f;
    isensor_adc.offsets[3] = 0.0f;
    isensor_adc.callback = NULL;

    continuous_adc_init(&isensor_adc);
    adc_initialized = true;

    return &isensor_adc.interface;
}
