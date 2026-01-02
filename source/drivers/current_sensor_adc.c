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
    esp_foc_lp_filter_t current_filters[4];
    adc_channel_t channels[4];
    adc_unit_t units[4];
    adc_continuous_handle_t handle;
    float adc_to_current_scale;
    uint32_t raw_reads[4];
    float currents[4];
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
static const float adc_to_volts = ((3.9f)/ 4096.0f);

static bool isensor_adc_done_callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    adc_hal_digi_enable(false);
    adc_hal_digi_connect(false);

    isensor_adc_t *isensor = (isensor_adc_t *)user_data;
    adc_digi_output_data_t *p = (adc_digi_output_data_t *)edata->conv_frame_buffer;

    for(int i = 0; i < isensor->number_of_channels; i++) {
        isensor->raw_reads[i] = ADC_GET_DATA(p);
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
    isensor_adc_t *obj =
        __containerof(self, isensor_adc_t, interface);

    esp_foc_critical_enter();
    obj->currents[0] = (float)obj->raw_reads[0] * adc_to_volts;
    obj->currents[1] = (float)obj->raw_reads[1] * adc_to_volts;
    obj->currents[2] = (float)obj->raw_reads[2] * adc_to_volts;
    obj->currents[3] = (float)obj->raw_reads[3] * adc_to_volts;
    esp_foc_critical_leave();

    values->iu_axis_0 = esp_foc_low_pass_filter_update(&obj->current_filters[0], (obj->currents[0] - obj->offsets[0]) * isensor_adc.adc_to_current_scale);
    values->iv_axis_0 = esp_foc_low_pass_filter_update(&obj->current_filters[1], (obj->currents[1] - obj->offsets[1]) * isensor_adc.adc_to_current_scale);
    values->iw_axis_0 = -(values->iu_axis_0 +  values->iv_axis_0);
    values->iu_axis_1 = esp_foc_low_pass_filter_update(&obj->current_filters[2], (obj->currents[2] - obj->offsets[2]) * isensor_adc.adc_to_current_scale);
    values->iv_axis_1 = esp_foc_low_pass_filter_update(&obj->current_filters[3], (obj->currents[3] - obj->offsets[3]) * isensor_adc.adc_to_current_scale);
    values->iw_axis_1 = -(values->iu_axis_1 +  values->iv_axis_1);

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

        obj->offsets[0] += ((float)obj->raw_reads[0] * adc_to_volts);
        obj->offsets[1] += ((float)obj->raw_reads[1] * adc_to_volts);
        obj->offsets[2] += ((float)obj->raw_reads[2] * adc_to_volts);
        obj->offsets[3] += ((float)obj->raw_reads[3] * adc_to_volts);
    }

    obj->offsets[0] /= calibration_rounds;
    obj->offsets[1] /= calibration_rounds;
    obj->offsets[2] /= calibration_rounds;
    obj->offsets[3] /= calibration_rounds;

    ESP_LOGI(TAG, "ADC calibrated, phase current offsets are: %f, %f, %f, %f",
            obj->offsets[0], obj->offsets[1], obj->offsets[2], obj->offsets[3]);
    esp_foc_sleep_ms(100);

    /* Dummy read to check reading when no current is flowing*/
    self->sample_isensors(self);
    esp_foc_sleep_ms(10);
    self->fetch_isensors(self, &val);

    ESP_LOGI(TAG, "No current flow isensor test read: %f, %f, %f, %f, %f,%f",
            val.iu_axis_0, val.iv_axis_0, val.iw_axis_0, val.iu_axis_1, val.iv_axis_1, val.iw_axis_1);
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


esp_foc_isensor_t *isensor_adc_new(esp_foc_isensor_adc_config_t *config)
{

    if(adc_initialized == true) {
        return &isensor_adc.interface;
    }

    isensor_adc.adc_to_current_scale = (1.0f / (config->amp_gain * config->shunt_resistance));

    isensor_adc.interface.fetch_isensors = fetch_isensors;
    isensor_adc.interface.sample_isensors = sample_isensors;
    isensor_adc.interface.calibrate_isensors = calibrate_isensors;
    isensor_adc.interface.set_isensor_callback = set_callback;
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

    esp_foc_low_pass_filter_init(&isensor_adc.current_filters[0], 1.0);
    esp_foc_low_pass_filter_init(&isensor_adc.current_filters[1], 1.0);
    esp_foc_low_pass_filter_init(&isensor_adc.current_filters[2], 1.0);
    esp_foc_low_pass_filter_init(&isensor_adc.current_filters[3], 1.0);

    continuous_adc_init(&isensor_adc);
    adc_initialized = true;

    return &isensor_adc.interface;
}
