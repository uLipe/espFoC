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
#include <math.h>
#include <sdkconfig.h>
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/biquad_q16.h"
#include "espFoC/driver_iq31_local.h"
#include "espFoC/current_sensor_adc_one_shot.h"
#include "hal/adc_hal.h"
#include "hal/adc_oneshot_hal.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_clk_tree.h"

#ifdef CONFIG_IDF_TARGET_ESP32P4
static const char *TAG = "ESP_FOC_ISENSOR_ONE_SHOT";

typedef struct {
    adc_channel_t channels[4];
    adc_unit_t units[4];
    adc_oneshot_hal_ctx_t hal;
    float adc_to_current_scale;
    q16_t adc_to_current_scale_q16;
    int32_t encoder_prev_iq;
    int64_t encoder_accum_i64;
    uint32_t encoder_ppr_u32;
    /* latest_raw is the unfiltered ADC sample per channel — used by
     * calibration AND by the optional analog encoder, which must not
     * see filtered position. */
    int32_t latest_raw[4];
    /* filtered_count is bq[i] output mapped back to count units. */
    int32_t filtered_count[4];
    esp_foc_biquad_q16_t bq[4];
    float offsets[4];
    esp_foc_isensor_t interface;

    int number_of_channels;
    int number_of_conversions;
    isensor_callback_t callback;
    void *user_data;

    bool enable_analog_encoder;
    float analog_encoder_ppr;
    float encoder_accumulated;
    float encoder_previous;
    uint16_t encoder_zero_offset;
    esp_foc_rotor_sensor_t encoder_interface;

}isensor_adc_t;

#define ISENSOR_HWREG(x)    (*((volatile uint32_t *)(x)))
#define ISENSOR_LP_ADC 0x50127000
#define ISENSOR_LP_ADC_INT_CLR_OFFSET     0x0054
#define ISENSOR_LP_ADC_INT_ENA_SET_OFFSET 0x0058
#define ISENSOR_LP_ADC_READER1_CTRL_OFFSET 0x0000

DRAM_ATTR static isensor_adc_t isensor_adc;
static bool adc_initialized = false;
static const float adc_to_volts = ((3.1f)/ 4096.0f);

static adc_oneshot_hal_chan_cfg_t chan_cfg = {
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
};

static adc_oneshot_hal_cfg_t hal_cfg = {
    .work_mode = ADC_HAL_LP_MODE,
    .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
    .clk_src_freq_hz = 0,
};

static void oneshot_adc_start_sample(isensor_adc_t *isensor, adc_channel_t channel, adc_unit_t unit);

static void isensor_adc_isr(void *arg)
{
    isensor_adc_t * isensor = (isensor_adc_t *)arg;
    uint32_t event = (isensor->units[isensor->number_of_conversions] ==  ADC_UNIT_1) ?
                    ADC_LL_EVENT_ADC1_ONESHOT_DONE : ADC_LL_EVENT_ADC2_ONESHOT_DONE;

    /* Clear the interrupts flags */
    ISENSOR_HWREG(ISENSOR_LP_ADC + ISENSOR_LP_ADC_INT_CLR_OFFSET) = 0x7F;

    if(adc_oneshot_ll_get_event(event)) {

        int32_t raw = (int32_t)adc_oneshot_ll_get_raw_result(
                            isensor->units[isensor->number_of_conversions]);
        int ch = isensor->number_of_conversions;
        isensor->latest_raw[ch] = raw;
        /* Only filter the channels that carry currents. When the
         * analog encoder is enabled it lives on channel 2 and must
         * stay raw — otherwise the position read would lag the
         * actual rotor angle. */
        if (isensor->enable_analog_encoder && ch == 2) {
            isensor->filtered_count[ch] = raw;
        } else {
            q16_t y = esp_foc_biquad_q16_update(&isensor->bq[ch],
                                                (q16_t)(raw << 16));
            isensor->filtered_count[ch] = y >> 16;
        }

        isensor->number_of_conversions++;
        if(isensor->number_of_conversions == isensor->number_of_channels) {
            isensor->number_of_conversions = 0;
            adc_oneshot_ll_clear_event(event);
            adc_oneshot_ll_disable_all_unit();
            if(isensor->callback) {
                isensor->callback(isensor->user_data);
            }
        } else {
            adc_oneshot_ll_clear_event(event);
            oneshot_adc_start_sample(isensor,
                                    isensor->channels[isensor->number_of_conversions],
                                    isensor->units[isensor->number_of_conversions]);
        }
    } else {
        adc_oneshot_ll_clear_event(event);
    }
}

static int8_t adc_get_io_num(adc_unit_t adc_unit, uint8_t adc_channel)
{
    assert(adc_unit < SOC_ADC_PERIPH_NUM);
    uint8_t adc_n = (adc_unit == ADC_UNIT_1) ? 0 : 1;
    return adc_channel_io_map[adc_n][adc_channel];
}

static esp_err_t adc_gpio_init(adc_unit_t adc_unit, uint16_t channel_mask)
{
    esp_err_t ret = ESP_OK;
    uint64_t gpio_mask = 0;
    uint32_t n = 0;
    int8_t io = 0;

    while (channel_mask) {
        if (channel_mask & 0x1) {
            io = adc_get_io_num(adc_unit, n);
            if (io < 0) {
                return ESP_ERR_INVALID_ARG;
            }
            gpio_mask |= BIT64(io);
        }
        channel_mask = channel_mask >> 1;
        n++;
    }

    gpio_config_t cfg = {
        .pin_bit_mask = gpio_mask,
        .mode = GPIO_MODE_DISABLE,
    };
    ret = gpio_config(&cfg);

    return ret;
}


static void oneshot_adc_init(isensor_adc_t *isensor)
{
    for(int i = 0; i < isensor->number_of_channels; i++) {
        uint16_t adc_mask = (1 << isensor->channels[i]);
        adc_gpio_init(isensor->units[i], adc_mask);
    }

    ESP_LOGI(TAG, "interrupt source is :%"PRIx8, ETS_LP_ADC_INTR_SOURCE);
    /* And allocate the LP_ADC interrupt vector */
    esp_err_t sts = esp_intr_alloc(ETS_LP_ADC_INTR_SOURCE, ESP_INTR_FLAG_IRAM,
        (intr_handler_t)isensor_adc_isr, (void *)isensor,NULL);

    ESP_ERROR_CHECK(sts);

    esp_foc_sleep_ms(10);
}

static void oneshot_adc_start_sample(isensor_adc_t *isensor, adc_channel_t channel, adc_unit_t unit)
{
    hal_cfg.unit = unit;
    adc_oneshot_hal_init(&isensor->hal, &hal_cfg);
    adc_oneshot_hal_channel_config(&isensor->hal, &chan_cfg, channel);
    adc_oneshot_hal_setup(&isensor->hal, channel);
    adc_oneshot_ll_enable(unit);

    /* Tweak LP ADC registers to enable their both interrupts */
    ISENSOR_HWREG(ISENSOR_LP_ADC + ISENSOR_LP_ADC_INT_ENA_SET_OFFSET) = 0x03;

    adc_oneshot_ll_start(unit);

}

static void fetch_isensors(esp_foc_isensor_t *self, isensor_values_t *values)
{
    isensor_adc_t *obj = __containerof(self, isensor_adc_t, interface);
    const int32_t adc_rng = 2048;

    int32_t f0 = obj->filtered_count[0];
    int32_t f1 = obj->filtered_count[1];
    int32_t f2 = obj->filtered_count[2];
    int32_t f3 = obj->filtered_count[3];

    int32_t d0 = f0 - (int32_t)lroundf(obj->offsets[0]);
    int32_t d1 = f1 - (int32_t)lroundf(obj->offsets[1]);
    d0 = esp_foc_clamp_int32(d0, -adc_rng, adc_rng);
    d1 = esp_foc_clamp_int32(d1, -adc_rng, adc_rng);

    q16_t iu0 = q16_mul(esp_foc_q16_from_adc_diff_clamped(d0, adc_rng), obj->adc_to_current_scale_q16);
    q16_t iv0 = q16_mul(esp_foc_q16_from_adc_diff_clamped(d1, adc_rng), obj->adc_to_current_scale_q16);
    q16_t zero = 0;
    q16_t iw0 = q16_sub(zero, q16_add(iu0, iv0));

    values->iu_axis_0 = iu0;
    values->iv_axis_0 = iv0;
    values->iw_axis_0 = iw0;

    if (!obj->enable_analog_encoder) {
        int32_t d2 = f2 - (int32_t)lroundf(obj->offsets[2]);
        int32_t d3 = f3 - (int32_t)lroundf(obj->offsets[3]);
        d2 = esp_foc_clamp_int32(d2, -adc_rng, adc_rng);
        d3 = esp_foc_clamp_int32(d3, -adc_rng, adc_rng);
        q16_t iu1 = q16_mul(esp_foc_q16_from_adc_diff_clamped(d2, adc_rng), obj->adc_to_current_scale_q16);
        q16_t iv1 = q16_mul(esp_foc_q16_from_adc_diff_clamped(d3, adc_rng), obj->adc_to_current_scale_q16);
        q16_t iw1 = q16_sub(zero, q16_add(iu1, iv1));
        values->iu_axis_1 = iu1;
        values->iv_axis_1 = iv1;
        values->iw_axis_1 = iw1;
    } else {
        values->iu_axis_1 = 0;
        values->iv_axis_1 = 0;
        values->iw_axis_1 = 0;
    }
}

static void sample_isensors(esp_foc_isensor_t *self)
{
    isensor_adc_t *obj =
        __containerof(self, isensor_adc_t, interface);
    oneshot_adc_start_sample(obj, obj->channels[0], obj->units[0]);
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

        /* Use the unfiltered latest_raw value here on purpose: see the
         * matching note in current_sensor_adc.c. */
        obj->offsets[0] += ((float)obj->latest_raw[0]);
        obj->offsets[1] += ((float)obj->latest_raw[1]);
        obj->offsets[2] += ((float)obj->latest_raw[2]);
        obj->offsets[3] += ((float)obj->latest_raw[3]);
    }

    obj->offsets[0] /= calibration_rounds;
    obj->offsets[1] /= calibration_rounds;
    obj->offsets[2] /= calibration_rounds;
    obj->offsets[3] /= calibration_rounds;

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

    esp_foc_critical_enter();
    for (int i = 0; i < 4; ++i) {
        if (obj->enable_analog_encoder && i == 2) {
            esp_foc_biquad_q16_set_bypass(&obj->bq[i]);
            continue;
        }
        esp_foc_biquad_butterworth_lpf_design_q16(&obj->bq[i], fc_hz, fs_hz);
    }
    esp_foc_critical_leave();
}

static void set_to_zero(esp_foc_rotor_sensor_t *self)
{
    isensor_adc_t *obj = __containerof(self, isensor_adc_t, encoder_interface);

    sample_isensors(&obj->interface);
    esp_foc_sleep_ms(10);
    obj->encoder_zero_offset = (uint16_t)obj->latest_raw[2];
    obj->encoder_prev_iq = (int32_t)obj->encoder_zero_offset;

    ESP_LOGI(TAG, "Setting %d [ticks] as offset.", obj->encoder_zero_offset);
}

static uint32_t get_counts_per_revolution(esp_foc_rotor_sensor_t *self)
{
    isensor_adc_t *obj = __containerof(self, isensor_adc_t, encoder_interface);
    return obj->encoder_ppr_u32 ? obj->encoder_ppr_u32 : 1u;
}

static q16_t read_counts_encoder(esp_foc_rotor_sensor_t *self)
{
    isensor_adc_t *obj = __containerof(self, isensor_adc_t, encoder_interface);

    uint16_t raw = (uint16_t)obj->latest_raw[2];
    uint32_t ppr = obj->encoder_ppr_u32 ? obj->encoder_ppr_u32 : 1u;
    uint32_t wrap = ppr * 95u / 100u;

    int32_t delta = (int32_t)raw - obj->encoder_prev_iq;
    int32_t ad = delta < 0 ? -delta : delta;

    if ((uint32_t)ad >= wrap) {
        if (delta < 0) {
            obj->encoder_accum_i64 += (int64_t)ppr;
        } else {
            obj->encoder_accum_i64 -= (int64_t)ppr;
        }
    }

    obj->encoder_prev_iq = (int32_t)raw;

    uint32_t cm = (uint32_t)((raw - obj->encoder_zero_offset) & (ppr - 1u));
    return esp_foc_q16_from_counts_mod(cm, ppr);
}

static int64_t read_accumulated_counts_i64_enc(esp_foc_rotor_sensor_t *self)
{
    isensor_adc_t *obj = __containerof(self, isensor_adc_t, encoder_interface);
    return obj->encoder_accum_i64 + (int64_t)obj->encoder_prev_iq;
}

static void encoder_set_simulation_count(esp_foc_rotor_sensor_t *self, q16_t increment_normalized)
{
    isensor_adc_t *obj = __containerof(self, isensor_adc_t, encoder_interface);
    uint32_t ppr = obj->encoder_ppr_u32;
    int64_t dt = ((int64_t)increment_normalized * (int64_t)ppr) >> 31;
    obj->encoder_accum_i64 += dt;
}

esp_foc_isensor_t *isensor_adc_oneshot_new(esp_foc_isensor_adc_oneshot_config_t *config,
                                        esp_foc_rotor_sensor_t **optional_encoder)
{
    if(config->enable_analog_encoder && config->number_of_channels != 3) {
        ESP_LOGE(TAG, "share analog chgannel for encoder requires exact three channels!");
        return NULL;
    }

    if (config->enable_analog_encoder && optional_encoder == NULL) {
        ESP_LOGE(TAG, "Optional analog channel for encoder activated but no storage provided, aborting");
        return NULL;
    }

    if(adc_initialized == true) {
        return &isensor_adc.interface;
    }

    isensor_adc.adc_to_current_scale = adc_to_volts * (1.0f / (config->amp_gain * config->shunt_resistance));
    isensor_adc.adc_to_current_scale_q16 = q16_from_float(2048.0f * isensor_adc.adc_to_current_scale);
    isensor_adc.encoder_prev_iq = 0;
    isensor_adc.encoder_accum_i64 = 0;
    isensor_adc.encoder_ppr_u32 = (uint32_t)(int32_t)config->analog_encoder_ppr;

    isensor_adc.interface.fetch_isensors = fetch_isensors;
    isensor_adc.interface.sample_isensors = sample_isensors;
    isensor_adc.interface.calibrate_isensors = calibrate_isensors;
    isensor_adc.interface.set_isensor_callback = set_callback;
    isensor_adc.interface.set_filter_cutoff = set_filter_cutoff;
    isensor_adc.number_of_channels = config->number_of_channels;
    /* Default to bypass on every channel; axis init dials a real
     * cutoff in via set_filter_cutoff. */
    for (int i = 0; i < 4; ++i) {
        esp_foc_biquad_q16_set_bypass(&isensor_adc.bq[i]);
    }

    isensor_adc.encoder_interface.get_counts_per_revolution = get_counts_per_revolution;
    isensor_adc.encoder_interface.read_counts = read_counts_encoder;
    isensor_adc.encoder_interface.set_to_zero = set_to_zero;
    isensor_adc.encoder_interface.read_accumulated_counts_i64 = read_accumulated_counts_i64_enc;
    isensor_adc.encoder_interface.set_simulation_count = encoder_set_simulation_count;
    isensor_adc.encoder_accumulated = 0.0f;
    isensor_adc.encoder_previous = 0.0f;
    isensor_adc.encoder_zero_offset = 0;
    isensor_adc.enable_analog_encoder = config->enable_analog_encoder;
    isensor_adc.analog_encoder_ppr = config->analog_encoder_ppr;
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

    oneshot_adc_init(&isensor_adc);
    adc_initialized = true;

    if(config->enable_analog_encoder && optional_encoder) {
        *optional_encoder = &isensor_adc.encoder_interface;
    }

    return &isensor_adc.interface;
}
#endif
