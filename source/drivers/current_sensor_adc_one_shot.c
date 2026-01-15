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
#include "espFoC/current_sensor_adc_one_shot.h"
#include "hal/adc_hal.h"
#include "hal/adc_oneshot_hal.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_clk_tree.h"

#ifdef CONFIG_IDF_TARGET_ESP32P4
static const char *TAG = "ESP_FOC_ISENSOR_ONE_SHOT";

typedef struct {
    esp_foc_lp_filter_t current_filters[4];
    adc_channel_t channels[4];
    adc_unit_t units[4];
    adc_oneshot_hal_ctx_t hal;
    float adc_to_current_scale;
    uint32_t raw_reads[4];
    float currents[4];
    float offsets[4];
    esp_foc_isensor_t interface;
    int number_of_channels;
    int number_of_conversions;
    isensor_callback_t callback;
    void *user_data;
}isensor_adc_t;

#define ISENSOR_HWREG(x)    (*((volatile uint32_t *)(x)))
#define ISENSOR_LP_ADC 0x50127000
#define ISENSOR_LP_ADC_INT_ENA_OFFSET 0x004C

DRAM_ATTR static isensor_adc_t isensor_adc;
static bool adc_initialized = false;
static const float adc_to_volts = ((3.9f)/ 4096.0f);

static adc_oneshot_hal_chan_cfg_t chan_cfg = {
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
};

static adc_oneshot_hal_cfg_t hal_cfg = {
    .work_mode = ADC_HAL_LP_MODE,
    .clk_src = LP_ADC_CLK_SRC_LP_DYN_FAST,
    .clk_src_freq_hz = 0,
};

static void oneshot_adc_start_sample(isensor_adc_t *isensor, adc_channel_t channel, adc_unit_t unit);

static void isensor_adc_isr(void *arg)
{
    isensor_adc_t * isensor = (isensor_adc_t *)arg;
    uint32_t event = 0;

    adc_oneshot_ll_get_event(event);
    if((event == ADC_LL_EVENT_ADC1_ONESHOT_DONE) ||
       (event == ADC_LL_EVENT_ADC2_ONESHOT_DONE)) {

        isensor->raw_reads[isensor->number_of_conversions] =
            adc_oneshot_ll_get_raw_result(isensor->hal.unit);

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

static void oneshot_adc_init(isensor_adc_t *isensor)
{
    /* And allocate the LP_ADC interrupt vector */
    esp_intr_alloc(ETS_LP_ADC_INTR_SOURCE, ESP_INTR_FLAG_IRAM,
        (intr_handler_t)isensor_adc_isr, (void *)isensor,NULL);

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
    ISENSOR_HWREG(ISENSOR_LP_ADC + ISENSOR_LP_ADC_INT_ENA_OFFSET) = 0x03;

    adc_oneshot_ll_start(unit);
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


esp_foc_isensor_t *isensor_adc_oneshot_new(esp_foc_isensor_adc_oneshot_config_t *config)
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

    oneshot_adc_init(&isensor_adc);
    adc_initialized = true;

    return &isensor_adc.interface;
}
#endif
