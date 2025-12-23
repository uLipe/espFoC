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
#include "esp_log.h"
#include "esp_err.h"

#include "espFoC/current_sensor_adc.h"
#include "espFoC/esp_foc.h"

static const char *TAG = "esp-foc-example";

static esp_foc_isensor_t  *shunts;

static void initialize_foc_drivers(void)
{
    esp_foc_isensor_adc_config_t shunt_cfg = {
        .axis_channels = {ADC_CHANNEL_1, ADC_CHANNEL_5},
        .units = {ADC_UNIT_1, ADC_UNIT_1},
        .amp_gain = 50.0f,
        .shunt_resistance = 0.01f,
        .number_of_channels = 2,
    };

    shunts = isensor_adc_new(&shunt_cfg);
    if(shunts == NULL) {
        ESP_LOGE(TAG, "failed to create the shunt sensor driver, aborting!");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
        abort();
    }
}

void app_main(void)
{
    isensor_values_t values;
    initialize_foc_drivers();

    shunts->calibrate_isensors(shunts, 50);

    while(1) {
        shunts->sample_isensors(shunts);
        esp_foc_sleep_ms(100);
        shunts->fetch_isensors(shunts, &values);
        ESP_LOGI(TAG, "phase currents:  %f, %f, %f", values.iu_axis_0,
            values.iv_axis_0,  values.iw_axis_0);
    }
}