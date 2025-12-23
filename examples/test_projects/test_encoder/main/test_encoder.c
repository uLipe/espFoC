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

#include "espFoC/rotor_sensor_as5600.h"
#include "espFoC/esp_foc.h"

static const char *TAG = "esp-foc-example";

static esp_foc_rotor_sensor_t *sensor;

static void initialize_foc_drivers(void)
{
    sensor = rotor_sensor_as5600_new(
        CONFIG_FOC_ENCODER_SDA_PIN,
        CONFIG_FOC_ENCODER_SCL_PIN,
        0
    );

    if(sensor == NULL) {
        ESP_LOGE(TAG, "failed to create the inverter driver, aborting!");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
        abort();
    }
}

void app_main(void)
{
    initialize_foc_drivers();

    while(1) {
        esp_foc_sleep_ms(100);
        ESP_LOGI(TAG, "Turn the motor axis by Hand! Encoder reding raw:  %f", sensor->read_counts(sensor));
    }
}