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

#include "espFoC/esp_foc.h"

#ifdef CONFIG_FOC_TEST_6_PWM
#include "espFoC/inverter_6pwm_mcpwm.h"
#else
#include "espFoC/inverter_3pwm_mcpwm.h"
#endif

static const char *TAG = "esp-foc-example";

static esp_foc_inverter_t *inverter;

static void initialize_foc_drivers(void)
{
#ifdef CONFIG_FOC_TEST_6_PWM
    inverter = inverter_6pwm_mpcwm_new(
        CONFIG_FOC_PWM_U_PIN,
        CONFIG_FOC_PWM_UL_PIN,
        CONFIG_FOC_PWM_V_PIN,
        CONFIG_FOC_PWM_VL_PIN,
        CONFIG_FOC_PWM_W_PIN,
        CONFIG_FOC_PWM_WL_PIN,
        CONFIG_FOC_PWM_EN_PIN,
        24.0f,
        0
    );
#else
    inverter = inverter_3pwm_mpcwm_new(
        CONFIG_FOC_PWM_U_PIN,
        CONFIG_FOC_PWM_V_PIN,
        CONFIG_FOC_PWM_W_PIN,
        CONFIG_FOC_PWM_EN_PIN,
        24.0f,
        0
    );
#endif

    if(inverter == NULL) {
        ESP_LOGE(TAG, "failed to create the inverter driver, aborting!");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
        abort();
    }

}

void app_main(void)
{
    float theta = 0.0f;
    float valpha;
    float vbeta;
    float da, db, dc;

    initialize_foc_drivers();
    inverter->set_voltages(inverter, 0.65f, 0.5f, 0.5f);
    inverter->enable(inverter);
    ESP_LOGI(TAG, "Observe if the motor runs!");
    esp_foc_sleep_ms(500);

    while(1) {
        float e_sin = esp_foc_sine(theta);
        float e_cos = esp_foc_cosine(theta);

        esp_foc_modulate_dq_voltage(e_sin, e_cos,
                                    0.0f, 1.0f,
                                    &valpha, &vbeta,
                                    &da, &db, &dc,
                                    24 / 1.7320508075688772f,
                                    24 / 2.0f,
                                    1.0f / 24.0f);

        inverter->set_voltages(inverter, da, db, dc);
        esp_foc_sleep_ms(10);

        theta -= 0.2f;
        theta = esp_foc_normalize_angle(theta);
    }
}