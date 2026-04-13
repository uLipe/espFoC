/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#include "esp_log.h"
#include "esp_err.h"

#include "espFoC/esp_foc.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/foc_math_q16.h"
#include "espFoC/utils/modulator.h"

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
    q16_t theta = 0;
    const q16_t vbus = q16_from_float(24.0f);
    const q16_t vmax = q16_from_float(24.0f / 1.7320508075688772f);
    const q16_t inv_vbus = q16_from_float(1.0f / 24.0f);
    const q16_t step = q16_from_float(0.2f);

    initialize_foc_drivers();
    inverter->set_voltages(inverter, q16_from_float(0.65f), q16_from_float(0.5f), q16_from_float(0.5f));
    inverter->enable(inverter);
    ESP_LOGI(TAG, "Observe if the motor runs!");
    esp_foc_sleep_ms(500);

    while(1) {
        q16_t e_sin = q16_sin(theta);
        q16_t e_cos = q16_cos(theta);

        q16_t valpha, vbeta, da, db, dc;
        esp_foc_modulate_dq_voltage(e_sin, e_cos,
                                    0, q16_from_float(1.0f),
                                    &valpha, &vbeta,
                                    &da, &db, &dc,
                                    vmax,
                                    q16_mul(vbus, Q16_HALF),
                                    inv_vbus);

        inverter->set_voltages(inverter, da, db, dc);
        esp_foc_sleep_ms(10);

        theta = q16_normalize_angle_rad(q16_sub(theta, step));
    }
}
