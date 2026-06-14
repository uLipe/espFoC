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
#include "espFoC/drivers/esp_foc_inverter_mcpwm.h"

static const char *TAG = "esp-foc-example";

static esp_foc_inverter_t *inverter;

static int mcpwm_enable_gpio_from_kconfig(void)
{
    if (CONFIG_FOC_PWM_EN_PIN < 0) {
        return -1;
    }
#ifdef CONFIG_FOC_PWM_EN_ACT_LOW
    if (CONFIG_FOC_PWM_EN_ACT_LOW) {
        return -CONFIG_FOC_PWM_EN_PIN;
    }
#endif
    return CONFIG_FOC_PWM_EN_PIN;
}

static void initialize_foc_drivers(void)
{
    const int en = mcpwm_enable_gpio_from_kconfig();
#ifdef CONFIG_FOC_TEST_6_PWM
    inverter = esp_foc_inverter_mcpwm_6pwm_new(
        CONFIG_FOC_PWM_U_PIN,
        CONFIG_FOC_PWM_UL_PIN,
        CONFIG_FOC_PWM_V_PIN,
        CONFIG_FOC_PWM_VL_PIN,
        CONFIG_FOC_PWM_W_PIN,
        CONFIG_FOC_PWM_WL_PIN,
        en,
        24.0f,
        0,
        ADC_CHANNEL_1,
        ADC_CHANNEL_5,
        50.0f,
        0.01f);
#else
    inverter = esp_foc_inverter_mcpwm_3pwm_new(
        CONFIG_FOC_PWM_U_PIN,
        CONFIG_FOC_PWM_V_PIN,
        CONFIG_FOC_PWM_W_PIN,
        en,
        24.0f,
        0,
        ADC_CHANNEL_1,
        ADC_CHANNEL_5,
        50.0f,
        0.01f);
#endif

    if (inverter == NULL) {
        ESP_LOGE(TAG, "failed to create the inverter driver, aborting!");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
        abort();
    }
}

void app_main(void)
{
    q16_t theta = 0;
    const q16_t vmax = ESP_FOC_MOD_INDEX_LIMIT_Q16;
    const q16_t step = q16_mul(q16_from_float(0.2f), Q16_INV_TWO_PI);

    initialize_foc_drivers();
    inverter->set_duties(inverter, 0, 0, 0);
    inverter->enable(inverter);
    ESP_LOGI(TAG, "Observe if the motor runs!");
    esp_foc_sleep_ms(500);

    while (1) {
        q16_t e_sin = q16_sin(theta);
        q16_t e_cos = q16_cos(theta);

        q16_t valpha, vbeta, da, db, dc;
        esp_foc_modulate_dq_to_duties(e_sin, e_cos,
                                      0, ESP_FOC_VPU_ONE_Q16,
                                      &valpha, &vbeta,
                                      &da, &db, &dc,
                                      vmax);

        inverter->set_duties(inverter, da, db, dc);
        esp_foc_sleep_ms(10);

        theta = q16_normalize_angle(q16_sub(theta, step));
    }
}
