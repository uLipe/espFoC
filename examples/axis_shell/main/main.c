/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * axis_shell — FOC axis with console shell (UART).
 */

#include "esp_log.h"
#include "esp_err.h"

#include "espFoC/esp_foc.h"
#include "espFoC/shell/espfoc_shell.h"
#include "espFoC/utils/esp_foc_q16.h"

#if defined(CONFIG_ESP_FOC_FITL) && CONFIG_ESP_FOC_FITL
#include "espFoC/esp_foc_in_the_loop.h"
#else
#include "espFoC/esp_foc_inverter_mcpwm.h"
#if defined(CONFIG_AXIS_TUNING_ROTOR_AS5600)
#include "espFoC/esp_foc_encoder_as5600.h"
#elif defined(CONFIG_AXIS_TUNING_ROTOR_SIMU)
#include "espFoC/esp_foc_encoder_simu.h"
#endif
#endif

static const char *TAG = "axis_shell";

static esp_foc_axis_t s_axis;
static esp_foc_inverter_t *s_inverter;
static esp_foc_encoder_t *s_encoder;

static void regulation_stub(esp_foc_axis_t *axis,
                            esp_foc_d_current_q16_t *id_ref,
                            esp_foc_q_current_q16_t *iq_ref)
{
    (void)axis;
    (void)id_ref;
    (void)iq_ref;
}

#if !defined(CONFIG_ESP_FOC_FITL) || !CONFIG_ESP_FOC_FITL
static int pwm_enable_gpio(void)
{
    if (CONFIG_AXIS_TUNING_PWM_EN_PIN < 0) {
        return -1;
    }
#ifdef CONFIG_AXIS_TUNING_PWM_EN_ACT_LOW
    if (CONFIG_AXIS_TUNING_PWM_EN_ACT_LOW) {
        return -CONFIG_AXIS_TUNING_PWM_EN_PIN;
    }
#endif
    return CONFIG_AXIS_TUNING_PWM_EN_PIN;
}
#endif

void app_main(void)
{
    ESP_LOGI(TAG, "boot — axis_shell");

#if defined(CONFIG_ESP_FOC_FITL) && CONFIG_ESP_FOC_FITL
    esp_foc_in_the_loop_config_t fitl_cfg = {
        .r_ohm = (float)CONFIG_FOC_ITL_DEFAULT_R_MILLIOHM / 1000.0f,
        .l_henry = (float)CONFIG_FOC_ITL_DEFAULT_L_UH / 1000000.0f,
        .j_kgm2 = (float)CONFIG_FOC_ITL_DEFAULT_J_X1E7 / 1.0e7f,
        .b_nms = (float)CONFIG_FOC_ITL_DEFAULT_B_X1E7 / 1.0e7f,
        .kt_nm_per_a = (float)CONFIG_FOC_ITL_DEFAULT_KT_X1E7 / 1.0e7f,
        .pole_pairs = CONFIG_AXIS_TUNING_POLE_PAIRS,
        .vdc_q16 = q16_from_float((float)CONFIG_AXIS_TUNING_DC_LINK_V),
        .i_max_a = (float)CONFIG_FOC_ITL_DEFAULT_I_MAX_MA / 1000.0f,
        .pwm_hz = 0,
    };
    esp_foc_in_the_loop_handles_t fitl = {0};
    if (esp_foc_in_the_loop_create(&fitl_cfg, &fitl) != ESP_FOC_OK) {
        ESP_LOGE(TAG, "FITL init failed");
        return;
    }
    s_inverter = fitl.inverter;
    s_encoder = fitl.encoder;
#else
    s_inverter = esp_foc_inverter_mcpwm_6pwm_new(
        CONFIG_AXIS_TUNING_PWM_U_HI,
        CONFIG_AXIS_TUNING_PWM_U_LO,
        CONFIG_AXIS_TUNING_PWM_V_HI,
        CONFIG_AXIS_TUNING_PWM_V_LO,
        CONFIG_AXIS_TUNING_PWM_W_HI,
        CONFIG_AXIS_TUNING_PWM_W_LO,
        pwm_enable_gpio(),
        (float)CONFIG_AXIS_TUNING_DC_LINK_V,
        0,
        (adc_channel_t)CONFIG_AXIS_TUNING_ISENSE_CH_U,
        (adc_channel_t)CONFIG_AXIS_TUNING_ISENSE_CH_V,
        (float)CONFIG_AXIS_TUNING_ISENSE_AMP_GAIN_X100 / 100.0f,
        (float)CONFIG_AXIS_TUNING_ISENSE_SHUNT_MOHM / 1000.0f);
    if (s_inverter == NULL) {
        ESP_LOGE(TAG, "inverter init failed");
        return;
    }

#if defined(CONFIG_AXIS_TUNING_ROTOR_AS5600)
    s_encoder = esp_foc_encoder_as5600_new(
        CONFIG_AXIS_TUNING_ENC_SDA,
        CONFIG_AXIS_TUNING_ENC_SCL,
        0);
#elif defined(CONFIG_AXIS_TUNING_ROTOR_SIMU)
    q16_t vdc = q16_from_float((float)CONFIG_AXIS_TUNING_DC_LINK_V);
    s_encoder = esp_foc_encoder_simu_new(
        0,
        CONFIG_AXIS_TUNING_POLE_PAIRS,
        (float)CONFIG_AXIS_TUNING_SIMU_R_MILLIOHM / 1000.0f,
        (float)CONFIG_AXIS_TUNING_SIMU_L_MICROHENRY / 1000000.0f,
        vdc);
#endif
    if (s_encoder == NULL) {
        ESP_LOGE(TAG, "encoder init failed");
        return;
    }
#endif

    esp_foc_motor_control_settings_t settings = {
        .motor_pole_pairs = CONFIG_AXIS_TUNING_POLE_PAIRS,
        .natural_direction = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW,
        .motor_unit = 0,
    };

    if (esp_foc_initialize_axis(&s_axis, s_inverter, s_encoder, settings) != ESP_FOC_OK) {
        ESP_LOGE(TAG, "axis init failed");
        return;
    }

#if defined(CONFIG_AXIS_TUNING_ROTOR_SIMU) && (!defined(CONFIG_ESP_FOC_FITL) || !CONFIG_ESP_FOC_FITL)
    esp_foc_encoder_simu_wire_ud_uq(s_encoder, &s_axis.u_d.raw, &s_axis.u_q.raw);
#endif

    esp_foc_set_regulation_callback(&s_axis, regulation_stub);

#if defined(CONFIG_ESPFOC_SHELL)
    ESP_ERROR_CHECK(espfoc_shell_register_axis(0, &s_axis));
    espfoc_shell_start();
#endif

    ESP_LOGI(TAG, "shell on UART (idf monitor)");

    while (1) {
        esp_foc_sleep_ms(500);
    }
}
