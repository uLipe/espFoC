/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * axis_tuning — reference firmware for live PI tuning over the espFoC
 * tuner link. Boots, auto-loads NVS calibration when present, attaches
 * the axis to the runtime tuner, and waits for the host (align / run /
 * stop / target writes / store / erase).
 */

#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "espFoC/esp_foc.h"
#include "espFoC/gui_link/esp_foc_tuner.h"
#include "espFoC/inverter_6pwm_mcpwm.h"
#if defined(CONFIG_AXIS_TUNING_ROTOR_AS5600)
#include "espFoC/rotor_sensor_as5600.h"
#elif defined(CONFIG_AXIS_TUNING_ROTOR_SIMU)
#include "espFoC/rotor_sensor_simu.h"
#endif
#if defined(CONFIG_IDF_TARGET_ESP32P4)
#include "espFoC/current_sensor_adc_one_shot.h"
#else
#include "espFoC/current_sensor_adc.h"
#endif
#include "espFoC/utils/esp_foc_q16.h"

static const char *TAG = "axis_tuning";

static esp_foc_axis_t s_axis;
static esp_foc_inverter_t *s_inverter;
static esp_foc_rotor_sensor_t *s_rotor;
static esp_foc_isensor_t *s_shunts;

uint32_t esp_foc_tuner_firmware_type(void)
{
    return ESP_FOC_TUNER_FIRMWARE_TYPE_TSGX;
}

static void regulation_stub(esp_foc_axis_t *axis,
                              esp_foc_d_current_q16_t *id_ref,
                              esp_foc_q_current_q16_t *iq_ref)
{
    (void)axis;
    (void)id_ref;
    (void)iq_ref;
}

static void wire_scope_channels(void)
{
#if defined(CONFIG_ESP_FOC_SCOPE)
    /* Group 0–3: current / torque */
    esp_foc_scope_add_channel(&s_axis.target_i_d.raw, 0);
    esp_foc_scope_add_channel(&s_axis.i_d.raw, 1);
    esp_foc_scope_add_channel(&s_axis.target_i_q.raw, 2);
    esp_foc_scope_add_channel(&s_axis.i_q.raw, 3);
    /* Group 4–5: dq voltage */
    esp_foc_scope_add_channel(&s_axis.u_d.raw, 4);
    esp_foc_scope_add_channel(&s_axis.u_q.raw, 5);
    /* Group 6–9: encoder PLL */
    esp_foc_scope_add_channel(
        (const q16_t *)(const volatile void *)&s_axis.rotor_estimator.theta_meas_mech, 6);
    esp_foc_scope_add_channel(&s_axis.rotor_estimator.theta_est_mech, 7);
    esp_foc_scope_add_channel(&s_axis.rotor_estimator.omega_est_mech, 8);
    esp_foc_scope_add_channel(&s_axis.rotor_estimator.pll_err, 9);
    /* Group 10–12: phase / Clarke */
    esp_foc_scope_add_channel(&s_axis.i_u, 10);
    esp_foc_scope_add_channel(&s_axis.i_v, 11);
    esp_foc_scope_add_channel(&s_axis.i_alpha.raw, 12);
    /* Group 13: ISR hot-path timing */
    esp_foc_scope_add_channel(&esp_foc_debug_scope_hot_path_dt_us_q16, 13);
    esp_foc_scope_initalize();
#endif
}

#if defined(CONFIG_ESP_FOC_SCOPE)
static void pump_scope_idle(void)
{
    for (int n = 0; n < CONFIG_ESP_FOC_SCOPE_BUFFER_SIZE; n++) {
        esp_foc_scope_data_push();
    }
}
#endif

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

void app_main(void)
{
    ESP_LOGI(TAG, "boot — axis_tuning");

    s_inverter = inverter_6pwm_mpcwm_new(
        CONFIG_AXIS_TUNING_PWM_U_HI,
        CONFIG_AXIS_TUNING_PWM_U_LO,
        CONFIG_AXIS_TUNING_PWM_V_HI,
        CONFIG_AXIS_TUNING_PWM_V_LO,
        CONFIG_AXIS_TUNING_PWM_W_HI,
        CONFIG_AXIS_TUNING_PWM_W_LO,
        pwm_enable_gpio(),
        (float)CONFIG_AXIS_TUNING_DC_LINK_V,
        0);
    if (s_inverter == NULL) {
        ESP_LOGE(TAG, "inverter init failed");
        return;
    }

#if defined(CONFIG_AXIS_TUNING_ROTOR_AS5600)
    s_rotor = rotor_sensor_as5600_new(
        CONFIG_AXIS_TUNING_ENC_SDA,
        CONFIG_AXIS_TUNING_ENC_SCL,
        0);
#elif defined(CONFIG_AXIS_TUNING_ROTOR_SIMU)
    q16_t vdc = q16_from_float((float)CONFIG_AXIS_TUNING_DC_LINK_V);
    s_rotor = rotor_sensor_simu_new(
        0,
        CONFIG_AXIS_TUNING_POLE_PAIRS,
        (float)CONFIG_AXIS_TUNING_SIMU_R_MILLIOHM / 1000.0f,
        (float)CONFIG_AXIS_TUNING_SIMU_L_MICROHENRY / 1000000.0f,
        vdc);
#endif
    if (s_rotor == NULL) {
        ESP_LOGE(TAG, "rotor sensor init failed");
        return;
    }

#if defined(CONFIG_IDF_TARGET_ESP32P4)
    esp_foc_isensor_adc_oneshot_config_t shunt_cfg = {
        .axis_channels = {(adc_channel_t)CONFIG_AXIS_TUNING_ISENSE_CH_U,
                          (adc_channel_t)CONFIG_AXIS_TUNING_ISENSE_CH_V},
        .units         = {(adc_unit_t)(CONFIG_AXIS_TUNING_ISENSE_ADC_UNIT - 1),
                          (adc_unit_t)(CONFIG_AXIS_TUNING_ISENSE_ADC_UNIT - 1)},
        .amp_gain      = (float)CONFIG_AXIS_TUNING_ISENSE_AMP_GAIN_X100 / 100.0f,
        .shunt_resistance = (float)CONFIG_AXIS_TUNING_ISENSE_SHUNT_MOHM / 1000.0f,
        .number_of_channels = 2,
        .enable_analog_encoder = false,
    };
    s_shunts = isensor_adc_oneshot_new(&shunt_cfg, NULL);
#else
    esp_foc_isensor_adc_config_t shunt_cfg = {
        .channels = {(adc_channel_t)CONFIG_AXIS_TUNING_ISENSE_CH_U,
                      (adc_channel_t)CONFIG_AXIS_TUNING_ISENSE_CH_V},
        .unit = ADC_UNIT_1,
        .amp_gain = (float)CONFIG_AXIS_TUNING_ISENSE_AMP_GAIN_X100 / 100.0f,
        .shunt_resistance = (float)CONFIG_AXIS_TUNING_ISENSE_SHUNT_MOHM / 1000.0f,
    };
    s_shunts = isensor_adc_new(&shunt_cfg);
#endif
    if (s_shunts == NULL) {
        ESP_LOGE(TAG, "current sensor init failed");
        return;
    }

    esp_foc_motor_control_settings_t settings = {
        .motor_pole_pairs  = CONFIG_AXIS_TUNING_POLE_PAIRS,
        .natural_direction = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW,
        .motor_unit        = 0,
    };

    esp_foc_err_t err = esp_foc_initialize_axis(&s_axis, s_inverter,
                                                s_rotor, s_shunts, settings);
    if (err != ESP_FOC_OK) {
        ESP_LOGE(TAG, "axis init failed: %d", err);
        return;
    }

#if defined(CONFIG_AXIS_TUNING_ROTOR_SIMU)
    rotor_sensor_simu_wire_ud_uq(s_rotor, &s_axis.u_d.raw, &s_axis.u_q.raw);
#endif

    wire_scope_channels();
#if defined(CONFIG_ESP_FOC_SCOPE)
    pump_scope_idle();
#endif

    esp_foc_set_regulation_callback(&s_axis, regulation_stub);
    ESP_ERROR_CHECK(esp_foc_tuner_attach_axis(0, &s_axis));

    ESP_LOGI(TAG, "ready — connect host and run: align → run → tune");

    while (1) {
#if defined(CONFIG_ESP_FOC_SCOPE)
        if (s_axis.state != ESP_FOC_AXIS_STATE_RUNNING) {
            pump_scope_idle();
        }
#endif
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
