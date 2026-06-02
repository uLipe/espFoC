/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * Locked-rotor current-sense characterization — bench axis + ESPF scope.
 * Control via espfoc_shell on console UART (when enabled in axis_shell-style builds).
 */

#include "esp_log.h"
#include "esp_err.h"

#include "espFoC/esp_foc.h"
#include "espFoC/stream/esp_foc_scope.h"
#include "espFoC/inverter_6pwm_mcpwm.h"
#include "espFoC/current_sensor_adc.h"
#include "soc/soc_caps.h"
#include "espFoC/utils/esp_foc_q16.h"

static const char *TAG = "isensor_char";

static esp_foc_axis_t s_axis;
static esp_foc_inverter_t *s_inverter;
static esp_foc_isensor_t *s_shunts;

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

static void wire_scope_channels(void)
{
#if defined(CONFIG_ESP_FOC_SCOPE)
    esp_foc_scope_add_channel(&s_axis.u_q.raw, 0);
    esp_foc_scope_add_channel(&s_axis.u_d.raw, 1);
    esp_foc_scope_add_channel(&s_axis.i_q.raw, 2);
    esp_foc_scope_add_channel(&s_axis.i_d.raw, 3);
    esp_foc_scope_add_channel(&s_axis.i_u, 4);
    esp_foc_scope_add_channel(&s_axis.i_v, 5);
    esp_foc_scope_add_channel(&s_axis.latest_i_alpha, 6);
    esp_foc_scope_add_channel(&s_axis.latest_i_beta, 7);
    esp_foc_scope_bind_axis(&s_axis);
    esp_foc_scope_initalize();
#endif
}

static void bench_task(void *arg)
{
    (void)arg;
    while (1) {
        if (s_axis.state == ESP_FOC_AXIS_STATE_BENCH) {
            esp_foc_bench_step(&s_axis);
        }
#if defined(CONFIG_ESP_FOC_SCOPE)
        esp_foc_scope_data_push();
#endif
        esp_foc_sleep_ms(1);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "isensor characterization — lock rotor, arm bench via shell, scope on USB");

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

    esp_foc_isensor_adc_config_t shunt_cfg = {
        .channels = {(adc_channel_t)CONFIG_AXIS_TUNING_ISENSE_CH_U,
                      (adc_channel_t)CONFIG_AXIS_TUNING_ISENSE_CH_V},
        .unit = ADC_UNIT_1,
        .amp_gain = (float)CONFIG_AXIS_TUNING_ISENSE_AMP_GAIN_X100 / 100.0f,
        .shunt_resistance = (float)CONFIG_AXIS_TUNING_ISENSE_SHUNT_MOHM / 1000.0f,
    };
    s_shunts = isensor_adc_new(&shunt_cfg);
    if (s_shunts == NULL) {
        ESP_LOGE(TAG, "current sensor init failed");
        return;
    }
#if SOC_ETM_SUPPORTED
    esp_foc_isensor_adc_etm_config_t etm_cfg = {
        .mcpwm_timer = 0,
        .event = ESP_FOC_ISENSOR_ADC_MCPWM_EVT_TIMER_TEZ,
    };
    esp_foc_isensor_adc_set_etm_source(s_shunts, &etm_cfg);
    esp_foc_isensor_adc_set_trigger(s_shunts, ESP_FOC_ISENSOR_ADC_TRIG_ETM);
#endif

    esp_foc_axis_bench_config_t bench_cfg = {
        .motor = {
            .motor_pole_pairs = CONFIG_AXIS_TUNING_POLE_PAIRS,
            .natural_direction = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW,
            .motor_unit = 0,
        },
        .calibrate_isensor_at_init = true,
        .bench_theta_e = 0,
    };

    if (esp_foc_initialize_axis_bench(&s_axis, s_inverter, s_shunts, &bench_cfg) !=
        ESP_FOC_OK) {
        ESP_LOGE(TAG, "bench axis init failed");
        return;
    }

    wire_scope_channels();

    if (esp_foc_task_spawn(bench_task, NULL, 4096, 5, NULL) != 0) {
        ESP_LOGE(TAG, "bench task spawn failed");
        return;
    }

    ESP_LOGI(TAG, "lock rotor; arm bench (shell: 0 run); scope when BENCH active");

    while (1) {
        esp_foc_sleep_ms(500);
    }
}
