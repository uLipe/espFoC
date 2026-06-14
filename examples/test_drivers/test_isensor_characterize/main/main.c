/*
 * Locked-rotor current-sense characterization — bench axis + console shell.
 */

#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "espFoC/esp_foc.h"
#include "espFoC/esp_foc_inverter_mcpwm.h"

static const char *TAG = "isensor_char";

static esp_foc_axis_t s_axis;
static esp_foc_inverter_t *s_inverter;

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

static void bench_task(void *arg)
{
    (void)arg;
    while (1) {
        if (s_axis.state == ESP_FOC_AXIS_STATE_BENCH) {
            esp_foc_bench_step(&s_axis);
        }
        esp_foc_sleep_ms(1);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "isensor characterization — lock rotor, arm bench via shell");

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

    esp_foc_axis_bench_config_t bench_cfg = {
        .motor = {
            .motor_pole_pairs = CONFIG_AXIS_TUNING_POLE_PAIRS,
            .natural_direction = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW,
            .motor_unit = 0,
        },
        .calibrate_isensor_at_init = true,
        .bench_theta_e = 0,
    };

    if (esp_foc_initialize_axis_bench(&s_axis, s_inverter, &bench_cfg) !=
        ESP_FOC_OK) {
        ESP_LOGE(TAG, "bench axis init failed");
        return;
    }

    xTaskCreate(bench_task, "bench", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "lock rotor; arm bench (shell: bench arm)");
    while (1) {
        esp_foc_sleep_ms(1000);
    }
}
