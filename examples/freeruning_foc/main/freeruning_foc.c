#include <stdio.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "espFoC/rotor_sensor_analog.h"
#include "espFoC/inverter_3pwm_ledc.h"
#include "espFoC/esp_foc.h"

static const char *TAG = "esp-foc-example";

static esp_foc_inverter_t *inverter;
static esp_foc_rotor_sensor_t *sensor;
static esp_foc_axis_t axis;

void app_main(void)
{
    esp_foc_d_voltage vd;
    esp_foc_q_voltage vq;

    ESP_LOGI(TAG, "Initializing the esp foc motor controller!");

    inverter_3pwm_ledc_init();
    rotor_sensor_analog_init();

    inverter = inverter_3pwm_ledc_new(
        LEDC_CHANNEL_0,
        LEDC_CHANNEL_1,
        LEDC_CHANNEL_2,
        CONFIG_FOC_PWM_U_PIN,
        CONFIG_FOC_PWM_V_PIN,
        CONFIG_FOC_PWM_W_PIN,
        12.0f,
        0
    );

    if(inverter == NULL) {
        ESP_LOGE(TAG, "failed to create the inverter driver, aborting!");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }

    sensor = rotor_sensor_analog_new(
        ADC_CHANNEL_3, 
        CONFIG_FOC_SENSOR_COUNT_MIN,
        CONFIG_FOC_SENSOR_COUNT_MAX,
        0
    );

    if(sensor == NULL) {
        ESP_LOGE(TAG, "failed to create the inverter driver, aborting!");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }

    esp_foc_initialize_axis(
        &axis,
        inverter,
        sensor,
        CONFIG_FOC_DEFAULT_MOTOR_POLE_PAIRS
    );

    vd.raw = 0.0f;
    vq.raw = 1.0f;

    esp_foc_align_axis(&axis);
    esp_foc_set_target_voltage(&axis, &vq, &vd);
    esp_foc_run(&axis);
}


