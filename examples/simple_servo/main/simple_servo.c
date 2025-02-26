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
#include "espFoC/inverter_3pwm_ledc.h"
#include "espFoC/esp_foc.h"

static const char *TAG = "esp-foc-example";

static esp_foc_inverter_t *inverter;
static esp_foc_rotor_sensor_t *sensor;
static esp_foc_axis_t axis;
static esp_foc_motor_control_settings_t settings = {
    .downsampling_position_rate = 160, // position control runs 1/4 rate speed control,
    .downsampling_speed_rate = 40, //speed control runs 1/40 torque loop
    .motor_pole_pairs = 7, //Assuming HT2250 motor
    .position_control_settings.kp = 1.8f,
    .position_control_settings.ki = 0.0f,
    .position_control_settings.kd = 0.0f,
    .position_control_settings.integrator_limit = 20000.0f,
    .position_control_settings.max_output_value = 100.0f,

    .velocity_control_settings.kp = 0.1f,
    .velocity_control_settings.ki = 0.008f,
    .velocity_control_settings.kd = 0.0f,
    .velocity_control_settings.integrator_limit = 20000.0f,
    .velocity_control_settings.max_output_value = 6.0f, //conservative setpoint to the current controller
    .torque_control_settings[0].max_output_value = 6.0f, //Uses the max driver voltage allowed as limit
    .natural_direction = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW,
};

static void initialize_foc_drivers(void)
{

    inverter = inverter_3pwm_ledc_new(
        LEDC_CHANNEL_0,
        LEDC_CHANNEL_1,
        LEDC_CHANNEL_2,
        CONFIG_FOC_PWM_U_PIN,
        CONFIG_FOC_PWM_V_PIN,
        CONFIG_FOC_PWM_W_PIN,
        CONFIG_FOC_PWM_EN_PIN,
        12.0f,
        0
    );

    if(inverter == NULL) {
        ESP_LOGE(TAG, "failed to create the inverter driver, aborting!");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
        abort();
    }

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
    esp_foc_control_data_t control_data;

    ESP_LOGI(TAG, "Initializing the esp foc motor controller!");

    initialize_foc_drivers();

    esp_foc_initialize_axis(
        &axis,
        inverter,
        sensor,
        NULL,
        settings
    );

    esp_foc_align_axis(&axis);
    esp_foc_run(&axis);

    while(1) {
        esp_foc_set_target_position(&axis, (esp_foc_radians){.raw = 3.1416});
        esp_foc_sleep_ms(2500);
        esp_foc_get_control_data(&axis, &control_data);
        ESP_LOGI(TAG, "Current mechanical position: %f [rad]", control_data.position.raw);
        esp_foc_set_target_position(&axis, (esp_foc_radians){.raw = 0.0f});
        esp_foc_sleep_ms(2500);
        esp_foc_get_control_data(&axis, &control_data);
        ESP_LOGI(TAG, "Current mechanical position: %f [rad]", control_data.position.raw);
    }
}