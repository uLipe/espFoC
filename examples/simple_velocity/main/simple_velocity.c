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
#include "espFoC/inverter_3pwm_mcpwm.h"
#include "espFoC/current_sensor_adc.h"
#include "espFoC/esp_foc.h"

static const char *TAG = "esp-foc-example";

static esp_foc_inverter_t *inverter;
static esp_foc_inverter_t *inverter;
static esp_foc_isensor_t  *shunts;

static esp_foc_rotor_sensor_t *sensor;
static esp_foc_axis_t axis;
static esp_foc_motor_control_settings_t settings = {
    .motor_pole_pairs = 4,
    .velocity_control_settings.kp = 1.0f,
    .velocity_control_settings.ki = 0.0f,
    .velocity_control_settings.kd = 0.0f,
    .velocity_control_settings.integrator_limit = 100.0f,
    .velocity_control_settings.max_output_value = 4.0f, //conservative setpoint to the current controller
    .torque_control_settings[0].max_output_value = 6.0f, //Uses the max driver voltage allowed as limit
    .natural_direction = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW,
};

static void initialize_foc_drivers(void)
{

    inverter = inverter_3pwm_mpcwm_new(
        CONFIG_FOC_PWM_U_PIN,
        CONFIG_FOC_PWM_V_PIN,
        CONFIG_FOC_PWM_W_PIN,
        CONFIG_FOC_PWM_EN_PIN,
        24.0f,
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

    esp_foc_isensor_adc_config_t shunt_cfg = {
        .axis_channels = {ADC_CHANNEL_7, ADC_CHANNEL_6},
        .amp_gain = 50.0f,
        .shunt_resistance = 0.01f,
        .number_of_channels = 2,
    };

    shunts = isensor_adc_new(&shunt_cfg);
    if(sensor == NULL) {
        ESP_LOGE(TAG, "failed to create the shunt sensor driver, aborting!");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
        abort();
    }
}

void app_main(void)
{
    esp_foc_control_data_t control_data;
    esp_foc_q_voltage uq = {.raw = -6.0f};

    ESP_LOGI(TAG, "Initializing the esp foc motor controller!");

    initialize_foc_drivers();

    esp_foc_initialize_axis(
        &axis,
        inverter,
        sensor,
        shunts,
        settings
    );

    esp_foc_align_axis(&axis);
    esp_foc_run(&axis);

    /* Set velocity by using state vector given by vq+vd */
    esp_foc_set_target_voltage(&axis, (esp_foc_q_voltage){.raw = 0.0}, (esp_foc_d_voltage){.raw = 0.0});
    /* ramp the velocity */
    uq.raw = 1.0f;

    while(1) {
        esp_foc_set_target_voltage(&axis, uq, (esp_foc_d_voltage){.raw = 0.0});
        uq.raw *= -1.0f;
        esp_foc_sleep_ms(200);
        esp_foc_get_control_data(&axis, &control_data);
        ESP_LOGI(TAG, "Estimated speed: %f rad/s, dt: %f s", control_data.speed.raw, control_data.dt.raw);
        esp_foc_set_target_voltage(&axis, (esp_foc_q_voltage){.raw = 0.0}, (esp_foc_d_voltage){.raw = 0.0});
        esp_foc_sleep_ms(200);
    }
}