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

#pragma once

typedef struct esp_foc_servo_speed_ctl_s esp_foc_servo_speed_ctl_t;
typedef void (*esp_foc_servo_speed_regulation_callback_t) (esp_foc_servo_speed_ctl_t *controller, float current_rate);

struct esp_foc_servo_speed_ctl_s {
    float target_speed_rpm;
    float current_speed_rpm;
    float current_speed_ticks_per_sec;
    esp_foc_pid_controller_t control_law;
    esp_foc_axis_t target_motor;
    esp_foc_servo_speed_regulation_callback_t regulator_cb;
};

typedef struct {
    esp_foc_motor_control_settings_t motor_config;
    esp_foc_isensor_t *isensor;
    esp_foc_rotor_sensor_t *rotorsensor;
    esp_foc_inverter_t *inverter;
}esp_foc_servo_speed_config_t;

esp_foc_err_t esp_foc_servo_speed_init(esp_foc_servo_speed_ctl_t *servo, esp_foc_servo_speed_config_t *config);
esp_foc_err_t esp_foc_servo_speed_run(esp_foc_servo_speed_ctl_t *servo);
esp_foc_err_t esp_foc_servo_speed_set_regulation_callback(esp_foc_servo_speed_ctl_t *servo,
                                                        esp_foc_servo_speed_regulation_callback_t cb);

