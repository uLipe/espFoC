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

#include <sys/cdefs.h>
#include "espFoC/esp_foc.h"

#define ESP_FOC_SERVO_SPEED_CONTROL_RATE 10
#define ESP_FOC_FROM_RPM_TO_RADS    0.104719755f

static void on_regulation_callback(esp_foc_axis_t *axis,
                                esp_foc_d_current *id_ref,
                                esp_foc_q_current *iq_ref,
                                esp_foc_d_voltage *ud_forward,
                                esp_foc_q_voltage *uq_forward)
{
    esp_foc_servo_speed_ctl_t *servo = __containerof(axis, esp_foc_servo_speed_ctl_t, target_motor);
    if(servo->downsampling) {
        servo->downsampling--;
        if(!servo->downsampling) {

            float current_speed = servo->target_motor.current_speed;

            id_ref->raw = 0.0f;
            iq_ref->raw = esp_foc_pid_update(&servo->control_law,
                                        servo->target_speed_rads,
                                        current_speed);

            if(servo->regulator_cb) {
                servo->regulator_cb(servo,
                                    &servo->target_speed_rads,
                                    current_speed,
                                    servo->dt,
                                    servo->inv_dt,
                                    &uq_forward->raw,
                                    &servo->control_law);
            }

            servo->downsampling = ESP_FOC_SERVO_SPEED_CONTROL_RATE;
        }
    }
}



esp_foc_err_t esp_foc_servo_speed_init(esp_foc_servo_speed_ctl_t *servo, esp_foc_servo_speed_config_t *config)
{
    if(!servo || !config) {
        return ESP_FOC_ERR_INVALID_ARG;
    }

    servo->target_speed_rads = 0.0f;
    servo->current_speed_ticks_per_sec = 0.0f;
    servo->current_speed_rads = 0.0f;
    servo->regulator_cb = NULL;
    servo->feedforward = 0.0f;
    servo->downsampling = ESP_FOC_SERVO_SPEED_CONTROL_RATE;

    /*Make it invalid until the user tunes it*/
    servo->control_law.dt = -1.0f;
    servo->control_law.inv_dt = -1.0f;

    return esp_foc_initialize_axis(&servo->target_motor, config->inverter, config->rotorsensor,
                                                config->isensor, config->motor_config);
}

esp_foc_err_t esp_foc_servo_speed_set_regulation_callback(esp_foc_servo_speed_ctl_t *servo,
                                                        esp_foc_servo_speed_regulation_callback_t cb)
{
    if(!servo || (servo && !servo->target_motor.inverter_driver)) {
        return ESP_FOC_ERR_INVALID_ARG;
    }

    esp_foc_critical_enter();
    servo->regulator_cb = cb;
    esp_foc_critical_leave();

    return ESP_FOC_OK;
}

esp_foc_err_t esp_foc_servo_speed_run(esp_foc_servo_speed_ctl_t *servo)
{
    if(!servo || (servo && !servo->target_motor.inverter_driver)) {
        return ESP_FOC_ERR_INVALID_ARG;
    }

    esp_foc_err_t err = esp_foc_align_axis(&servo->target_motor);
    if(err != ESP_FOC_OK) {
        return err;
    }

    err = esp_foc_set_regulation_callback(&servo->target_motor, on_regulation_callback);
    if(err != ESP_FOC_OK) {
        return err;
    }

    float master_rate = servo->target_motor.torque_controller[0].dt;
    servo->dt = master_rate * ESP_FOC_SERVO_SPEED_CONTROL_RATE;
    servo->inv_dt = 1.0f / (master_rate * ESP_FOC_SERVO_SPEED_CONTROL_RATE);

    return esp_foc_run(&servo->target_motor);
}
