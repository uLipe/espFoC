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

#define ESP_FOC_VELOCITY_PID_DOWNSAMPLING  1
#define ESP_FOC_POSITION_PID_DOWNSAMPLING  10
#define ESP_FOC_ESTIMATORS_DOWNSAMPLING    4
#define ESP_FOC_LOW_SPEED_DOWNSAMPLING     10
#define ESP_FOC_PWM_RATE_HZ                20000
#define ESP_FOC_ISENSOR_CALIBRATION_ROUNDS 100
#define ESP_FOC_PLL_BANDWIDTH_HZ 50.0f
#define ESP_FOC_PLL_ZETA 0.707f


static inline void esp_foc_current_control_loop(esp_foc_axis_t *axis)
{

    axis->u_q.raw =  esp_foc_pid_update( &axis->torque_controller[0],
                                        axis->target_i_q.raw,
                                        esp_foc_low_pass_filter_update(
                                            &axis->current_filters[0], axis->i_q.raw));

    axis->u_d.raw = esp_foc_pid_update( &axis->torque_controller[1],
                                        axis->target_i_d.raw,
                                        esp_foc_low_pass_filter_update(
                                            &axis->current_filters[1], axis->i_d.raw));
}

static inline void esp_foc_position_control_loop(esp_foc_axis_t *axis)
{
    /* position control is disabled */
    if(!axis->downsampling_position) return;

    axis->downsampling_position--;

    if(axis->downsampling_position == 0) {
        axis->downsampling_position = ESP_FOC_POSITION_PID_DOWNSAMPLING;

        axis->target_speed = esp_foc_pid_update( &axis->position_controller,
                                            axis->target_position,
                                            axis->rotor_position);
    }
}

static inline void esp_foc_velocity_control_loop(esp_foc_axis_t *axis)
{
    /* speed control is disabled */
    if(!axis->downsampling_speed) return;

    axis->downsampling_speed--;

    if(axis->downsampling_speed == 0) {

        axis->downsampling_speed = ESP_FOC_VELOCITY_PID_DOWNSAMPLING;

        axis->target_i_q.raw =  esp_foc_pid_update( &axis->velocity_controller,
                                            axis->target_speed,
                                            axis->current_speed) ;
        axis->target_i_d.raw = (axis->target_i_q.raw < 0.0f) ? (axis->target_i_q.raw * 0.25) : -(axis->target_i_q.raw * 0.25);
    }
}


/* Voltage mode core controllers */
void do_voltage_mode_sensored_high_speed_loop(void *arg);
void do_voltage_mode_sensored_low_speed_loop(void *arg);

/* current mode sensored core controllers */
void do_current_mode_sensored_high_speed_loop(void *arg);
void do_current_mode_sensored_low_speed_loop(void *arg);

/* current mode sensorless core controllers */
void do_current_mode_sensorless_high_speed_loop(void *arg);
void do_current_mode_sensorless_low_speed_loop(void *arg);
