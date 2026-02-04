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

#define ESP_FOC_ESTIMATORS_DOWNSAMPLING    4
#define ESP_FOC_LOW_SPEED_DOWNSAMPLING     10
#define ESP_FOC_PWM_RATE_HZ                50000
#define ESP_FOC_ISENSOR_CALIBRATION_ROUNDS 100
#define ESP_FOC_PLL_BANDWIDTH_HZ 200.0f
#define ESP_FOC_PLL_ZETA 0.707f

static inline void esp_foc_current_control_loop(esp_foc_axis_t *axis)
{

    if(!axis->skip_torque_control) {
        axis->u_q.raw =  esp_foc_pid_update( &axis->torque_controller[0],
                                            axis->target_i_q.raw,
                                            esp_foc_low_pass_filter_update(
                                                &axis->current_filters[0], axis->i_q.raw));

        axis->u_d.raw = esp_foc_pid_update( &axis->torque_controller[1],
                                            axis->target_i_d.raw,
                                            esp_foc_low_pass_filter_update(
                                                &axis->current_filters[1], axis->i_d.raw));
    } else {
        axis->u_d.raw = 0.0f;
        axis->u_q.raw = 0.0f;
    }

    /* feed forward the IQ voltages*/
    axis->u_d.raw += axis->target_u_d.raw;
    axis->u_q.raw += axis->target_u_q.raw;
}


/* current mode sensored core controllers */
void do_current_mode_sensored_high_speed_loop(void *arg);
void do_current_mode_sensored_low_speed_loop(void *arg);

/* current mode sensorless core controllers */
void do_current_mode_sensorless_high_speed_loop(void *arg);
void do_current_mode_sensorless_low_speed_loop(void *arg);
