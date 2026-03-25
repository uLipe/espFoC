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

#include "espFoC/esp_foc.h"
#include "espFoC/esp_foc_axis_iq31.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_foc_err_t esp_foc_initialize_axis_iq31(esp_foc_axis_iq31_t *axis,
                                           esp_foc_inverter_t *inverter,
                                           esp_foc_rotor_sensor_t *rotor,
                                           esp_foc_isensor_t *isensor,
                                           esp_foc_motor_control_settings_iq31_t settings);
esp_foc_err_t esp_foc_align_axis_iq31(esp_foc_axis_iq31_t *axis);
esp_foc_err_t esp_foc_run_iq31(esp_foc_axis_iq31_t *axis);
esp_foc_err_t esp_foc_set_regulation_callback_iq31(esp_foc_axis_iq31_t *axis,
                                                   esp_foc_motor_regulation_callback_iq31_t callback);

/* IQ31 sensored strategy callbacks */
void do_current_mode_sensored_high_speed_loop_iq31(void *arg);
void do_current_mode_sensored_low_speed_loop_iq31(void *arg);

#ifdef __cplusplus
}
#endif

