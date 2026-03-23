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

/**
 * @file pid_controller_iq31.h
 * @brief PID controller with IQ31 reference/measure/output (per-unit [-1, 1]).
 *        Internal state matches esp_foc_pid_controller_t (float) for identical dynamics;
 *        use esp_foc_pid_iq31_t as alias. Call esp_foc_pid_iq31_init_from_float() to fill gains.
 */

#pragma once

#include "espFoC/utils/esp_foc_iq31.h"
#include "espFoC/utils/foc_math.h"
#include "espFoC/utils/pid_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Same memory layout as esp_foc_pid_controller_t — can cast if needed. */
typedef esp_foc_pid_controller_t esp_foc_pid_iq31_t;

static inline void esp_foc_pid_iq31_init_from_float(esp_foc_pid_iq31_t *self,
                                                    float kp, float ki, float kd,
                                                    float dt, float out_min, float out_max,
                                                    float int_lim)
{
    self->kp = kp;
    self->ki = ki;
    self->kd = kd;
    self->dt = dt;
    self->inv_dt = (dt > 0.0f) ? (1.0f / dt) : 0.0f;
    self->min_output_value = out_min;
    self->max_output_value = out_max;
    self->integrator_limit = int_lim;
    esp_foc_pid_reset(self);
}

/** Same as esp_foc_pid_reset. */
static inline void esp_foc_pid_iq31_reset(esp_foc_pid_iq31_t *self)
{
    esp_foc_pid_reset(self);
}

/**
 * PID update: reference and measure are Q1.31 in [-1, 1] (per-unit).
 * Converts to float, runs same law as esp_foc_pid_update, returns output as Q1.31.
 */
static inline iq31_t esp_foc_pid_update_iq31(esp_foc_pid_iq31_t *self,
                                             iq31_t reference,
                                             iq31_t measure)
{
    float out = esp_foc_pid_update(self,
                                   iq31_to_float(reference),
                                   iq31_to_float(measure));
    return iq31_from_float(out);
}

#ifdef __cplusplus
}
#endif
