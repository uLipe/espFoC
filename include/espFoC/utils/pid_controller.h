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

typedef struct {
    float kp;
    float ki;
    float kd;

    float integrator_limit;
    float accumulated_error;
    float previous_error;
    float max_output_value;
    float min_output_value;
    float dt;
    float inv_dt;

}esp_foc_pid_controller_t;

static inline void esp_foc_design_current_pi_discrete(float R, float L, float Fs, float fc, float *kp, float *ki)
{
    float Ts = 1.0f / Fs;
    float a = expf(-(R / L) * Ts);
    float b = (1.0f - a) / R;

    // Desired closed-loop pole from bandwidth
    float p = expf(-2.0f * (float)M_PI * fc * Ts);

    // Gain for C(z)=K(1-a z^-1)/(1-z^-1)
    float K = ((1.0f / p) - 1.0f) / b;

    // Convert to incremental PI form
    *kp = K * a;
    *ki = K * (1.0f - a) / Ts;
}

static inline void esp_foc_pid_reset(esp_foc_pid_controller_t *self)
{
    self->accumulated_error = 0.0f;
    self->previous_error = 0.0f;
}

static inline float  esp_foc_pid_update(esp_foc_pid_controller_t *self,
                                        float reference,
                                        float measure)
{

    float error = reference - measure;
    float error_diff = (error - self->previous_error) * self->inv_dt;
    float mv = (self->kp * error) +
        (self->accumulated_error) +
        (self->kd * error_diff);
    mv = esp_foc_clamp(mv, self->min_output_value, self->max_output_value) ;

    /* Avoid to integrate if the output was saturated but the error still drivers in the wrong direction */
    self->accumulated_error += (error * self->dt * self->ki);
    self->accumulated_error = esp_foc_clamp(self->accumulated_error, -self->integrator_limit, self->integrator_limit);
    self->previous_error = error;

    return mv;
}