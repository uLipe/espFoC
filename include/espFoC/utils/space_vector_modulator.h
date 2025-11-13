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

#include <math.h>

static inline void esp_foc_svm_set(float v_alpha,
                                    float v_beta,
                                    float inv_vbus,
                                    float *duty_a,
                                    float *duty_b,
                                    float *duty_c)
{
    // Inverse Clarke: αβ → phase voltages (line-to-neutral)
    // α axis aligned with phase A
    const float SQRT3_OVER_2 = 0.8660254037844386f;

    float v_a = v_alpha;
    float v_b = -0.5f * v_alpha + SQRT3_OVER_2 * v_beta;
    float v_c = -0.5f * v_alpha - SQRT3_OVER_2 * v_beta;

    // Common-mode voltage for SVPWM (space vector zero-sequence)
    float v_max = v_a;
    if (v_b > v_max) v_max = v_b;
    if (v_c > v_max) v_max = v_c;

    float v_min = v_a;
    if (v_b < v_min) v_min = v_b;
    if (v_c < v_min) v_min = v_c;

    float v_cm = 0.5f * (v_max + v_min);

    // Subtract common-mode and normalize with precomputed inv_vbus
    float d_a = 0.5f + (v_a - v_cm) * inv_vbus;
    float d_b = 0.5f + (v_b - v_cm) * inv_vbus;
    float d_c = 0.5f + (v_c - v_cm) * inv_vbus;

    // Clamp to [0,1] for safety
    if (d_a < 0.0f) d_a = 0.0f; else if (d_a > 1.0f) d_a = 1.0f;
    if (d_b < 0.0f) d_b = 0.0f; else if (d_b > 1.0f) d_b = 1.0f;
    if (d_c < 0.0f) d_c = 0.0f; else if (d_c > 1.0f) d_c = 1.0f;

    *duty_a = d_a;
    *duty_b = d_b;
    *duty_c = d_c;
}