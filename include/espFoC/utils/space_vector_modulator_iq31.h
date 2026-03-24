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
 * @file space_vector_modulator_iq31.h
 * @brief Space-vector PWM (SVPWM) in Q1.31: alpha/beta to three-phase duties [0,1].
 *        Mirrors the algorithm in space_vector_modulator.h (float).
 */

#pragma once

#include "espFoC/utils/esp_foc_iq31.h"

#ifdef __cplusplus
extern "C" {
#endif

static inline iq31_t esp_foc_iq31_max3(iq31_t a, iq31_t b, iq31_t c)
{
    return iq31_max(iq31_max(a, b), c);
}

static inline iq31_t esp_foc_iq31_min3(iq31_t a, iq31_t b, iq31_t c)
{
    return iq31_min(iq31_min(a, b), c);
}

/** Midpoint (a+b)/2 in Q1.31 without overflow for typical motor voltage range. */
static inline iq31_t esp_foc_iq31_avg2(iq31_t a, iq31_t b)
{
    return (iq31_t)(((int64_t)a + (int64_t)b) >> 1);
}

/**
 * SVPWM: inverse-Clarke to phase voltages, common-mode injection, normalize by inv_vbus.
 * @param v_alpha, v_beta  Phase voltages in alpha-beta (same units as float path, typically pu).
 * @param inv_vbus         Scale factor (same role as float inv_vbus in esp_foc_svm_set).
 * @param duty_a,b,c      Duties in [0, IQ31_ONE].
 */
static inline void esp_foc_svm_set_iq31(iq31_t v_alpha,
                                        iq31_t v_beta,
                                        iq31_t inv_vbus,
                                        iq31_t *duty_a,
                                        iq31_t *duty_b,
                                        iq31_t *duty_c)
{
    iq31_t zero = 0;
    iq31_t neg_half_alpha = iq31_sub(zero, iq31_mul(v_alpha, IQ31_HALF));
    iq31_t sqrt3_beta = iq31_mul(v_beta, IQ31_SQRT3_OVER_2);

    iq31_t v_a = v_alpha;
    iq31_t v_b = iq31_add(neg_half_alpha, sqrt3_beta);
    iq31_t v_c = iq31_sub(neg_half_alpha, sqrt3_beta);

    iq31_t v_max = esp_foc_iq31_max3(v_a, v_b, v_c);
    iq31_t v_min = esp_foc_iq31_min3(v_a, v_b, v_c);
    iq31_t v_cm = esp_foc_iq31_avg2(v_max, v_min);

    iq31_t da = iq31_add(IQ31_HALF, iq31_mul(iq31_sub(v_a, v_cm), inv_vbus));
    iq31_t db = iq31_add(IQ31_HALF, iq31_mul(iq31_sub(v_b, v_cm), inv_vbus));
    iq31_t dc = iq31_add(IQ31_HALF, iq31_mul(iq31_sub(v_c, v_cm), inv_vbus));

    *duty_a = iq31_clamp(da, zero, IQ31_ONE);
    *duty_b = iq31_clamp(db, zero, IQ31_ONE);
    *duty_c = iq31_clamp(dc, zero, IQ31_ONE);
}

#ifdef __cplusplus
}
#endif
