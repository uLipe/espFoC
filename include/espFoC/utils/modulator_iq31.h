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
 * @file modulator_iq31.h
 * @brief DQ voltage modulation and ABC->DQ current conversion in Q1.31.
 *        Mirrors modulator.h (float). All arguments are iq31_t unless noted.
 */

#pragma once

#include "espFoC/utils/foc_math_iq31.h"
#include "espFoC/utils/space_vector_modulator_iq31.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Limit (v_d,v_q) to magnitude vmax, inverse Park, then SVPWM to duties.
 * The @p bias parameter is accepted for API symmetry with the float modulator but is unused.
 */
static inline void esp_foc_modulate_dq_voltage_iq31(iq31_t sin,
                                                    iq31_t cos,
                                                    iq31_t v_d,
                                                    iq31_t v_q,
                                                    iq31_t *v_alpha,
                                                    iq31_t *v_beta,
                                                    iq31_t *v_u,
                                                    iq31_t *v_v,
                                                    iq31_t *v_w,
                                                    iq31_t vmax,
                                                    iq31_t bias,
                                                    iq31_t normalization_scale)
{
    (void)bias;
    iq31_t vd = v_d;
    iq31_t vq = v_q;
    esp_foc_limit_voltage_iq31(&vd, &vq, vmax);
    iq31_inverse_park(sin, cos, vd, vq, v_alpha, v_beta);
    esp_foc_svm_set_iq31(*v_alpha, *v_beta, normalization_scale, v_u, v_v, v_w);
}

/**
 * Clarke then Park: phase currents (u,v,w) -> alpha/beta and d/q.
 */
static inline void esp_foc_get_dq_currents_iq31(iq31_t sin,
                                                iq31_t cos,
                                                iq31_t i_u,
                                                iq31_t i_v,
                                                iq31_t i_w,
                                                iq31_t *i_alpha,
                                                iq31_t *i_beta,
                                                iq31_t *i_q,
                                                iq31_t *i_d)
{
    iq31_t d;
    iq31_t q;
    iq31_clarke(i_u, i_v, i_w, i_alpha, i_beta);
    iq31_park(sin, cos, *i_alpha, *i_beta, &d, &q);
    *i_d = d;
    *i_q = q;
}

#ifdef __cplusplus
}
#endif
