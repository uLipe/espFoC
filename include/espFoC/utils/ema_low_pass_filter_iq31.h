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
 * @file ema_low_pass_filter_iq31.h
 * @brief First-order EMA low-pass filter in Q1.31 (same structure as ema_low_pass_filter.h).
 *        y[n] = alpha*x[n] + beta*y[n-1], beta = 1 - alpha (approximated in fixed point).
 */

#pragma once

#include "espFoC/utils/esp_foc_iq31.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    iq31_t alpha;
    iq31_t beta;
    iq31_t y_n_prev;
} esp_foc_lp_filter_iq31_t;

/** alpha in [0, IQ31_ONE]; y_n_prev cleared. */
static inline void esp_foc_low_pass_filter_init_iq31(esp_foc_lp_filter_iq31_t *filter,
                                                     iq31_t alpha)
{
    if (alpha > IQ31_ONE) {
        alpha = IQ31_ONE;
    } else if (alpha < 0) {
        alpha = 0;
    }
    filter->y_n_prev = 0;
    filter->alpha = alpha;
    filter->beta = iq31_sub(IQ31_ONE, alpha);
}

/** Same formula as float path: wc_norm = 2*pi*cutoff/fs; alpha = wc_norm/(1+wc_norm). */
static inline void esp_foc_low_pass_filter_set_cutoff_iq31(esp_foc_lp_filter_iq31_t *filter,
                                                           float cutoff, float fs)
{
    float wc_norm = (2.0f * (float)M_PI * (cutoff / fs));
    float alpha_f = wc_norm / (1.0f + wc_norm);
    esp_foc_low_pass_filter_init_iq31(filter, iq31_from_float(alpha_f));
}

/** y[n] = alpha*x[n] + beta*y[n-1] */
static inline iq31_t esp_foc_low_pass_filter_update_iq31(esp_foc_lp_filter_iq31_t *filter,
                                                           iq31_t x_n)
{
    iq31_t y_n = iq31_add(iq31_mul(filter->alpha, x_n), iq31_mul(filter->beta, filter->y_n_prev));
    filter->y_n_prev = y_n;
    return y_n;
}

#ifdef __cplusplus
}
#endif
