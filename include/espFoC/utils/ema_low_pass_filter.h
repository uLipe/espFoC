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

typedef struct {
    float alpha;
    float beta;
    float y_n_prev;
} esp_foc_lp_filter_t;

static inline void esp_foc_low_pass_filter_init(esp_foc_lp_filter_t *filter,
                                                float alpha)
{
    if(alpha > 1.0f) {
        alpha = 1.0f;
    } else if (alpha < 0.0f) {
        alpha = 0.0f;
    }

    filter->y_n_prev = 0.0f;
    filter->alpha = alpha;
    filter->beta = 1.0f - alpha;

}

static inline void esp_foc_low_pass_filter_set_cutoff(esp_foc_lp_filter_t *filter,
                                                    float cutoff, float fs)
{
    float wc_norm = (2.0f * M_PI * (cutoff / fs));
    float alpha =  wc_norm / (1 + wc_norm);
    esp_foc_low_pass_filter_init(filter, alpha);
}

static inline float esp_foc_low_pass_filter_update(esp_foc_lp_filter_t *filter,
                                                  float x_n)
{
    float y_n = filter->alpha * x_n + filter->beta * filter->y_n_prev;
    filter->y_n_prev = y_n;
    return y_n;
}