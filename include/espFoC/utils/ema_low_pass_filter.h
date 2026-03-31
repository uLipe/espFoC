/*
 * MIT License
 */
/**
 * @file ema_low_pass_filter.h
 * @brief First-order EMA low-pass in Q16.16.
 */
#pragma once

#include "espFoC/utils/esp_foc_q16.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    q16_t alpha;
    q16_t beta;
    q16_t y_n_prev;
} esp_foc_lp_filter_t;

static inline void esp_foc_low_pass_filter_init(esp_foc_lp_filter_t *filter,
                                                q16_t alpha)
{
    if (alpha > Q16_ONE) {
        alpha = Q16_ONE;
    } else if (alpha < 0) {
        alpha = 0;
    }
    filter->y_n_prev = 0;
    filter->alpha = alpha;
    filter->beta = q16_sub(Q16_ONE, alpha);
}

static inline void esp_foc_low_pass_filter_set_cutoff(esp_foc_lp_filter_t *filter,
                                                      float cutoff, float fs)
{
    float wc_norm = (2.0f * (float)M_PI * (cutoff / fs));
    float alpha_f = wc_norm / (1.0f + wc_norm);
    esp_foc_low_pass_filter_init(filter, q16_from_float(alpha_f));
}

static inline q16_t esp_foc_low_pass_filter_update(esp_foc_lp_filter_t *filter,
                                                    q16_t x_n)
{
    q16_t y_n = q16_add(q16_mul(filter->alpha, x_n), q16_mul(filter->beta, filter->y_n_prev));
    filter->y_n_prev = y_n;
    return y_n;
}

#ifdef __cplusplus
}
#endif
