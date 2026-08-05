/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

/**
 * @file biquad_q16.h
 * @brief Direct-Form-II Transposed biquad in Q16.16 plus a 2nd-order
 *        Butterworth low-pass designer.
 *
 * Replaces the EMA (`esp_foc_lp_filter_t`) historically used across the
 * library. DF-II Transposed is the most numerically stable structure
 * for fixed-point: state holds bounded values regardless of the input
 * magnitude, no internal accumulator drifts.
 *
 * Difference equation:
 *
 *     y[n]  = b0 * x[n] + s1[n - 1]
 *     s1[n] = b1 * x[n] + s2[n - 1] - a1 * y[n]
 *     s2[n] = b2 * x[n]             - a2 * y[n]
 *
 * The designer runs in float at init time; only the resulting
 * coefficients live in Q16. Coefficient ranges for Butterworth low-pass
 * stay within ~+-2 for any sane fc/fs ratio so Q16.16 fits comfortably.
 */

#pragma once

#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    q16_t b0;
    q16_t b1;
    q16_t b2;
    q16_t a1;
    q16_t a2;
    q16_t s1;
    q16_t s2;
} esp_foc_biquad_q16_t;

/**
 * @brief Clear filter state (s1, s2). Coefficients are preserved.
 */
static inline void esp_foc_biquad_q16_reset(esp_foc_biquad_q16_t *f)
{
    if (f == NULL) {
        return;
    }
    f->s1 = 0;
    f->s2 = 0;
}

/**
 * @brief Push one sample through the biquad. Hot path; fully inlined,
 *        no float, no division. Five Q16 multiplies per call.
 */
static inline q16_t esp_foc_biquad_q16_update(esp_foc_biquad_q16_t *f,
                                              q16_t x)
{
    q16_t y = q16_add(q16_mul(f->b0, x), f->s1);
    f->s1 = q16_sub(q16_add(q16_mul(f->b1, x), f->s2), q16_mul(f->a1, y));
    f->s2 = q16_sub(q16_mul(f->b2, x), q16_mul(f->a2, y));
    return y;
}

/**
 * @brief Bypass design: y[n] = x[n]. Used by drivers that have not
 *        been configured yet so the filter never silently zeroes the
 *        signal before init completes.
 */
void esp_foc_biquad_q16_set_bypass(esp_foc_biquad_q16_t *f);

/**
 * @brief Design a 2nd-order Butterworth low-pass via bilinear
 *        transform with prewarping at fc.
 *
 * @param f       biquad to fill; state is cleared as a side-effect.
 * @param fc_hz   cutoff frequency. Clamped to (0, fs/2).
 * @param fs_hz   sample rate at which the filter will be updated.
 *
 * Falls back to a bypass design when fc_hz / fs_hz are not usable
 * (zero, negative, fc beyond Nyquist), so callers can never end up
 * with garbage coefficients silently stalling the loop.
 */
void esp_foc_biquad_butterworth_lpf_design_q16(esp_foc_biquad_q16_t *f,
                                               float fc_hz,
                                               float fs_hz);

#ifdef __cplusplus
}
#endif
