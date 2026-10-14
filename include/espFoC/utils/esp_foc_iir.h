/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stddef.h>

#include "esp_err.h"
#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Second-order IIR (biquad) in Q16.16.
 * Hot path: transposed DF-II.
 *
 * H(z) = (b0 + b1 z^-1 + b2 z^-2) / (1 + a1 z^-1 + a2 z^-2)
 */
typedef struct {
    q16_t b0;
    q16_t b1;
    q16_t b2;
    q16_t a1;
    q16_t a2;
    q16_t w1;
    q16_t w2;
} esp_foc_iir_t;

ESP_FOC_INLINE q16_t esp_foc_iir_update(esp_foc_iir_t *f, q16_t x)
{
    if (f == NULL) {
        return 0;
    }

    const q16_t b0 = f->b0;
    const q16_t b1 = f->b1;
    const q16_t b2 = f->b2;
    const q16_t a1 = f->a1;
    const q16_t a2 = f->a2;
    const q16_t w1 = f->w1;
    const q16_t w2 = f->w2;

    const q16_t y = q16_sat_shr16(((int64_t)b0 * (int64_t)x) + ((int64_t)w1 << 16));
    const q16_t w1n = q16_sat_shr16(((int64_t)b1 * (int64_t)x) -
                                    ((int64_t)a1 * (int64_t)y) +
                                    ((int64_t)w2 << 16));
    const q16_t w2n = q16_sat_shr16(((int64_t)b2 * (int64_t)x) -
                                    ((int64_t)a2 * (int64_t)y));

    f->w1 = w1n;
    f->w2 = w2n;
    return y;
}

void esp_foc_iir_reset(esp_foc_iir_t *f);
void esp_foc_iir_set_coeffs(esp_foc_iir_t *f, q16_t b0, q16_t b1, q16_t b2, q16_t a1, q16_t a2);

/** Butterworth LPF, bilinear + prewarp. Resets delays. fc must be in (0, fs/2). */
esp_err_t esp_foc_iir_design_lpf(esp_foc_iir_t *f, float fs_hz, float fc_hz);

#ifdef __cplusplus
}
#endif
