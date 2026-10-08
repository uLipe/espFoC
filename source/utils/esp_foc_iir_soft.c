/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Software TDF-II biquad + Butterworth LPF design (float at design only).
 */
#include "esp_foc_iir_soft.h"

#include <math.h>
#include <limits.h>

static q16_t sat_shift16(int64_t acc)
{
    int64_t v = acc >> 16;
    if (v > INT32_MAX) {
        return (q16_t)INT32_MAX;
    }
    if (v < INT32_MIN) {
        return (q16_t)INT32_MIN;
    }
    return (q16_t)v;
}

q16_t esp_foc_iir_soft_update(esp_foc_iir_t *f, q16_t x)
{
    if (f == NULL) {
        return 0;
    }

    q16_t b0 = f->b0;
    q16_t b1 = f->b1;
    q16_t b2 = f->b2;
    q16_t a1 = f->a1;
    q16_t a2 = f->a2;
    q16_t w1 = f->w1;
    q16_t w2 = f->w2;

    q16_t y = sat_shift16(((int64_t)b0 * (int64_t)x) + ((int64_t)w1 << 16));
    q16_t w1n = sat_shift16(((int64_t)b1 * (int64_t)x) -
                            ((int64_t)a1 * (int64_t)y) +
                            ((int64_t)w2 << 16));
    q16_t w2n = sat_shift16(((int64_t)b2 * (int64_t)x) -
                            ((int64_t)a2 * (int64_t)y));

    f->w1 = w1n;
    f->w2 = w2n;
    return y;
}

void esp_foc_iir_soft_reset(esp_foc_iir_t *f)
{
    if (f == NULL) {
        return;
    }
    f->w1 = 0;
    f->w2 = 0;
}

void esp_foc_iir_soft_set_coeffs(esp_foc_iir_t *f, q16_t b0, q16_t b1, q16_t b2, q16_t a1, q16_t a2)
{
    if (f == NULL) {
        return;
    }
    f->b0 = b0;
    f->b1 = b1;
    f->b2 = b2;
    f->a1 = a1;
    f->a2 = a2;
}

esp_err_t esp_foc_iir_soft_design_lpf(esp_foc_iir_t *f, float fs_hz, float fc_hz)
{
    if (f == NULL || !(fs_hz > 0.0f) || !(fc_hz > 0.0f) || !(fc_hz < (fs_hz * 0.5f))) {
        return ESP_ERR_INVALID_ARG;
    }

    float k = tanf((float)M_PI * fc_hz / fs_hz);
    float k2 = k * k;
    float sqrt2 = 1.41421356237f;
    float a0 = 1.0f + sqrt2 * k + k2;
    if (!(a0 > 1e-12f)) {
        return ESP_ERR_INVALID_ARG;
    }

    f->b0 = q16_from_float(k2 / a0);
    f->b1 = q16_from_float((2.0f * k2) / a0);
    f->b2 = q16_from_float(k2 / a0);
    f->a1 = q16_from_float((2.0f * k2 - 2.0f) / a0);
    f->a2 = q16_from_float((1.0f - sqrt2 * k + k2) / a0);
    f->w1 = 0;
    f->w2 = 0;
    return ESP_OK;
}
