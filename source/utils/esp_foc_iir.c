/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Biquad coefficient design and setters; update() is inline in the header.
 */
#include "espFoC/utils/esp_foc_iir.h"

#include <math.h>

void esp_foc_iir_reset(esp_foc_iir_t *f)
{
    if (f == NULL) {
        return;
    }
    f->w1 = 0;
    f->w2 = 0;
}

void esp_foc_iir_set_coeffs(esp_foc_iir_t *f, q16_t b0, q16_t b1, q16_t b2, q16_t a1, q16_t a2)
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

esp_err_t esp_foc_iir_design_lpf(esp_foc_iir_t *f, float fs_hz, float fc_hz)
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
