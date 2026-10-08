/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include "esp_err.h"
#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Second-order IIR (biquad) in Q16.16.
 * Hot path: transposed DF-II. Optional HW via CONFIG_ESP_FOC_USE_HW_ACCEL_IIR
 * (falls back to software; no IIR accelerator on current targets).
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

q16_t esp_foc_iir_update(esp_foc_iir_t *f, q16_t x);
void esp_foc_iir_reset(esp_foc_iir_t *f);
void esp_foc_iir_set_coeffs(esp_foc_iir_t *f, q16_t b0, q16_t b1, q16_t b2, q16_t a1, q16_t a2);

/** Butterworth LPF, bilinear + prewarp. Resets delays. fc must be in (0, fs/2). */
esp_err_t esp_foc_iir_design_lpf(esp_foc_iir_t *f, float fs_hz, float fc_hz);

#ifdef __cplusplus
}
#endif
