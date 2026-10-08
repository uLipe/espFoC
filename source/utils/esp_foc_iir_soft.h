/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Internal software IIR entry points.
 */
#pragma once

#include "esp_err.h"
#include "espFoC/utils/esp_foc_iir.h"

q16_t esp_foc_iir_soft_update(esp_foc_iir_t *f, q16_t x);
void esp_foc_iir_soft_reset(esp_foc_iir_t *f);
void esp_foc_iir_soft_set_coeffs(esp_foc_iir_t *f, q16_t b0, q16_t b1, q16_t b2, q16_t a1, q16_t a2);
esp_err_t esp_foc_iir_soft_design_lpf(esp_foc_iir_t *f, float fs_hz, float fc_hz);
