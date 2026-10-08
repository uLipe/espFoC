/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Fixed-point trigonometry (Q16.16 radians).
 * Default: software CORDIC (32 iterations, quarter-circle + quadrant map).
 * Optional HW path via CONFIG_ESP_FOC_USE_HW_ACCEL_TRIGO (falls back to soft).
 */

/** sin and cos of @p angle in one call. Angle wrapped to (−π, +π]. */
void esp_foc_sincos(q16_t angle, q16_t *s_out, q16_t *c_out);

q16_t esp_foc_sin(q16_t angle);
q16_t esp_foc_cos(q16_t angle);

/** Two-argument arctangent; result in (−π, +π]. atan2(0,0) → 0. */
q16_t esp_foc_atan2(q16_t y, q16_t x);

/** Non-negative square root. Negative inputs return 0. */
q16_t esp_foc_sqrt(q16_t x);

#ifdef __cplusplus
}
#endif
