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
 * Park / inverse Park (Q16.16). Caller supplies sin/cos of θ_e
 * (one sincos per ISR for both current Park and voltage inverse Park).
 *
 * d = α cos + β sin
 * q = β cos − α sin
 * α = d cos − q sin
 * β = d sin + q cos
 *
 * Optional HW via CONFIG_ESP_FOC_USE_HW_ACCEL_PARK (soft fallback).
 */
void esp_foc_park(q16_t s, q16_t c, q16_t alpha, q16_t beta, q16_t *d, q16_t *q);
void esp_foc_inv_park(q16_t s, q16_t c, q16_t d, q16_t q, q16_t *alpha, q16_t *beta);

#ifdef __cplusplus
}
#endif
