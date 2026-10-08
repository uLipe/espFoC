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
 * Amplitude-invariant Clarke (Q16.16).
 * α = (2u − v − w)/3,  β = (v − w)/√3
 * Inverse: u=α, v=−α/2 + β√3/2, w=−α/2 − β√3/2
 *
 * Optional HW via CONFIG_ESP_FOC_USE_HW_ACCEL_CLARKE (soft fallback).
 * 2-shunt callers reconstruct w = −u−v before the forward transform.
 */
void esp_foc_clarke(q16_t u, q16_t v, q16_t w, q16_t *alpha, q16_t *beta);
void esp_foc_inv_clarke(q16_t alpha, q16_t beta, q16_t *u, q16_t *v, q16_t *w);

#ifdef __cplusplus
}
#endif
