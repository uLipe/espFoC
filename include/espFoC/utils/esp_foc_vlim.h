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
 * Circle clamp |Vdq| ≤ vmax (pu of Vdc). Uses CORDIC sqrt.
 * Under-limit: unchanged. vmax ≤ 0: both components zeroed.
 * Linear SVPWM: vmax = Q16_INV_SQRT3.
 *
 * Optional HW via CONFIG_ESP_FOC_USE_HW_ACCEL_VLIM (soft fallback).
 */
void esp_foc_vlim_dq(q16_t *vd, q16_t *vq, q16_t vmax);

#ifdef __cplusplus
}
#endif
