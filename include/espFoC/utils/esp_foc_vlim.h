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

/**
 * Same clamp with |Vdq| supplied by the caller (e.g. the trigo_hw driver's
 * magnitude in the TEZ ISR), so no square root is taken here.
 * mag must be |(*vd, *vq)|; mag ≤ 0 with vmax > 0 leaves the vector unchanged.
 */
void esp_foc_vlim_dq_mag(q16_t *vd, q16_t *vq, q16_t mag, q16_t vmax);

#ifdef __cplusplus
}
#endif
