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
 * Min-max common-mode SVPWM (no sector switch).
 * αβ are per-unit of Vdc. Duties are unipolar [0, Q16_ONE].
 *
 * Inverse Clarke → v_cm = (max+min)/2 → duty = 1/2 + (v − v_cm).
 *
 * Optional HW via CONFIG_ESP_FOC_USE_HW_ACCEL_SVM (soft fallback).
 */
void esp_foc_svm(q16_t v_alpha, q16_t v_beta, q16_t *du, q16_t *dv, q16_t *dw);

#ifdef __cplusplus
}
#endif
