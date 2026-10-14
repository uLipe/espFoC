/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include "espFoC/utils/esp_foc_clarke.h"
#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Min-max common-mode SVPWM (no sector switch).
 * αβ are per-unit of Vdc. Duties are unipolar [0, Q16_ONE].
 *
 * Inverse Clarke → v_cm = (max+min)/2 → duty = 1/2 + (v − v_cm).
 */
ESP_FOC_INLINE void esp_foc_svm(q16_t v_alpha, q16_t v_beta, q16_t *du, q16_t *dv, q16_t *dw)
{
    q16_t vu;
    q16_t vv;
    q16_t vw;
    esp_foc_inv_clarke(v_alpha, v_beta, &vu, &vv, &vw);

    const q16_t vmax = q16_max(q16_max(vu, vv), vw);
    const q16_t vmin = q16_min(q16_min(vu, vv), vw);
    const q16_t v_cm = (q16_t)(((int64_t)vmax + (int64_t)vmin) >> 1);

    *du = q16_clamp(q16_add(Q16_HALF, q16_sub(vu, v_cm)), 0, Q16_ONE);
    *dv = q16_clamp(q16_add(Q16_HALF, q16_sub(vv, v_cm)), 0, Q16_ONE);
    *dw = q16_clamp(q16_add(Q16_HALF, q16_sub(vw, v_cm)), 0, Q16_ONE);
}

#ifdef __cplusplus
}
#endif
