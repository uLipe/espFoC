/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/esp_foc_trig.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Same clamp as esp_foc_vlim_dq() with |Vdq| supplied by the caller (e.g.
 * the trigo_hw driver's magnitude in the TEZ ISR), so no square root is
 * taken here. mag must be |(*vd, *vq)|; mag ≤ 0 with vmax > 0 leaves the
 * vector unchanged.
 */
ESP_FOC_INLINE void esp_foc_vlim_dq_mag(q16_t *vd, q16_t *vq, q16_t mag, q16_t vmax)
{
    const q16_t d = *vd;
    const q16_t q = *vq;

    if (vmax <= 0) {
        *vd = 0;
        *vq = 0;
        return;
    }
    if (mag <= vmax) {
        return;
    }

    /* 0 < vmax < mag: with vmax < 1.0 the Q16 numerator fits 32 bits and a
     * single divu replaces the 64-bit divide; same truncated quotient. */
    const q16_t scale = (vmax < Q16_ONE)
                            ? (q16_t)(((uint32_t)vmax << 16) / (uint32_t)mag)
                            : q16_div(vmax, mag);
    *vd = q16_mul(d, scale);
    *vq = q16_mul(q, scale);
}

/**
 * Circle clamp |Vdq| ≤ vmax (pu of Vdc). Uses the software CORDIC sqrt.
 * Under-limit: unchanged. vmax ≤ 0: both components zeroed.
 * Linear SVPWM: vmax = Q16_INV_SQRT3.
 */
ESP_FOC_INLINE void esp_foc_vlim_dq(q16_t *vd, q16_t *vq, q16_t vmax)
{
    const q16_t d = *vd;
    const q16_t q = *vq;
    esp_foc_vlim_dq_mag(vd, vq, esp_foc_sqrt(q16_add(q16_mul(d, d), q16_mul(q, q))), vmax);
}

#ifdef __cplusplus
}
#endif
