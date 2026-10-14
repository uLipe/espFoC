/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include "espFoC/utils/esp_foc_angle.h"
#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Amplitude-invariant Clarke (Q16.16).
 * α = (2u − v − w)/3,  β = (v − w)/√3
 * Inverse: u=α, v=−α/2 + β√3/2, w=−α/2 − β√3/2
 *
 * 2-shunt callers reconstruct w = −u−v before the forward transform.
 */
ESP_FOC_INLINE void esp_foc_clarke(q16_t u, q16_t v, q16_t w, q16_t *alpha, q16_t *beta)
{
    const q16_t s = q16_sub(q16_sub(q16_add(u, u), v), w);
    /* s/3 truncated toward zero for any int32 s, without a 64-bit divide:
     * ceil(2^32/3) and >>32 floor, then +1 for negatives fixes the rounding. */
    *alpha = (q16_t)(((int64_t)s * 1431655766LL) >> 32) - (s >> 31);
    *beta = q16_mul(q16_sub(v, w), Q16_INV_SQRT3);
}

ESP_FOC_INLINE void esp_foc_inv_clarke(q16_t alpha, q16_t beta, q16_t *u, q16_t *v, q16_t *w)
{
    const q16_t half_a = q16_mul(alpha, Q16_HALF);
    const q16_t b_s3 = q16_mul(beta, Q16_SQRT3_2);
    *u = alpha;
    *v = q16_add(q16_neg(half_a), b_s3);
    *w = q16_sub(q16_neg(half_a), b_s3);
}

#ifdef __cplusplus
}
#endif
