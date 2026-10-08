/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Software amplitude-invariant Clarke / inverse Clarke.
 */
#include "esp_foc_clarke_soft.h"

#include "espFoC/utils/esp_foc_angle.h"

#define Q16_THREE ((q16_t)196608)

void esp_foc_clarke_soft(q16_t u, q16_t v, q16_t w, q16_t *alpha, q16_t *beta)
{
    q16_t two_u = q16_add(u, u);
    *alpha = q16_div(q16_sub(q16_sub(two_u, v), w), Q16_THREE);
    *beta = q16_mul(q16_sub(v, w), Q16_INV_SQRT3);
}

void esp_foc_inv_clarke_soft(q16_t alpha, q16_t beta, q16_t *u, q16_t *v, q16_t *w)
{
    q16_t half_a = q16_mul(alpha, Q16_HALF);
    q16_t b_s3 = q16_mul(beta, Q16_SQRT3_2);
    *u = alpha;
    *v = q16_add(q16_neg(half_a), b_s3);
    *w = q16_sub(q16_neg(half_a), b_s3);
}
