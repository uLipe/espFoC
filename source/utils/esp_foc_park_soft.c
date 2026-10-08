/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Software Park / inverse Park.
 */
#include "esp_foc_park_soft.h"

void esp_foc_park_soft(q16_t s, q16_t c, q16_t alpha, q16_t beta, q16_t *d, q16_t *q)
{
    *d = q16_add(q16_mul(alpha, c), q16_mul(beta, s));
    *q = q16_sub(q16_mul(beta, c), q16_mul(alpha, s));
}

void esp_foc_inv_park_soft(q16_t s, q16_t c, q16_t d, q16_t q, q16_t *alpha, q16_t *beta)
{
    *alpha = q16_sub(q16_mul(d, c), q16_mul(q, s));
    *beta = q16_add(q16_mul(d, s), q16_mul(q, c));
}
