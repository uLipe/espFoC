/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Software dq circle clamp via CORDIC sqrt.
 */
#include "esp_foc_vlim_soft.h"

#include "espFoC/utils/esp_foc_trig.h"

void esp_foc_vlim_dq_soft(q16_t *vd, q16_t *vq, q16_t vmax)
{
    q16_t d = *vd;
    q16_t q = *vq;

    if (vmax <= 0) {
        *vd = 0;
        *vq = 0;
        return;
    }

    q16_t mag2 = q16_add(q16_mul(d, d), q16_mul(q, q));
    q16_t mag = esp_foc_sqrt(mag2);
    if (mag <= vmax) {
        return;
    }
    if (mag == 0) {
        *vd = 0;
        *vq = 0;
        return;
    }

    q16_t scale = q16_div(vmax, mag);
    *vd = q16_mul(d, scale);
    *vq = q16_mul(q, scale);
}
