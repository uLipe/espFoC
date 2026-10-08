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

/* π and 2π in Q16.16 (π ≈ 3.14159265). */
#define Q16_PI      ((q16_t)205887)
#define Q16_PI_2    ((q16_t)102944)
#define Q16_TWO_PI  ((q16_t)411775)
#define Q16_MINUS_PI ((q16_t)(-205887))
#define Q16_MINUS_PI_2 ((q16_t)(-102944))
/* √3/2 ≈ 0.8660254 — 120° phase shift helpers */
#define Q16_SQRT3_2 ((q16_t)56756)
/* 1/√3 ≈ 0.5773503 — linear SVPWM |Vdq| max in pu of Vdc */
#define Q16_INV_SQRT3 ((q16_t)37837)

/** Wrap angle into (−π, +π]. */
static inline q16_t q16_wrap_pi(q16_t x)
{
    while (x > Q16_PI) {
        x = q16_sub(x, Q16_TWO_PI);
    }
    while (x <= Q16_MINUS_PI) {
        x = q16_add(x, Q16_TWO_PI);
    }
    return x;
}

/** Shortest signed delta from prev → now, result in (−π, +π]. */
static inline q16_t q16_angle_delta(q16_t prev, q16_t now)
{
    return q16_wrap_pi(q16_sub(now, prev));
}

#ifdef __cplusplus
}
#endif
