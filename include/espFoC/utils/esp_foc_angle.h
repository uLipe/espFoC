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

/* Design-time (float / double) counterparts. */
#define ESP_FOC_TWO_PI_F     6.28318530718f
#define ESP_FOC_TWO_PI       6.283185307179586
#define ESP_FOC_INV_SQRT3_F  0.57735026919f

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

/** Angle covered at omega [rad/s, Q16] over t seconds given in Q0.32. */
static inline q16_t q16_angle_travel(q16_t omega, uint32_t t_q32)
{
    return (q16_t)(((int64_t)omega * (int64_t)t_q32) >> 32);
}

/** Electrical angle step per PWM period for fe_hz (Q16) at pwm_hz. */
static inline q16_t q16_angle_step_hz(q16_t fe_hz, uint32_t pwm_hz)
{
    return (q16_t)(((int64_t)fe_hz * (int64_t)Q16_TWO_PI) / ((int64_t)pwm_hz << 16));
}

#ifdef __cplusplus
}
#endif
