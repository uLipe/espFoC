/*
 * MIT License
 */
/**
 * @file foc_math_q16.h
 * @brief FOC math in Q16.16: Clarke/Park, normalize angle (radians), limit voltage, bias.
 */
#pragma once

#include "espFoC/utils/esp_foc_q16.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Normalize electrical angle in radians to [0, Q16_TWO_PI). */
static inline q16_t q16_normalize_angle_rad(q16_t angle_rad)
{
    int64_t p = (int64_t)Q16_TWO_PI;
    if (p <= 0) {
        return 0;
    }
    int64_t a = (int64_t)angle_rad % p;
    if (a < 0) {
        a += p;
    }
    return (q16_t)a;
}

static inline void q16_clarke(q16_t u, q16_t v, q16_t w, q16_t *alpha, q16_t *beta)
{
    (void)w;
    q16_t sum_u2v = q16_add(q16_add(u, v), v);
    *alpha = u;
    *beta = q16_mul(sum_u2v, Q16_K3_CLARKE);
}

static inline void q16_park(q16_t sin_t, q16_t cos_t, q16_t alpha, q16_t beta,
                            q16_t *d, q16_t *q)
{
    *d = q16_add(q16_mul(alpha, cos_t), q16_mul(beta, sin_t));
    *q = q16_sub(q16_mul(beta, cos_t), q16_mul(alpha, sin_t));
}

static inline void q16_inverse_park(q16_t sin_t, q16_t cos_t, q16_t d, q16_t q,
                                    q16_t *alpha, q16_t *beta)
{
    *alpha = q16_sub(q16_mul(d, cos_t), q16_mul(q, sin_t));
    *beta = q16_add(q16_mul(d, sin_t), q16_mul(q, cos_t));
}

static inline void q16_inverse_clarke(q16_t alpha, q16_t beta,
                                    q16_t *u, q16_t *v, q16_t *w)
{
    *u = alpha;
    *v = q16_sub(q16_mul(beta, Q16_SQRT3_OVER_2), q16_mul(alpha, Q16_HALF));
    *w = q16_sub(q16_sub(0, alpha), *v);
}

/** Integer sqrt for uint64 (Newton). No float. */
static inline uint64_t esp_foc_u64_isqrt(uint64_t x)
{
    if (x == 0u) {
        return 0u;
    }
    uint64_t y = x;
    uint64_t z = (x + 1u) >> 1;
    while (z < y) {
        y = z;
        z = (x / z + z) >> 1;
    }
    return y;
}

/**
 * Clamp (vd,vq) to magnitude <= v_dc in Q16 space (same geometry as float hypot).
 * Integer-only — intended for PWM-period hot path.
 */
static inline void esp_foc_limit_voltage_q16(q16_t *v_d, q16_t *v_q, q16_t v_dc)
{
    int64_t vd64 = (int64_t)*v_d;
    int64_t vq64 = (int64_t)*v_q;
    int64_t dc = (int64_t)v_dc;
    if (dc <= 0) {
        return;
    }
    uint64_t av = (uint64_t)(vd64 >= 0 ? vd64 : -vd64);
    uint64_t aq = (uint64_t)(vq64 >= 0 ? vq64 : -vq64);
    uint64_t m2 = av * av + aq * aq;
    uint64_t dc_u = (uint64_t)dc;
    uint64_t dc2 = dc_u * dc_u;
    if (m2 <= dc2) {
        return;
    }
    uint64_t mag = esp_foc_u64_isqrt(m2);
    if (mag == 0u) {
        return;
    }
    int64_t s = ((int64_t)dc_u << 16) / (int64_t)mag;
    if (s > (int64_t)INT32_MAX) {
        s = INT32_MAX;
    }
    if (s < (int64_t)INT32_MIN) {
        s = INT32_MIN;
    }
    *v_d = q16_mul(*v_d, (q16_t)s);
    *v_q = q16_mul(*v_q, (q16_t)s);
}

static inline void esp_foc_apply_bias_q16(q16_t *v_alpha, q16_t *v_beta)
{
    *v_alpha = q16_add(q16_mul(*v_alpha, Q16_HALF), Q16_HALF);
    *v_beta = q16_add(q16_mul(*v_beta, Q16_HALF), Q16_HALF);
}

#ifdef CONFIG_ESP_FOC_COMP_THI
static inline void esp_foc_third_harmonic_injection_q16(q16_t *v_alpha, q16_t *v_beta)
{
    q16_t zero = 0;
    q16_t v_max = q16_max(q16_max(*v_alpha, *v_beta), zero);
    q16_t v_min = q16_min(q16_min(*v_alpha, *v_beta), zero);
    q16_t v_offset = (q16_t)(((int64_t)v_max + (int64_t)v_min) >> 1);
    *v_alpha = q16_sub(*v_alpha, v_offset);
    *v_beta = q16_sub(*v_beta, v_offset);
    /* 0.95 * Q16_ONE ≈ 62259 */
    *v_alpha = q16_mul(*v_alpha, (q16_t)62259);
    *v_beta = q16_mul(*v_beta, (q16_t)62259);
}
#endif

#ifdef __cplusplus
}
#endif
