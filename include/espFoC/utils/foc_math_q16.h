/*
 * MIT License
 */
/**
 * @file foc_math_q16.h
 * @brief FOC math in Q16.16: Clarke/Park, normalize angle (radians), limit voltage, bias.
 */
#pragma once

#include <sdkconfig.h>
#include "espFoC/utils/esp_foc_int_sqrt.h"
#include "espFoC/utils/esp_foc_q16.h"
#include <stdint.h>
#include <limits.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Normalize electrical angle: θ_norm ∈ [0, Q16_ONE) ↔ θ_rad ∈ [0, 2π). */
static inline q16_t q16_normalize_angle(q16_t angle_rad)
{
    while (angle_rad < 0)
         angle_rad = q16_add(angle_rad, Q16_ONE);
    while (angle_rad >= Q16_ONE)
        angle_rad = q16_sub(angle_rad, Q16_ONE);

    return (q16_t)angle_rad;
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

/**
 * Clamp (vd,vq) to magnitude <= |v_dc| in per-unit Q16 (same geometry as float hypot).
 *
 * Canonical form: scale = v_dc / |V| with |V| from
 * hypot; then v_d *= scale, v_q *= scale — no component-wise divide by |V|.
 *
 * Integer path: inv_mag via esp_foc_u64_rsqrt(m^2) ~ (65536^2)/|V|; then
 * q16_mul chain (normalization + inverse sqrt — esp_foc_int_sqrt.h):
 *   scale = q16_mul(|v_dc|, inv_mag)
 *   v_d' = q16_mul(v_d, scale)
 */
static inline void esp_foc_limit_voltage_q16(q16_t *v_d, q16_t *v_q, q16_t v_dc)
{
    int64_t dc64 = (int64_t)v_dc;
    int64_t dc_abs = (dc64 >= 0) ? dc64 : -dc64;

    if (dc_abs == 0) {
        *v_d = 0;
        *v_q = 0;
        return;
    }

    int64_t vd64 = (int64_t)*v_d;
    int64_t vq64 = (int64_t)*v_q;
    uint64_t av = (uint64_t)(vd64 >= 0 ? vd64 : -vd64);
    uint64_t aq = (uint64_t)(vq64 >= 0 ? vq64 : -vq64);
    uint64_t m2 = av * av + aq * aq;
    uint64_t dc2 = (uint64_t)dc_abs * (uint64_t)dc_abs;

    if (m2 > dc2) {
        uint64_t inv_mag_q = esp_foc_u64_rsqrt(m2);

        if (inv_mag_q > (uint64_t)INT32_MAX) {
            inv_mag_q = (uint64_t)INT32_MAX;
        }

        q16_t inv_mag = (q16_t)inv_mag_q;
        q16_t dc_q = (q16_t)dc_abs;
        q16_t scale = q16_mul(dc_q, inv_mag);

        *v_d = q16_mul(*v_d, scale);
        *v_q = q16_mul(*v_q, scale);
    }
}

static inline void esp_foc_apply_bias_q16(q16_t *v_alpha, q16_t *v_beta)
{
    *v_alpha = q16_add(q16_mul(*v_alpha, Q16_HALF), Q16_HALF);
    *v_beta = q16_add(q16_mul(*v_beta, Q16_HALF), Q16_HALF);
}

/* Whole microseconds on the scope wire as Q16 float (host: raw/65536). Integer-only
 * so PWM ISR paths never touch the FPU (Xtensa saves FP context elsewhere). */
static inline q16_t hot_path_us_elapsed_to_q16(uint64_t el_us)
{
    const uint64_t cap = 32767ULL; /* INT32_MAX / Q16_ONE */
    if (el_us > cap) {
        el_us = cap;
    }
    return (q16_t)((int64_t)el_us * (int64_t)Q16_ONE);
}

static inline q16_t calc_time_isr_q16(uint64_t hot_path_t0, uint64_t now)
{
    uint64_t hot_path_t1 = now;
    uint64_t el = (hot_path_t1 >= hot_path_t0)
                        ? (hot_path_t1 - hot_path_t0)
                        : 0U;
    if (el > 10000000U) {
        el = 10000000U;
    }

    return hot_path_us_elapsed_to_q16(el);
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
