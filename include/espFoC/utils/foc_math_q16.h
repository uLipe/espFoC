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

/**
 * Clamp (vd,vq) to magnitude <= |v_dc| in Q16 space (same geometry as float hypot).
 *
 * Canonical form: scale = v_dc / |V| with |V| from
 * hypot; then v_d *= scale, v_q *= scale — no component-wise divide by |V|.
 *
 * Integer path: mag = esp_foc_u64_isqrt(vd^2+vq^2); one reciprocal (65536^2)/mag and
 * q16_mul chain (deterministic — see esp_foc_int_sqrt.h):
 *   scale = q16_mul(|v_dc|, inv_mag_q16)  ~  v_dc * 65536 / mag
 *   v_d' = q16_mul(v_d, scale)  ~  v_d * v_dc / mag
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

#if defined(CONFIG_ESP_FOC_VOLTAGE_LIMIT_LINF) && CONFIG_ESP_FOC_VOLTAGE_LIMIT_LINF
    uint64_t mag = (av > aq) ? av : aq;
    if (mag <= (uint64_t)dc_abs) {
        return;
    }
#else
    uint64_t m2 = av * av + aq * aq;
    uint64_t dc2 = (uint64_t)dc_abs * (uint64_t)dc_abs;

    if (m2 <= dc2) {
        return;
    }

    uint64_t mag = esp_foc_u64_isqrt(m2);
    if (mag == 0ULL) {
        return;
    }
#endif

    uint64_t inv_mag_q = (65536ULL * 65536ULL) / mag;
    if (inv_mag_q > (uint64_t)INT32_MAX) {
        inv_mag_q = (uint64_t)INT32_MAX;
    }

    q16_t inv_mag = (q16_t)inv_mag_q;
    q16_t dc_q = (q16_t)dc_abs;
    q16_t scale = q16_mul(dc_q, inv_mag);

    *v_d = q16_mul(*v_d, scale);
    *v_q = q16_mul(*v_q, scale);
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
