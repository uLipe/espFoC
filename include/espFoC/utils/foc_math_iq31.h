/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file foc_math_iq31.h
 * @brief FOC math in fixed-point Q1.31 (IQ31): Clarke/Park transforms,
 *        normalize angle, limit voltage, apply bias, third harmonic.
 *        Mirrors the scope of foc_math.h. Depends on esp_foc_iq31.h.
 */

#pragma once

#include "espFoC/utils/esp_foc_iq31.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --- Clarke / Park transforms (all in/out Q1.31) --- */

/** Clarke: u,v,w (3-phase, sum assumed 0) -> alpha, beta. alpha=u, beta=(u+2*v)*K3. */
static inline void iq31_clarke(iq31_t u, iq31_t v, iq31_t w,
                               iq31_t *alpha, iq31_t *beta)
{
    (void)w;
    iq31_t sum_u2v = iq31_add(iq31_add(u, v), v);
    *alpha = u;
    *beta  = iq31_mul(sum_u2v, IQ31_K3_CLARKE);
}

/** Park: alpha, beta and sin, cos of angle -> d, q */
static inline void iq31_park(iq31_t sin_t, iq31_t cos_t, iq31_t alpha, iq31_t beta,
                            iq31_t *d, iq31_t *q)
{
    *d = iq31_add(iq31_mul(alpha, cos_t), iq31_mul(beta, sin_t));
    *q = iq31_sub(iq31_mul(beta, cos_t), iq31_mul(alpha, sin_t));
}

/** Inverse Park: d, q and sin, cos -> alpha, beta */
static inline void iq31_inverse_park(iq31_t sin_t, iq31_t cos_t, iq31_t d, iq31_t q,
                                    iq31_t *alpha, iq31_t *beta)
{
    *alpha = iq31_sub(iq31_mul(d, cos_t), iq31_mul(q, sin_t));
    *beta  = iq31_add(iq31_mul(d, sin_t), iq31_mul(q, cos_t));
}

/** Inverse Clarke: alpha, beta -> u, v, w (sum = 0). v = (sqrt3/2)*beta - alpha/2, w = -u - v. */
static inline void iq31_inverse_clarke(iq31_t alpha, iq31_t beta,
                                      iq31_t *u, iq31_t *v, iq31_t *w)
{
    *u = alpha;
    *v = iq31_sub(iq31_mul(beta, IQ31_SQRT3_OVER_2), iq31_mul(alpha, IQ31_HALF));
    *w = iq31_sub(iq31_sub(0, alpha), *v);
}

/* --- Normalize angle to [0, 2*pi) in Q1.31 --- */

/** Angle in [0, IQ31_ONE); 2*pi (IQ31_ONE) wraps to 0. */
static inline iq31_t iq31_normalize_angle(iq31_t angle_q31)
{
    uint32_t u = (uint32_t)angle_q31 & 0x7FFFFFFFU;
    uint32_t period = (uint32_t)IQ31_ONE + 1u;
    u = u % period;
    if (u == (uint32_t)IQ31_ONE) {
        return 0;  /* 2*pi -> 0 */
    }
    return (iq31_t)u;
}

/* --- Limit voltage: scale (v_d, v_q) to fit in circle of radius v_dc --- */

/**
 * If |(v_d, v_q)| > v_dc, scale both by v_dc/|v| so result lies on the circle.
 * All arguments in Q1.31. No-op if magnitude squared is zero or already <= v_dc.
 */
static inline void esp_foc_limit_voltage_iq31(iq31_t *v_d, iq31_t *v_q, iq31_t v_dc)
{
    int64_t vd = (int64_t)*v_d;
    int64_t vq = (int64_t)*v_q;
    uint64_t sum_sq = (uint64_t)(vd * vd) + (uint64_t)(vq * vq);
    uint32_t mag_sq_u = (uint32_t)(sum_sq >> 31);
    if (mag_sq_u > 0x7FFFFFFFU) {
        mag_sq_u = 0x7FFFFFFFU;
    }
    iq31_t mag_sq = (iq31_t)mag_sq_u;

    int64_t vdc_sq = (int64_t)v_dc * (int64_t)v_dc;
    uint32_t vdc_sq_u = (uint32_t)(vdc_sq >> 31);
    if (vdc_sq_u > 0x7FFFFFFFU) {
        vdc_sq_u = 0x7FFFFFFFU;
    }
    iq31_t v_dc_sq = (iq31_t)vdc_sq_u;

    if (mag_sq > v_dc_sq && mag_sq > 0) {
        iq31_t r = iq31_rsqrt_fast(mag_sq);
        iq31_t scale = iq31_mul_q230(v_dc, r);
        *v_d = iq31_mul(*v_d, scale);
        *v_q = iq31_mul(*v_q, scale);
    }
}

/* --- Apply bias: map [-1,1] -> [0,1] (v*0.5 + 0.5) --- */

static inline void esp_foc_apply_bias_iq31(iq31_t *v_alpha, iq31_t *v_beta)
{
    *v_alpha = iq31_add(iq31_mul(*v_alpha, IQ31_HALF), IQ31_HALF);
    *v_beta  = iq31_add(iq31_mul(*v_beta, IQ31_HALF), IQ31_HALF);
}

/* --- Third harmonic injection (optional, CONFIG_ESP_FOC_COMP_THI) --- */

#ifdef CONFIG_ESP_FOC_COMP_THI
static inline void esp_foc_third_harmonic_injection_iq31(iq31_t *v_alpha, iq31_t *v_beta)
{
    iq31_t v_max = iq31_max(iq31_max(*v_alpha, *v_beta), 0);
    iq31_t v_min = iq31_min(iq31_min(*v_alpha, *v_beta), 0);
    iq31_t v_offset = (iq31_t)(((int64_t)v_max + (int64_t)v_min) >> 1);
    *v_alpha = iq31_sub(*v_alpha, v_offset);
    *v_beta  = iq31_sub(*v_beta, v_offset);
    *v_alpha = iq31_mul(*v_alpha, iq31_from_float(0.95f));
    *v_beta  = iq31_mul(*v_beta, iq31_from_float(0.95f));
}
#endif

#ifdef __cplusplus
}
#endif
