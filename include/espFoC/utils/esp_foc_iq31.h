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
 * @file esp_foc_iq31.h
 * @brief Fixed-point Q1.31 (IQ31) helpers for espFoC. Optimized for speed over code-size.
 * Value range: [-1.0, +1.0) with scale 2^-31.
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- Type and constants --- */
typedef int32_t iq31_t;

#define IQ31_ONE         ((iq31_t)0x7FFFFFFF)
#define IQ31_MINUS_ONE   ((iq31_t)0x80000000)
#define IQ31_HALF        ((iq31_t)0x40000000)

/* Clarke/Park constants. K3 = 1/sqrt(3), 2*K3 in Q2.30 for (u+2*v)*K3; SQRT3 for inverse Clarke */
#define IQ31_K3_CLARKE       ((iq31_t)0x49E69A76)   /* 1/sqrt(3) in Q1.31 */
#define IQ31_2K3_Q230        ((iq31_t)1239852235)   /* 2/sqrt(3) in Q2.30 */
#define IQ31_SQRT3_OVER_2    ((iq31_t)0x6ED9EBA1)   /* sqrt(3)/2 in Q1.31 */

/* Scale factor for conversion: 2^31 */
#define IQ31_SCALE       (2147483648.0)

/* --- Conversion (saturate to [-1, 1]; invalid/NaN/Inf -> 0) --- */
static inline iq31_t iq31_from_float(float x)
{
    if (x >= 1.0f) {
        return IQ31_ONE;
    }
    if (x <= -1.0f) {
        return IQ31_MINUS_ONE;
    }
    /* NaN / Inf check via comparison (NaN compares false to everything) */
    if (x != x) {
        return 0;
    }
    return (iq31_t)((double)x * (double)0x80000000ULL);
}

static inline float iq31_to_float(iq31_t x)
{
    return (float)x / (float)(1ULL << 31);
}

#define IQ31_FROM_FLOAT(x)   iq31_from_float(x)
#define IQ31_TO_FLOAT(x)     iq31_to_float(x)

/* --- Arithmetic (speed-oriented: no branches where avoidable, round to nearest) --- */

/** Q1.31 * Q1.31 -> Q1.31 with rounding; saturates on overflow/underflow */
static inline iq31_t iq31_mul(iq31_t a, iq31_t b)
{
    int64_t p = (int64_t)a * (int64_t)b;
    p += (1ULL << 30);   /* round to nearest */
    int64_t r = p >> 31;
    if (r > (int64_t)IQ31_ONE) {
        return IQ31_ONE;
    }
    if (r < (int64_t)IQ31_MINUS_ONE) {
        return IQ31_MINUS_ONE;
    }
    return (iq31_t)r;
}

/** Q1.31 + Q1.31 -> Q1.31; saturates on overflow/underflow */
static inline iq31_t iq31_add(iq31_t a, iq31_t b)
{
    int64_t s = (int64_t)a + (int64_t)b;
    if (s > (int64_t)IQ31_ONE) {
        return IQ31_ONE;
    }
    if (s < (int64_t)IQ31_MINUS_ONE) {
        return IQ31_MINUS_ONE;
    }
    return (iq31_t)s;
}

/** Q1.31 - Q1.31 -> Q1.31; saturates on overflow/underflow */
static inline iq31_t iq31_sub(iq31_t a, iq31_t b)
{
    int64_t d = (int64_t)a - (int64_t)b;
    if (d > (int64_t)IQ31_ONE) {
        return IQ31_ONE;
    }
    if (d < (int64_t)IQ31_MINUS_ONE) {
        return IQ31_MINUS_ONE;
    }
    return (iq31_t)d;
}

/** Clamp x to [lo, hi] (all Q1.31) */
static inline iq31_t iq31_clamp(iq31_t x, iq31_t lo, iq31_t hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

/** Absolute value; saturates INT32_MIN to IQ31_ONE to avoid undefined behaviour */
static inline iq31_t iq31_abs(iq31_t x)
{
    if (x == IQ31_MINUS_ONE) {
        return IQ31_ONE;
    }
    if (x < 0) {
        return (iq31_t)(-(int64_t)x);
    }
    return x;
}

static inline iq31_t iq31_min(iq31_t a, iq31_t b)
{
    return (a < b) ? a : b;
}

static inline iq31_t iq31_max(iq31_t a, iq31_t b)
{
    return (a > b) ? a : b;
}

/**
 * Reciprocal square root (1/sqrt(x)) in fixed point. Input x in Q1.31 (positive);
 * output in Q2.30 (value 1.0 = 1<<30). Uses deterministic esp_foc_u32_isqrt + divides.
 * Use: scale_q31 = iq31_mul_q230(v_q31, iq31_rsqrt_fast(mag_sq_q31)) for normalization.
 * Returns 0 if x <= 0 (invalid).
 */
iq31_t iq31_rsqrt_fast(iq31_t x);

/**
 * Multiply Q1.31 by Q2.30 (e.g. from iq31_rsqrt_fast), result Q1.31.
 * r = (a * b_q230) >> 30, with saturation.
 */
static inline iq31_t iq31_mul_q230(iq31_t a, iq31_t b_q230)
{
    int64_t p = (int64_t)a * (int64_t)b_q230;
    int64_t r = (p + (1ULL << 29)) >> 30;
    if (r > (int64_t)IQ31_ONE) {
        return IQ31_ONE;
    }
    if (r < (int64_t)IQ31_MINUS_ONE) {
        return IQ31_MINUS_ONE;
    }
    return (iq31_t)r;
}

/* --- Sin/Cos LUT: angle in [0, 2*pi) as Q1.31 (0..IQ31_ONE = 2*pi). Output Q1.31 [-1, 1]. --- */
iq31_t iq31_sin(iq31_t angle_q31);
iq31_t iq31_cos(iq31_t angle_q31);

/* Clarke/Park, normalize_angle, limit_voltage: see foc_math_iq31.h */

#ifdef __cplusplus
}
#endif
