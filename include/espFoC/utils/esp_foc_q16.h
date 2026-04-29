/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

/**
 * @file esp_foc_q16.h
 * @brief Signed Q16.16 fixed-point (int32_t). ~±32768 with ~1/65536 resolution.
 *        Used as the canonical scalar format for espFoC control signals and parameters.
 */

#pragma once

#include <math.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t q16_t;

#define Q16_FRAC_BITS 16

/** 1.0 in Q16.16 */
#define Q16_ONE ((q16_t)65536)
/** 0.5 */
#define Q16_HALF ((q16_t)32768)
/** -1.0 */
#define Q16_MINUS_ONE ((q16_t)-65536)

/** 2*pi radians (for angle helpers) */
#define Q16_TWO_PI ((q16_t)411775)

#define Q16_K3_CLARKE ((q16_t)37837)
#define Q16_SQRT3_OVER_2 ((q16_t)56756)

static inline q16_t q16_from_float(float x)
{
    double d = (double)x * 65536.0;
    if (d >= 2147483647.0) {
        return (q16_t)2147483647;
    }
    if (d <= -2147483648.0) {
        return (q16_t)(-2147483647 - 1);
    }
    if (d != d) {
        return 0;
    }
    return (q16_t)(int64_t)(d >= 0.0 ? d + 0.5 : d - 0.5);
}

static inline float q16_to_float(q16_t x)
{
    return (float)x / 65536.0f;
}

static inline q16_t q16_from_int(int32_t v)
{
    int64_t x = (int64_t)v * (int64_t)Q16_ONE;
    if (x > (int64_t)INT32_MAX) {
        return (q16_t)INT32_MAX;
    }
    if (x < (int64_t)INT32_MIN) {
        return (q16_t)INT32_MIN;
    }
    return (q16_t)x;
}

static inline q16_t q16_mul(q16_t a, q16_t b)
{
    int64_t p = (int64_t)a * (int64_t)b;
    p += (1LL << (Q16_FRAC_BITS - 1));
    int64_t r = p >> Q16_FRAC_BITS;
    if (r > (int64_t)INT32_MAX) {
        return (q16_t)INT32_MAX;
    }
    if (r < (int64_t)INT32_MIN) {
        return (q16_t)INT32_MIN;
    }
    return (q16_t)r;
}

static inline q16_t q16_add(q16_t a, q16_t b)
{
    int64_t s = (int64_t)a + (int64_t)b;
    if (s > (int64_t)INT32_MAX) {
        return (q16_t)INT32_MAX;
    }
    if (s < (int64_t)INT32_MIN) {
        return (q16_t)INT32_MIN;
    }
    return (q16_t)s;
}

static inline q16_t q16_sub(q16_t a, q16_t b)
{
    int64_t d = (int64_t)a - (int64_t)b;
    if (d > (int64_t)INT32_MAX) {
        return (q16_t)INT32_MAX;
    }
    if (d < (int64_t)INT32_MIN) {
        return (q16_t)INT32_MIN;
    }
    return (q16_t)d;
}

static inline q16_t q16_clamp(q16_t x, q16_t lo, q16_t hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

static inline q16_t q16_abs(q16_t v)
{
    if (v == (q16_t)INT32_MIN) {
        return (q16_t)INT32_MAX;
    }
    return (v < 0) ? (q16_t)(-(int64_t)v) : v;
}

static inline q16_t q16_min(q16_t a, q16_t b)
{
    return (a < b) ? a : b;
}

static inline q16_t q16_max(q16_t a, q16_t b)
{
    return (a > b) ? a : b;
}

/**
 * Reciprocal 1/x for strictly positive Q16 x; x<=0 → 0.
 * Integer-only (ISR-safe). Result is Q16 such that q16_mul(x, rec) ≈ Q16_ONE.
 */
static inline q16_t q16_reciprocal_positive(q16_t x)
{
    if (x <= 0) {
        return 0;
    }
    int64_t num = (int64_t)Q16_ONE * (int64_t)Q16_ONE;
    int64_t r = num / (int64_t)x;
    if (r > (int64_t)INT32_MAX) {
        return (q16_t)INT32_MAX;
    }
    return (q16_t)r;
}

/**
 * Elapsed interval in microseconds → Q16.16 seconds (>= 0).
 * Integer-only: seconds ≈ us * Q16_ONE / 1e6.
 */
static inline q16_t q16_from_elapsed_us_u64(uint64_t us)
{
    int64_t raw = ((int64_t)us * (int64_t)Q16_ONE + 500000LL) / 1000000LL;
    if (raw > (int64_t)INT32_MAX) {
        return (q16_t)INT32_MAX;
    }
    return (q16_t)raw;
}

static inline q16_t q16_from_iq31_per_unit(int32_t iq31)
{
    int64_t x = ((int64_t)iq31 * (int64_t)Q16_ONE) >> 31;
    if (x > INT32_MAX) {
        return (q16_t)INT32_MAX;
    }
    if (x < INT32_MIN) {
        return (q16_t)INT32_MIN;
    }
    return (q16_t)x;
}

q16_t q16_sin(q16_t angle_rad_q16);
q16_t q16_cos(q16_t angle_rad_q16);

#ifdef __cplusplus
}
#endif
