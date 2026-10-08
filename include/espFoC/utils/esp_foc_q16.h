/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdint.h>
#include <limits.h>

/**
 * Q16.16 fixed-point: 32-bit container (int32_t), not a 16-bit type.
 * 16 integer bits + 16 fractional bits; integer range about -32768..+32767.
 * Q16_ONE == 1.0.
 */
typedef int32_t q16_t;

#define Q16_ONE        ((q16_t)65536)
#define Q16_HALF       ((q16_t)32768)
#define Q16_MINUS_ONE  ((q16_t)-65536)

static inline q16_t q16_from_float(float x)
{
    double d = (double)x * 65536.0;
    if (d >= 2147483647.0) {
        return (q16_t)2147483647;
    }
    if (d <= -2147483648.0) {
        return (q16_t)(-2147483647 - 1);
    }
    return (q16_t)(int64_t)(d >= 0.0 ? d + 0.5 : d - 0.5);
}

static inline float q16_to_float(q16_t x)
{
    return (float)((double)x / 65536.0);
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

static inline q16_t q16_add(q16_t a, q16_t b)
{
    int64_t s = (int64_t)a + (int64_t)b;
    if (s > INT32_MAX) {
        return (q16_t)INT32_MAX;
    }
    if (s < INT32_MIN) {
        return (q16_t)INT32_MIN;
    }
    return (q16_t)s;
}

static inline q16_t q16_sub(q16_t a, q16_t b)
{
    int64_t d = (int64_t)a - (int64_t)b;
    if (d > INT32_MAX) {
        return (q16_t)INT32_MAX;
    }
    if (d < INT32_MIN) {
        return (q16_t)INT32_MIN;
    }
    return (q16_t)d;
}

static inline q16_t q16_mul(q16_t a, q16_t b)
{
    int64_t r = ((int64_t)a * (int64_t)b) >> 16;
    if (r > INT32_MAX) {
        return (q16_t)INT32_MAX;
    }
    if (r < INT32_MIN) {
        return (q16_t)INT32_MIN;
    }
    return (q16_t)r;
}

static inline q16_t q16_neg(q16_t v)
{
    if (v == (q16_t)INT32_MIN) {
        return (q16_t)INT32_MAX;
    }
    return (q16_t)(-v);
}

static inline q16_t q16_min(q16_t a, q16_t b)
{
    return a < b ? a : b;
}

static inline q16_t q16_max(q16_t a, q16_t b)
{
    return a > b ? a : b;
}

/** Saturating Q16.16 divide. b==0 → sign(a)*INT32_MAX (0/0 → 0). */
static inline q16_t q16_div(q16_t a, q16_t b)
{
    if (b == 0) {
        if (a > 0) {
            return (q16_t)INT32_MAX;
        }
        if (a < 0) {
            return (q16_t)INT32_MIN;
        }
        return 0;
    }
    int64_t n = ((int64_t)a << 16) / (int64_t)b;
    if (n > INT32_MAX) {
        return (q16_t)INT32_MAX;
    }
    if (n < INT32_MIN) {
        return (q16_t)INT32_MIN;
    }
    return (q16_t)n;
}
