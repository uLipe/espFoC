/*
 * MIT License
 *
 * ISR-safe inverse magnitude helpers for voltage limiting: normalize uint64
 * to a 32-bit mantissa, Quake-style seed + Newton–Raphson in Q16.16, then
 * denormalize — no digit-by-bit isqrt, no soft-float.
 */
#pragma once

#include <limits.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * ~2^32 / sqrt(m), m >= 1. Quake seed + two NR steps (integer only).
 */
static inline uint32_t esp_foc_inv_sqrt_u32_q32(uint32_t m)
{
    if (m <= 1u) {
        return 0xFFFFFFFFu;
    }

    uint32_t i = 0x5f3759dfu - (m >> 1);
    uint64_t y = (uint64_t)i;

    for (int k = 0; k < 2; k++) {
        uint64_t y2 = (y * y) >> 16;
        uint64_t my2 = ((uint64_t)m * y2) >> 16;
        y = (y * (98304ULL - (my2 >> 1))) >> 16;
    }

    uint64_t inv_q32 = y << 16;
    if (inv_q32 > 0xFFFFFFFFULL) {
        return 0xFFFFFFFFu;
    }
    return (uint32_t)inv_q32;
}

/**
 * ~2^32 / sqrt(x), x >= 1. MSB normalization + esp_foc_inv_sqrt_u32_q32.
 */
static inline uint32_t esp_foc_u64_inv_sqrt_q32(uint64_t x)
{
    if (x <= 1u) {
        return 0xFFFFFFFFu;
    }

    int lz = __builtin_clzll(x);
    int msb = 63 - lz;
    unsigned shr = msb > 31 ? (unsigned)(msb - 31) : 0u;
    shr &= ~1u;

    uint32_t m = (uint32_t)(x >> shr);
    if (m == 0u) {
        m = 1u;
    }

    uint32_t g = esp_foc_inv_sqrt_u32_q32(m);
    unsigned half_sh = shr >> 1;
    if (half_sh >= 32u) {
        return 1u;
    }
    return g >> half_sh;
}

/**
 * min( (65536^2) / sqrt(x), INT32_MAX ); x > 0 (typical: vd^2+vq^2).
 */
static inline uint64_t esp_foc_u64_rsqrt(uint64_t x)
{
    if (x == 0u) {
        return (uint64_t)INT32_MAX;
    }

    uint32_t inv_q = esp_foc_u64_inv_sqrt_q32(x);
    uint64_t hi = (((uint64_t)65536ULL * 65536ULL) * (uint64_t)inv_q) >> 32;

    if (hi > (uint64_t)INT32_MAX) {
        return (uint64_t)INT32_MAX;
    }
    return hi;
}

#ifdef __cplusplus
}
#endif
