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
 * Mantissa must land in [2^31, 2^32) or the NR path saturates.
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

    uint64_t m64 = x >> shr;
    while (m64 > UINT32_MAX) {
        shr += 2u;
        m64 = x >> shr;
    }
    while (m64 < (1ULL << 31) && shr >= 2u) {
        shr -= 2u;
        m64 = x >> shr;
    }
    if (m64 == 0u) {
        m64 = 1u;
    }

    uint32_t m = (uint32_t)m64;
    uint32_t g = esp_foc_inv_sqrt_u32_q32(m);
    unsigned half_sh = shr >> 1;
    if (half_sh >= 32u) {
        return 1u;
    }
    return g >> half_sh;
}

/** floor(sqrt(x)), x >= 0. Bit-at-a-time; ISR-safe, no soft-float. */
static inline uint64_t esp_foc_u64_sqrt(uint64_t x)
{
    if (x == 0u) {
        return 0u;
    }

    uint64_t res = 0;
    uint64_t bit = 1ULL << 62;

    while (bit > x) {
        bit >>= 2;
    }

    while (bit != 0u) {
        uint64_t trial = res + bit;
        if (x >= trial) {
            x -= trial;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }

    return res;
}

/**
 * min( (65536^2) / sqrt(x), INT32_MAX ); x > 0 (typical: vd^2+vq^2).
 */
static inline uint64_t esp_foc_u64_rsqrt(uint64_t x)
{
    if (x == 0u) {
        return (uint64_t)INT32_MAX;
    }

    uint64_t mag = esp_foc_u64_sqrt(x);
    if (mag == 0u) {
        return (uint64_t)INT32_MAX;
    }

    uint64_t hi = ((65536ULL * 65536ULL) + (mag >> 1)) / mag;
    if (hi > (uint64_t)INT32_MAX) {
        return (uint64_t)INT32_MAX;
    }
    return hi;
}

#ifdef __cplusplus
}
#endif
