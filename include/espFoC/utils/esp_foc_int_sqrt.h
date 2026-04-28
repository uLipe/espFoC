/*
 * MIT License
 *
 * Deterministic integer square root (digit-by-bit). Fixed iteration count,
 * no Newton loops — safe for ISR / hard-real-time FoC paths.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline uint32_t esp_foc_u32_isqrt(uint32_t x)
{
    if (x <= 1u) {
        return x;
    }
    uint32_t rem = x;
    uint32_t res = 0;
    uint32_t bit = 1u << 30;
    while (bit > rem) {
        bit >>= 2;
    }
    while (bit != 0) {
        if (rem >= res + bit) {
            rem -= res + bit;
            res = (res >> 1u) + bit;
        } else {
            res >>= 1u;
        }
        bit >>= 2;
    }
    return res;
}

static inline uint64_t esp_foc_u64_isqrt(uint64_t x)
{
    if (x == 0u) {
        return 0u;
    }
    uint64_t rem = x;
    uint64_t res = 0;
    uint64_t bit = (uint64_t)1 << 62;
    while (bit > rem) {
        bit >>= 2;
    }
    while (bit != 0) {
        if (rem >= res + bit) {
            rem -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

#ifdef __cplusplus
}
#endif
