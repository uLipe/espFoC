/*
 * MIT License
 */
/**
 * @file esp_foc_iq31_q16_bridge.h
 * @brief Convert per-unit scalars between Q1.31 ([-1,1]) and Q16.16 (engineering scale).
 *
 * Use at boundaries when IQ31 observer math meets Q16.16 blocks (e.g. EMA LPF).
 */
#pragma once

#include <stdint.h>
#include "espFoC/utils/esp_foc_iq31.h"
#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Map IQ31 per-unit value to Q16.16 (same real value: v = x/2^31 = q/65536). */
static inline q16_t esp_foc_iq31_per_unit_to_q16(iq31_t x)
{
    int64_t t = ((int64_t)x * (int64_t)Q16_ONE) >> 31;
    if (t > (int64_t)INT32_MAX) {
        return (q16_t)INT32_MAX;
    }
    if (t < (int64_t)INT32_MIN) {
        return (q16_t)INT32_MIN;
    }
    return (q16_t)t;
}

/** Map Q16.16 per-unit value back to IQ31 (saturates to IQ31_ONE / IQ31_MINUS_ONE). */
static inline iq31_t esp_foc_q16_per_unit_to_iq31(q16_t x)
{
    int64_t t = ((int64_t)x * (1LL << 31)) / (int64_t)Q16_ONE;
    if (t > (int64_t)IQ31_ONE) {
        return IQ31_ONE;
    }
    if (t < (int64_t)IQ31_MINUS_ONE) {
        return IQ31_MINUS_ONE;
    }
    return (iq31_t)t;
}

#ifdef __cplusplus
}
#endif
