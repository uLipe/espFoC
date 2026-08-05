/*
 * Integer / Q16.16 helpers for drivers.
 */
#pragma once

#include "espFoC/utils/esp_foc_q16.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline q16_t esp_foc_q16_from_adc_diff_clamped(int32_t diff, int32_t range_abs)
{
    int64_t x = (int64_t)diff * (int64_t)Q16_ONE;
    x /= (int64_t)range_abs;
    if (x > (int64_t)INT32_MAX) {
        return (q16_t)INT32_MAX;
    }
    if (x < (int64_t)INT32_MIN) {
        return (q16_t)INT32_MIN;
    }
    return (q16_t)x;
}

static inline int32_t esp_foc_clamp_int32(int32_t x, int32_t lo, int32_t hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

static inline uint32_t esp_foc_q16_duty_ticks(q16_t v_q16, uint32_t period_half_ticks)
{
    q16_t v = q16_clamp(v_q16, 0, Q16_ONE);
    return (uint32_t)(((uint64_t)(uint32_t)v * (uint64_t)period_half_ticks) >> 16);
}

#ifdef __cplusplus
}
#endif
