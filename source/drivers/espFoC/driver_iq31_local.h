/*
 * Integer / Q1.31-only helpers for drivers (do not call float driver code).
 */
#pragma once

#include <sdkconfig.h>

#if CONFIG_ESP_FOC_USE_FIXED_POINT

#include "espFoC/utils/esp_foc_iq31.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Map ADC difference in [-2048,2048] to Q1.31 (same range as float path). */
static inline iq31_t esp_foc_iq31_from_adc_diff_clamped(int32_t diff, int32_t range_abs)
{
    int64_t x = (int64_t)diff * (int64_t)IQ31_ONE;
    x /= (int64_t)range_abs;
    if (x > (int64_t)IQ31_ONE) {
        return IQ31_ONE;
    }
    if (x < (int64_t)IQ31_MINUS_ONE) {
        return IQ31_MINUS_ONE;
    }
    return (iq31_t)x;
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

/** v in [0,1] as Q1.31 × integer half-period (comparator ticks) → duty ticks. */
static inline uint32_t esp_foc_iq31_duty_ticks(iq31_t v_q31, uint32_t period_half_ticks)
{
    iq31_t v = iq31_clamp(v_q31, 0, IQ31_ONE);
    return (uint32_t)(((uint64_t)(uint32_t)v * (uint64_t)period_half_ticks) >> 31);
}

/** Map counts in [0, cpr) to [0,1) in Q1.31 (requires cpr > 0). */
static inline iq31_t esp_foc_iq31_from_counts_mod(uint32_t counts_mod, uint32_t cpr)
{
    if (cpr == 0u) {
        return 0;
    }
    return (iq31_t)(((uint64_t)(counts_mod % cpr) * (uint64_t)IQ31_ONE) / (uint64_t)cpr);
}

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_ESP_FOC_USE_FIXED_POINT */
