/*
 * Internal fixed-point helpers for observers (ISR-safe; no float in inlines).
 */
#pragma once

#include "espFoC/utils/esp_foc_iq31.h"
#include "espFoC/utils/foc_math_iq31.h"

static inline iq31_t esp_foc_obs_iq31_div(iq31_t num, iq31_t den)
{
    if (den == 0) {
        return 0;
    }
    int64_t r = ((int64_t)num << 31) / (int64_t)den;
    if (r > (int64_t)IQ31_ONE) {
        return IQ31_ONE;
    }
    if (r < (int64_t)IQ31_MINUS_ONE) {
        return IQ31_MINUS_ONE;
    }
    return (iq31_t)r;
}

/** Map error to ~(-pi, pi] in angle units where IQ31_ONE = 2*pi rad. */
static inline iq31_t esp_foc_obs_wrap_angle_err_pm_pi(iq31_t err)
{
    iq31_t x = iq31_add(err, IQ31_HALF);
    x = iq31_normalize_angle(x);
    return iq31_sub(x, IQ31_HALF);
}
