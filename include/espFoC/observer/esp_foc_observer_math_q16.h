/*
 * Internal fixed-point helpers for observers (ISR-safe; no float in inlines).
 */
#pragma once

#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/foc_math_q16.h"

#define ESP_FOC_OBS_Q16_PI_RAD ((q16_t)((Q16_TWO_PI + 1) / 2))

static inline q16_t esp_foc_obs_q16_div(q16_t num, q16_t den)
{
    if (den == 0) {
        return 0;
    }
    int64_t r = ((int64_t)num * (int64_t)Q16_ONE) / (int64_t)den;
    if (r > (int64_t)Q16_ONE) {
        return Q16_ONE;
    }
    if (r < (int64_t)Q16_MINUS_ONE) {
        return Q16_MINUS_ONE;
    }
    return (q16_t)r;
}

static inline q16_t esp_foc_obs_wrap_angle_err_pm_pi_rad(q16_t err_rad)
{
    q16_t x = q16_add(err_rad, ESP_FOC_OBS_Q16_PI_RAD);
    x = q16_normalize_angle_rad(x);
    return q16_sub(x, ESP_FOC_OBS_Q16_PI_RAD);
}
