/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Positional PI with gains and integrator in Q32: Ki·Ts of a slow loop is
 * under one Q16 LSB. P never enters the state, and the integrator only
 * takes a step that does not push the output further into its clamp, so
 * Kp·noise riding the limit is not integrated into a drift.
 */
typedef struct {
    int64_t kp_q32;
    int64_t ki_ts_q32;
    int64_t i_q32;
    int64_t i_lim_q32;
} esp_foc_pi_q32_t;

/** kp, ki >= 0 and fs_hz > 0; keeps the integrator. */
esp_err_t esp_foc_pi_q32_set_gains(esp_foc_pi_q32_t *pi, float kp, float ki, float fs_hz);

/** Integrator clamp, normally the output limit passed to update(). */
static inline void esp_foc_pi_q32_set_limit(esp_foc_pi_q32_t *pi, q16_t lim)
{
    pi->i_lim_q32 = (int64_t)lim << 16;
}

/** Bumpless: the next output with zero error and no ff is held. */
static inline void esp_foc_pi_q32_reset(esp_foc_pi_q32_t *pi, q16_t held)
{
    pi->i_q32 = (int64_t)held << 16;
}

/** u = Kp·e + ∫Ki·e + ff, clamped to ±lim (lim >= 0). O(1), ISR-safe. */
static inline q16_t esp_foc_pi_q32_update(esp_foc_pi_q32_t *pi, q16_t e, q16_t ff, q16_t lim)
{
    const int64_t kp = pi->kp_q32;
    const int64_t ki_ts = pi->ki_ts_q32;
    const int64_t i_old = pi->i_q32;
    const int64_t i_lim = pi->i_lim_q32;

    const int64_t inc = (ki_ts * (int64_t)e) >> 16;
    int64_t i_new = i_old + inc;
    if (i_new > i_lim) {
        i_new = i_lim;
    } else if (i_new < -i_lim) {
        i_new = -i_lim;
    }
    int64_t u = ((kp * (int64_t)e) >> 32) + (i_new >> 16) + (int64_t)ff;
    bool hold = false;
    if (u > (int64_t)lim) {
        u = lim;
        hold = (inc > 0);
    } else if (u < -(int64_t)lim) {
        u = -(int64_t)lim;
        hold = (inc < 0);
    }
    if (!hold) {
        pi->i_q32 = i_new;
    }
    return (q16_t)u;
}

#ifdef __cplusplus
}
#endif
