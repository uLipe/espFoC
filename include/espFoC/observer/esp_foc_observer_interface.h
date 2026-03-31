/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#pragma once

#include "espFoC/utils/esp_foc_iq31.h"

/**
 * Observer I/O is entirely IQ31 so update() is safe for ISR (no float).
 *
 * - u_*, i_*: per-unit voltages/currents in Q1.31 (same convention as the rest of espFoC).
 * - dt, inv_dt: sample time (s) and 1/dt as iq31_from_float(...) at the control rate.
 *
 * get_angle: electrical angle in [0, IQ31_ONE) mapping to [0, 2π) rad (same as iq31_sin).
 * get_speed: electrical ω [rad/s] normalized by ESP_FOC_OBS_OMEGA_MAX_RAD_S (see below).
 *
 * PLL/KF use the Q16.16 EMA low-pass internally; per-unit IQ31 is converted at the LPF
 * boundary (see esp_foc_iq31_q16_bridge.h). Other observer state stays IQ31.
 */
#define ESP_FOC_OBS_OMEGA_MAX_RAD_S (10000.0f)

typedef struct esp_foc_observer_s esp_foc_observer_t;

typedef struct {
    iq31_t u_dq[2];
    iq31_t u_alpha_beta[2];
    iq31_t i_dq[2];
    iq31_t i_alpha_beta[2];
    iq31_t dt;
    iq31_t inv_dt;
} esp_foc_observer_inputs_t;

struct esp_foc_observer_s {
    int (*update)(esp_foc_observer_t *self, esp_foc_observer_inputs_t *in);
    iq31_t (*get_angle)(esp_foc_observer_t *self);
    iq31_t (*get_speed)(esp_foc_observer_t *self);
    void (*reset)(esp_foc_observer_t *self, iq31_t offset_angle);
};

/** ω [rad/s] ↔ iq31 for get_speed / internal omega state */
static inline iq31_t esp_foc_obs_omega_to_q31(float omega_rad_s)
{
    return iq31_from_float(omega_rad_s / ESP_FOC_OBS_OMEGA_MAX_RAD_S);
}

static inline float esp_foc_obs_omega_from_q31(iq31_t w)
{
    return iq31_to_float(w) * ESP_FOC_OBS_OMEGA_MAX_RAD_S;
}
