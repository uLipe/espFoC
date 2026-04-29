/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#pragma once

#include "espFoC/utils/esp_foc_q16.h"

/**
 * Observer I/O is Q16.16 per-unit for voltages/currents; angle is electrical angle [rad] Q16.
 *
 * get_angle: electrical angle in [0, Q16_TWO_PI) radians.
 * get_speed: electrical ω [rad/s] normalized by ESP_FOC_OBS_OMEGA_MAX_RAD_S (dimensionless Q16).
 */
#define ESP_FOC_OBS_OMEGA_MAX_RAD_S (10000.0f)

typedef struct esp_foc_observer_s esp_foc_observer_t;

typedef struct {
    q16_t u_dq[2];
    q16_t u_alpha_beta[2];
    q16_t i_dq[2];
    q16_t i_alpha_beta[2];
    q16_t dt;
    q16_t inv_dt;
} esp_foc_observer_inputs_t;

struct esp_foc_observer_s {
    int (*update)(esp_foc_observer_t *self, esp_foc_observer_inputs_t *in);
    q16_t (*get_angle)(esp_foc_observer_t *self);
    q16_t (*get_speed)(esp_foc_observer_t *self);
    void (*reset)(esp_foc_observer_t *self, q16_t offset_angle_rad);
};

static inline q16_t esp_foc_obs_omega_to_q16(float omega_rad_s)
{
    return q16_from_float(omega_rad_s / ESP_FOC_OBS_OMEGA_MAX_RAD_S);
}

static inline float esp_foc_obs_omega_from_q16(q16_t w)
{
    return q16_to_float(w) * ESP_FOC_OBS_OMEGA_MAX_RAD_S;
}
