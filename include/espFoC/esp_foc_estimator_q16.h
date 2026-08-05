/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/foc_math_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Rotor angle / speed estimator (encoder PLL today; sensorless observers later).
 *
 * Mechanical domain: theta in turns [0, Q16_ONE), omega in rev/s Q16.
 * Electrical angle for Park is derived as normalize(theta_mech * pole_pairs).
 */
typedef struct esp_foc_estimator_q16_s {
    q16_t kp;
    q16_t ki;
    q16_t dt;
    q16_t omega_max;
    int pole_pairs;

    q16_t theta_est_mech;
    q16_t omega_est_mech;
    volatile q16_t theta_meas_mech;
    q16_t pll_err;
} esp_foc_estimator_q16_t;

typedef struct {
    q16_t dt_isr;
    float pll_bw_hz;
    q16_t omega_max_mech;
    int pole_pairs;
} esp_foc_estimator_q16_config_t;

void esp_foc_estimator_q16_init(esp_foc_estimator_q16_t *est,
                                const esp_foc_estimator_q16_config_t *cfg);

void esp_foc_estimator_q16_reset(esp_foc_estimator_q16_t *est);

void esp_foc_estimator_q16_snap(esp_foc_estimator_q16_t *est, q16_t theta_meas_mech);

void esp_foc_estimator_q16_set_meas(esp_foc_estimator_q16_t *est, q16_t theta_meas_mech);

void esp_foc_estimator_q16_set_pole_pairs(esp_foc_estimator_q16_t *est, int pole_pairs);

void esp_foc_estimator_q16_step(esp_foc_estimator_q16_t *est);

static inline q16_t esp_foc_estimator_q16_theta_est_mech(
    const esp_foc_estimator_q16_t *est)
{
    return est->theta_est_mech;
}

static inline q16_t esp_foc_estimator_q16_omega_est_mech(
    const esp_foc_estimator_q16_t *est)
{
    return est->omega_est_mech;
}

static inline q16_t esp_foc_estimator_q16_pll_err(const esp_foc_estimator_q16_t *est)
{
    return est->pll_err;
}

static inline q16_t esp_foc_estimator_q16_theta_meas_mech(
    const esp_foc_estimator_q16_t *est)
{
    return est->theta_meas_mech;
}

/** Electrical angle [0, Q16_ONE) for Park; call after step(). */
static inline q16_t esp_foc_estimator_q16_theta_elec(const esp_foc_estimator_q16_t *est)
{
    q16_t pp = q16_from_int(est->pole_pairs);
    return q16_normalize_angle(q16_mul(est->theta_est_mech, pp));
}

#ifdef __cplusplus
}
#endif
