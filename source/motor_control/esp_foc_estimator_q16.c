/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include "espFoC/esp_foc_estimator_q16.h"
#include "espFoC/utils/foc_math_q16.h"
#include "esp_attr.h"

static inline q16_t q16_wrap_turn_delta(q16_t delta)
{
    while (delta > Q16_HALF) {
        delta = q16_sub(delta, Q16_ONE);
    }
    while (delta < (q16_t)(-Q16_HALF)) {
        delta = q16_add(delta, Q16_ONE);
    }
    return delta;
}

static inline q16_t q16_neg(q16_t v)
{
    if (v == (q16_t)INT32_MIN) {
        return (q16_t)INT32_MAX;
    }
    return (q16_t)(-(int64_t)v);
}

void esp_foc_estimator_q16_init(esp_foc_estimator_q16_t *est,
                                const esp_foc_estimator_q16_config_t *cfg)
{
    if (est == NULL || cfg == NULL) {
        return;
    }

    float dt = q16_to_float(cfg->dt_isr);
    float wn = cfg->pll_bw_hz;
    float kp = 2.0f * wn * dt;
    float ki = wn * wn * dt;

    est->dt = cfg->dt_isr;
    est->kp = q16_from_float(kp);
    est->ki = q16_from_float(ki);
    est->omega_max = cfg->omega_max_mech;
    est->pole_pairs = (cfg->pole_pairs < 1) ? 1 : cfg->pole_pairs;

    esp_foc_estimator_q16_reset(est);
}

void esp_foc_estimator_q16_reset(esp_foc_estimator_q16_t *est)
{
    if (est == NULL) {
        return;
    }
    est->theta_est_mech = 0;
    est->omega_est_mech = 0;
    est->theta_meas_mech = 0;
    est->pll_err = 0;
}

void esp_foc_estimator_q16_snap(esp_foc_estimator_q16_t *est, q16_t theta_meas_mech)
{
    if (est == NULL) {
        return;
    }
    theta_meas_mech = q16_normalize_angle(theta_meas_mech);
    est->theta_meas_mech = theta_meas_mech;
    est->theta_est_mech = theta_meas_mech;
    est->omega_est_mech = 0;
    est->pll_err = 0;
}

void esp_foc_estimator_q16_set_meas(esp_foc_estimator_q16_t *est, q16_t theta_meas_mech)
{
    if (est == NULL) {
        return;
    }
    est->theta_meas_mech = q16_normalize_angle(theta_meas_mech);
}

void esp_foc_estimator_q16_set_pole_pairs(esp_foc_estimator_q16_t *est, int pole_pairs)
{
    if (est == NULL) {
        return;
    }
    est->pole_pairs = (pole_pairs < 1) ? 1 : pole_pairs;
}

void IRAM_ATTR esp_foc_estimator_q16_step(esp_foc_estimator_q16_t *est)
{
    q16_t theta_meas = est->theta_meas_mech;
    q16_t err = q16_wrap_turn_delta(q16_sub(theta_meas, est->theta_est_mech));
    est->pll_err = err;

    q16_t omega = q16_add(est->omega_est_mech, q16_mul(est->ki, err));
    omega = q16_clamp(omega, q16_neg(est->omega_max), est->omega_max);
    est->omega_est_mech = omega;

    q16_t theta = q16_add(est->theta_est_mech, q16_mul(omega, est->dt));
    theta = q16_add(theta, q16_mul(est->kp, err));
    est->theta_est_mech = q16_normalize_angle(theta);
}
