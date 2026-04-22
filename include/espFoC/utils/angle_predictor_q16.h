/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

/**
 * @file angle_predictor_q16.h
 * @brief Alpha-beta tracker for the electrical rotor angle in Q16.16.
 *
 * Bridges a slow encoder reading (e.g. AS5600 over I2C, ~2 kHz hard
 * cap) and a fast inner control loop (e.g. the FOC PI inside the PWM
 * ISR at 40 kHz). The outer task folds new measurements into the
 * tracker; the ISR queries `predict()` every PWM cycle to get a fresh
 * extrapolated electrical angle, no I2C blocking required.
 *
 * The math is the textbook 2nd-order alpha-beta filter (a constant-gain
 * Kalman simplification used for radar and motor tracking):
 *
 *     predict(t_now):
 *         dt = t_now - t_last
 *         theta_pred = wrap_2pi(theta_est + omega_est * dt)
 *         return theta_pred                      (state untouched)
 *
 *     update(theta_meas, t_meas):
 *         dt = t_meas - t_last
 *         theta_pred = wrap_2pi(theta_est + omega_est * dt)
 *         err        = wrap_pi(theta_meas - theta_pred)   in (-pi, +pi]
 *         theta_est  = wrap_2pi(theta_pred + alpha * err)
 *         omega_est += (beta / dt) * err
 *         t_last     = t_meas
 *
 * Critical-damping rule of thumb: beta = alpha^2 / (2 - alpha). The
 * defaults exposed via Kconfig (alpha = 0.30, beta = 0.05) sit close
 * to that for a 2 kHz update rate.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    q16_t alpha;
    q16_t beta;
    q16_t theta_est;     /* electrical angle, [0, 2*pi) */
    q16_t omega_est;     /* electrical speed, rad/s */
    uint64_t t_last_us;  /* esp_timer monotonic, set on init / update */
    bool initialised;
} esp_foc_angle_predictor_q16_t;

/**
 * @brief Reset state and program the gains. Sets t_last to t_now_us
 *        so the first predict() returns whatever theta_est was
 *        initialised to without an enormous extrapolation step.
 *
 * @param alpha measurement gain in (0, 1]
 * @param beta  velocity gain (typ alpha^2 / (2 - alpha))
 */
void esp_foc_angle_predictor_init_q16(esp_foc_angle_predictor_q16_t *p,
                                      float alpha, float beta,
                                      uint64_t t_now_us);

/**
 * @brief Reprogram the gains without touching state. Safe to call
 *        between updates; new gains take effect on the next update().
 */
void esp_foc_angle_predictor_set_gains_q16(esp_foc_angle_predictor_q16_t *p,
                                           float alpha, float beta);

/**
 * @brief Outer task: fold a fresh sensor reading into the tracker.
 *        theta_meas is the electrical angle in radians (q16), expected
 *        in [0, 2*pi). The residual is wrapped to (-pi, +pi] so the
 *        2*pi boundary never produces a huge omega step.
 *
 *        First call after init seeds theta_est = theta_meas and leaves
 *        omega_est at zero — no spurious velocity from the inaugural dt.
 */
void esp_foc_angle_predictor_update_q16(esp_foc_angle_predictor_q16_t *p,
                                        q16_t theta_meas,
                                        uint64_t t_meas_us);

/**
 * @brief PWM ISR: extrapolate to t_now. Cheap (one mul, one add, one
 *        wrap). Does NOT mutate state — concurrent calls from the
 *        outer task and the ISR are safe as long as update() is
 *        bracketed by a critical section the ISR also honours.
 */
q16_t esp_foc_angle_predictor_predict_q16(
    const esp_foc_angle_predictor_q16_t *p,
    uint64_t t_now_us);

/**
 * @brief Convenience accessor: latest electrical speed estimate
 *        (rad/s, q16). Read-only; safe from any context.
 */
static inline q16_t esp_foc_angle_predictor_speed_q16(
    const esp_foc_angle_predictor_q16_t *p)
{
    return p->omega_est;
}

#ifdef __cplusplus
}
#endif
