/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Angle estimator fed by sparse, timestamped, absolute measurements.
 *
 * Not hall-specific: the contract is "somebody hands me an absolute angle now
 * and then, with a hardware timestamp", which also describes an I2C encoder
 * sampled far slower than the PWM. Between measurements it dead-reckons at the
 * hot-path rate; on each measurement it applies two independent first-order
 * corrections.
 *
 * Why the corrections are independent: frequency is measured straight from the
 * period, so there is no need to let the phase error drive it. Dropping that
 * cross term (λ_c = 0 in a classic type-II loop) leaves the error map lower
 * triangular, with v = ω − ω̂ and e the phase error:
 *
 *     v_{k+1} = (1 − λ_ω) · v_k
 *     e_{k+1} = (1 − λ_θ) · e_k + (1 − λ_ω) · t_s · v_k
 *
 * so the eigenvalues are (1 − λ_θ) and (1 − λ_ω): stable for λ ∈ (0, 2),
 * deadbeat at λ = 1, and monotone without oscillation for λ ∈ (0, 1].
 *
 * The loop is sampled at the *measurement* rate, not the PWM rate, so its
 * bandwidth is constant in radians travelled and scales with speed on its own.
 *
 * Q16.16 throughout; float appears only in the config helper below.
 */

typedef struct {
    /* Phase and frequency correction gains, Q16.16 in (0, 2·Q16_ONE). */
    q16_t lambda_theta;
    q16_t lambda_omega;
    /* Timestamp source frequency [Hz]. */
    uint32_t tick_hz;
    /*
     * Hot-path rate [Hz]. Given as a frequency rather than a period because
     * Q16.16 cannot hold one: 50 µs is 3.28 LSB, so a period expressed that
     * way would be 8% wrong and the dead reckoning would drift by a whole
     * sector every revolution. init() turns this into a Q32 step internally.
     */
    uint32_t step_hz;
    /* Nominal angle travelled between two measurements [rad], for the clamp. */
    q16_t sector_span;
    /* Slack on the clamp, absorbing measurement-table error [rad]. */
    q16_t clamp_margin;
    /* Steps without a measurement before ω̂ is decayed toward zero. */
    uint32_t standstill_periods;
    /* Geometric factor applied to ω̂ per step past the standstill timeout. */
    q16_t omega_decay;
    /* Reject measurements closer together than this — contact bounce. */
    uint32_t dticks_min;
    /* Above this the period is too stale to be a velocity; re-anchor only. */
    uint32_t dticks_max;
} esp_foc_rotor_est_config_t;

typedef struct {
    esp_foc_rotor_est_config_t cfg;
    /* Angle of the last measurement, wrapped. Anchor of the dead reckoning. */
    q16_t theta_anchor;
    /* Signed travel since that anchor. Bounded by the clamp, so it cannot run. */
    q16_t adv;
    /* wrap(theta_anchor + adv), kept so the getter is a plain load. */
    q16_t theta_hat;
    q16_t omega_hat;
    /* Hot-path period in Q32 seconds: ω_q16 · dt_q32 >> 32 is the Q16 step. */
    uint32_t dt_q32;
    uint32_t half_dt_q32;
    q16_t clamp_lo;
    q16_t clamp_hi;
    uint64_t t_last;
    uint32_t idle_periods;
    /* Health. */
    uint32_t edges;
    uint32_t rejected;
    uint32_t clamped;
    uint32_t stale;
    int8_t dir;
    bool have_anchor;
    bool have_time;
    bool inited;
} esp_foc_rotor_est_t;

/**
 * Physical units → config. Floats are allowed here and nowhere else.
 *
 * @param step_hz         rate at which esp_foc_rotor_est_step() will be called
 * @param sector_span_rad nominal angle between two measurements
 * @param standstill_ms   silence after which ω̂ starts decaying
 */
void esp_foc_rotor_est_config_default(esp_foc_rotor_est_config_t *cfg,
                                      uint32_t tick_hz,
                                      uint32_t step_hz,
                                      float sector_span_rad,
                                      float standstill_ms);

esp_err_t esp_foc_rotor_est_init(esp_foc_rotor_est_t *e,
                                 const esp_foc_rotor_est_config_t *cfg);

/** Drop the estimate; config is kept. */
void esp_foc_rotor_est_reset(esp_foc_rotor_est_t *e);

/**
 * A measurement arrived. ISR-safe. One division per call.
 *
 * @param theta_meas absolute angle at the measurement, wrapped to (−π, +π]
 * @param ticks      hardware timestamp of the measurement
 * @param dir        +1 / −1 travel direction, 0 when unknown
 */
void esp_foc_rotor_est_on_edge(esp_foc_rotor_est_t *e,
                               q16_t theta_meas,
                               uint64_t ticks,
                               int dir);

/** One hot-path period. ISR-safe, O(1), no division, no timebase read. */
void esp_foc_rotor_est_step(esp_foc_rotor_est_t *e);

static inline q16_t esp_foc_rotor_est_get_theta(const esp_foc_rotor_est_t *e)
{
    return (e != NULL) ? e->theta_hat : 0;
}

static inline q16_t esp_foc_rotor_est_get_omega(const esp_foc_rotor_est_t *e)
{
    return (e != NULL) ? e->omega_hat : 0;
}

static inline bool esp_foc_rotor_est_is_moving(const esp_foc_rotor_est_t *e)
{
    return e != NULL && e->have_anchor &&
           e->idle_periods <= e->cfg.standstill_periods;
}

#ifdef __cplusplus
}
#endif
