/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "espFoC/drivers/esp_foc_rotor_sensor.h"
#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Type-2 tracking loop over an absolute angle sampled at a fixed rate.
 *
 * This is the *sensored* estimator. The sensorless stack has its own
 * (esp_foc_observer_*, driven by iαβ/vαβ) and the hall sensor has its own
 * (esp_foc_rotor_est, driven by sparse timestamped edges); neither fits here
 * and neither is touched by this unit.
 *
 * Why a third one. An encoder read on a fixed clock is the dual of a hall
 * edge: the timestamp is exact and the *angle* is quantized. Estimating speed
 * as Δθ/Δt therefore divides a quantum by a constant, so the noise floor is
 * q·f_s regardless of how slowly the rotor turns — on a 12-bit encoder at
 * 2.5 kHz that is 3.8 rad/s mechanical per LSB, and it is why the naive
 * estimate has to be followed by a low-pass tight enough to eat the phase
 * margin of whatever loop consumes it. A hall edge is the opposite: θ is exact
 * at the sector boundary and the period carries the information, so precision
 * *improves* as the rotor slows.
 *
 * The loop driven by the angle error has no such floor. Quantization is
 * averaged over roughly 1/bw seconds, the bandwidth is a design parameter
 * instead of a consequence of the encoder, and the caller gets one filter it
 * chose rather than two it inherited.
 *
 *     e     = wrap(θ_meas − θ̂)
 *     ω̂    ← ω̂ + k_i · T_s · e
 *     θ̂    ← wrap(θ̂ + T_s · (ω̂ + k_p · e))
 *
 * Type 2, so a constant speed is tracked with zero steady-state angle error:
 * the integrator, not the proportional path, carries ω̂.
 *
 * θ̂ leads the measurement by exactly one period. The correction is applied to
 * the instant that was measured and the result is then integrated forward, so
 * the value a caller reads is the prediction for the instant it will be used
 * at — which is what keeps the consumer off an angle that is already stale by
 * the time it reaches Park. ω̂ carries no such offset.
 *
 * Domain-agnostic. Feed it electrical angle and ω̂ is electrical; feed it
 * mechanical and ω̂ is mechanical. The pole-pair conversion stays with the
 * caller, as it already is in esp_foc_rotor_state_t.
 *
 * Q16.16 throughout; float appears only in the config helper.
 */

typedef enum {
    ESP_FOC_ROTOR_PLL_MECH = 0,
    ESP_FOC_ROTOR_PLL_ELEC = 1,
} esp_foc_rotor_pll_domain_t;

typedef struct {
    /* Phase gain 2·ζ·ωn [rad/s per rad]. */
    q16_t kp;
    /*
     * Frequency gain already multiplied by the loop period, ωn²·T_s
     * [rad/s per rad]. Pre-multiplied because ωn² alone leaves Q16.16 at
     * 26 Hz of bandwidth, while the product stays small for any rate a
     * sensor loop runs at.
     */
    q16_t ki_ts;
    /*
     * Loop period in Q32 seconds. Q16.16 cannot hold it: 400 µs is 26 LSB,
     * so the integration would be 1% short and θ̂ would trail forever.
     */
    uint32_t dt_q32;
    /* |ω̂| ceiling [rad/s]. 0 disables the clamp. */
    q16_t omega_max;
    /* Which field of esp_foc_rotor_state_t the update() helper reads. */
    esp_foc_rotor_pll_domain_t domain;
} esp_foc_rotor_pll_config_t;

typedef struct {
    esp_foc_rotor_pll_config_t cfg;
    q16_t theta_hat; /* wrapped to (−π, +π] */
    q16_t omega_hat;
    /* Last phase error, for a caller that wants a lock criterion. */
    q16_t err;
    /* seq of the last measurement consumed, so a repeat is not a measurement. */
    uint32_t seq_last;
    /* Health. */
    uint32_t updates;
    uint32_t coasted;
    uint32_t clamped;
    bool have_seq;
    bool inited;
} esp_foc_rotor_pll_t;

/**
 * Physical units → config. Floats are allowed here and nowhere else.
 *
 * @param step_hz        rate at which step()/update() will be called
 * @param bw_hz          loop bandwidth ωn/2π. init() refuses gains whose
 *                       per-step correction leaves the stable region; at
 *                       ζ = 1 that ceiling is near step_hz/12
 * @param zeta           damping; 1.0 is critical, ≥1 does not overshoot
 * @param omega_max_rads |ω̂| ceiling, 0 to disable
 */
void esp_foc_rotor_pll_config_default(esp_foc_rotor_pll_config_t *cfg,
                                      uint32_t step_hz,
                                      float bw_hz,
                                      float zeta,
                                      float omega_max_rads);

esp_err_t esp_foc_rotor_pll_init(esp_foc_rotor_pll_t *p,
                                 const esp_foc_rotor_pll_config_t *cfg);

/** Drop the estimate; config is kept. */
void esp_foc_rotor_pll_reset(esp_foc_rotor_pll_t *p);

/**
 * Place the estimate instead of letting the loop slew to it. Use after a
 * well-zero or an offset calibration, where the angle is known and the cost
 * of 1/bw seconds of garbage is a real torque transient.
 */
void esp_foc_rotor_pll_seed(esp_foc_rotor_pll_t *p, q16_t theta, q16_t omega);

/**
 * One loop period. ISR-safe, O(1), no division, no timebase read.
 *
 * @param theta_meas measured angle wrapped to (−π, +π]; ignored when !fresh
 * @param fresh      false dead-reckons on ω̂ alone, for a sensor slower than
 *                   the loop or a read that failed
 */
void esp_foc_rotor_pll_step(esp_foc_rotor_pll_t *p, q16_t theta_meas, bool fresh);

/**
 * Same, sourcing the measurement from the rotor sensor port. A sample counts
 * as fresh only when the snapshot is valid and seq moved, so a held latch
 * (ZOH after a failed transfer) coasts instead of being integrated as a
 * genuine zero-travel measurement.
 */
void esp_foc_rotor_pll_update(esp_foc_rotor_pll_t *p,
                              const esp_foc_rotor_sensor_t *sensor);

static inline q16_t esp_foc_rotor_pll_get_theta(const esp_foc_rotor_pll_t *p)
{
    return (p != NULL) ? p->theta_hat : 0;
}

static inline q16_t esp_foc_rotor_pll_get_omega(const esp_foc_rotor_pll_t *p)
{
    return (p != NULL) ? p->omega_hat : 0;
}

static inline q16_t esp_foc_rotor_pll_get_phase_err(const esp_foc_rotor_pll_t *p)
{
    return (p != NULL) ? p->err : 0;
}

#ifdef __cplusplus
}
#endif
