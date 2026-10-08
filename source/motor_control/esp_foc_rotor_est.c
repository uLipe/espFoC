/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#include <string.h>

#include "espFoC/motor_control/esp_foc_rotor_est.h"
#include "espFoC/utils/esp_foc_angle.h"

/*
 * Below this, ω̂ is snapped to zero. Geometric decay in Q16.16 truncates
 * toward −∞ for negatives, so a decaying negative would otherwise park on −1
 * LSB forever and keep is_moving()'s companion state alive. 16 LSB is
 * 2.4e-4 rad/s.
 */
#define OMEGA_EPS ((q16_t)16)

/**
 * ω [Q16.16 rad/s] × t [Q32 s] → angle [Q16.16 rad].
 *
 * The period cannot live in Q16.16: at 20 kHz it would round to 3 LSB out of
 * 3.2768 and the dead reckoning would lose 8% of every step. Q32 holds it to
 * a part in 2e6, and the product stays inside int64 for any ω the Q16.16
 * range can express.
 */
static inline q16_t travel(q16_t omega, uint32_t t_q32)
{
    return (q16_t)(((int64_t)omega * (int64_t)t_q32) >> 32);
}

static q16_t clamp_adv(esp_foc_rotor_est_t *e, q16_t adv)
{
    if (adv < e->clamp_lo) {
        e->clamped++;
        return e->clamp_lo;
    }
    if (adv > e->clamp_hi) {
        e->clamped++;
        return e->clamp_hi;
    }
    return adv;
}

/*
 * The clamp band follows the travel direction: the estimate may run to the
 * next expected measurement and no further, plus a margin at both ends that
 * covers table error and a correction that pulls backwards.
 *
 * This is a safety property, not tidiness. It bounds the angle error at one
 * sector even when ω̂ is completely wrong, which is what keeps a runaway
 * extrapolator from inverting the sign of the torque.
 */
static void set_band(esp_foc_rotor_est_t *e, int dir)
{
    q16_t span = q16_add(e->cfg.sector_span, e->cfg.clamp_margin);
    q16_t margin = e->cfg.clamp_margin;

    if (dir > 0) {
        e->clamp_lo = q16_neg(margin);
        e->clamp_hi = span;
    } else if (dir < 0) {
        e->clamp_lo = q16_neg(span);
        e->clamp_hi = margin;
    } else {
        e->clamp_lo = q16_neg(span);
        e->clamp_hi = span;
    }
}

void esp_foc_rotor_est_config_default(esp_foc_rotor_est_config_t *cfg,
                                      uint32_t tick_hz,
                                      uint32_t step_hz,
                                      float sector_span_rad,
                                      float standstill_ms)
{
    if (cfg == NULL) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));

    cfg->tick_hz = tick_hz;
    cfg->step_hz = step_hz;
    cfg->sector_span = q16_from_float(sector_span_rad);
    /* A tenth of a sector absorbs sensor placement error without letting the
     * estimate cross into the next one. */
    cfg->clamp_margin = q16_from_float(sector_span_rad * 0.1f);

    /* Monotone, no overshoot, and fast enough that a sector or two of history
     * is all the memory the loop keeps. */
    cfg->lambda_theta = q16_from_float(0.5f);
    cfg->lambda_omega = q16_from_float(0.3f);

    float periods = (standstill_ms * 1e-3f) * (float)step_hz;
    cfg->standstill_periods = (periods > 1.0f) ? (uint32_t)periods : 1u;
    /* One time constant per standstill window once the measurements stop. */
    cfg->omega_decay = q16_from_float(1.0f - 1.0f / (float)cfg->standstill_periods);

    /* Detection is polled at the hot-path rate, so two measurements an order
     * of magnitude closer than one period apart are contact bounce, not
     * travel. A gap past four standstill windows is no period at all. */
    uint32_t period_ticks = (step_hz != 0u) ? (tick_hz / step_hz) : tick_hz;
    cfg->dticks_min = period_ticks / 10u;
    if (cfg->dticks_min < 1u) {
        cfg->dticks_min = 1u;
    }
    cfg->dticks_max = (uint32_t)((standstill_ms * 1e-3f) * (float)tick_hz * 4.0f);
    if (cfg->dticks_max <= cfg->dticks_min) {
        cfg->dticks_max = cfg->dticks_min + 1u;
    }
}

esp_err_t esp_foc_rotor_est_init(esp_foc_rotor_est_t *e,
                                 const esp_foc_rotor_est_config_t *cfg)
{
    if (e == NULL || cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->tick_hz == 0u || cfg->step_hz == 0u || cfg->sector_span <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->lambda_theta <= 0 || cfg->lambda_theta >= 2 * Q16_ONE) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->lambda_omega <= 0 || cfg->lambda_omega >= 2 * Q16_ONE) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->clamp_margin < 0 || cfg->omega_decay < 0 || cfg->omega_decay > Q16_ONE) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->dticks_max <= cfg->dticks_min) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(e, 0, sizeof(*e));
    e->cfg = *cfg;
    e->dt_q32 = (uint32_t)((1ull << 32) / (uint64_t)cfg->step_hz);
    e->half_dt_q32 = e->dt_q32 / 2u;
    e->inited = true;
    esp_foc_rotor_est_reset(e);
    return ESP_OK;
}

void esp_foc_rotor_est_reset(esp_foc_rotor_est_t *e)
{
    if (e == NULL) {
        return;
    }
    e->theta_anchor = 0;
    e->adv = 0;
    e->theta_hat = 0;
    e->omega_hat = 0;
    e->t_last = 0;
    e->idle_periods = 0;
    e->edges = 0;
    e->rejected = 0;
    e->clamped = 0;
    e->stale = 0;
    e->dir = 0;
    e->have_anchor = false;
    e->have_time = false;
    set_band(e, 0);
}

void esp_foc_rotor_est_step(esp_foc_rotor_est_t *e)
{
    if (e == NULL || !e->inited) {
        return;
    }

    /* Load. */
    q16_t omega = e->omega_hat;
    q16_t adv = e->adv;
    q16_t anchor = e->theta_anchor;
    uint32_t idle = e->idle_periods;
    const uint32_t dt_q32 = e->dt_q32;
    const uint32_t timeout = e->cfg.standstill_periods;
    const q16_t decay = e->cfg.omega_decay;

    if (idle < UINT32_MAX) {
        idle++;
    }

    if (idle > timeout) {
        omega = q16_mul(omega, decay);
        if (omega <= OMEGA_EPS && omega >= q16_neg(OMEGA_EPS)) {
            omega = 0;
        }
    }

    adv = clamp_adv(e, q16_add(adv, travel(omega, dt_q32)));

    /* Store. */
    e->omega_hat = omega;
    e->adv = adv;
    e->idle_periods = idle;
    e->theta_hat = q16_wrap_pi(q16_add(anchor, adv));
}

void esp_foc_rotor_est_on_edge(esp_foc_rotor_est_t *e,
                               q16_t theta_meas,
                               uint64_t ticks,
                               int dir)
{
    if (e == NULL || !e->inited) {
        return;
    }

    /* Load. */
    const q16_t omega_prev = e->omega_hat;
    q16_t omega = omega_prev;
    const q16_t theta_hat = e->theta_hat;
    const q16_t anchor = e->theta_anchor;
    const bool have_anchor = e->have_anchor;
    const bool have_time = e->have_time;
    const uint64_t t_last = e->t_last;

    theta_meas = q16_wrap_pi(theta_meas);

    if (have_time) {
        /*
         * 54-bit hardware counter at 1 MHz wraps in centuries, so the
         * subtraction needs no wrap case.
         */
        uint64_t dticks = ticks - t_last;

        if (dticks < (uint64_t)e->cfg.dticks_min) {
            /* Bounce. Moving any state on it would corrupt both the period
             * and the anchor, so the measurement is dropped whole. */
            e->rejected++;
            return;
        }

        if (dticks <= (uint64_t)e->cfg.dticks_max) {
            /*
             * Δθ comes from the caller's measurement and not from a nominal
             * sector constant, so uneven sensor placement shows up as the true
             * average speed over that sector instead of ripple at the edge
             * rate.
             *
             * Δt is deliberately not converted to Q16.16 seconds first: 625 µs
             * would be 41 LSB and the quantization would swamp the result.
             */
            q16_t dtheta = q16_angle_delta(anchor, theta_meas);
            if (have_anchor && dtheta != 0) {
                int64_t num = (int64_t)dtheta * (int64_t)e->cfg.tick_hz;
                int64_t w = num / (int64_t)dticks;
                if (w > INT32_MAX) {
                    w = INT32_MAX;
                } else if (w < INT32_MIN) {
                    w = INT32_MIN;
                }
                q16_t omega_meas = (q16_t)w;
                omega = q16_add(omega, q16_mul(e->cfg.lambda_omega,
                                               q16_sub(omega_meas, omega)));
            }
        } else {
            e->stale++;
        }
    }

    /*
     * Phase error against the prediction, with the measurement back-dated by
     * half a hot-path period.
     *
     * The sign follows from the call order, not from intuition. This runs
     * before step(), which credits a *whole* period of travel — but the edge
     * happened somewhere inside the period that just elapsed, so only part of
     * that travel lies ahead of the measurement. Subtracting the mean of the
     * elapsed part (dt/2) leaves a zero-mean residual; getting it backwards
     * biases the estimate by a full ω·dt, which reads as a constant lead.
     *
     * The old ω̂ is used on purpose — letting the fresh one in here would put
     * a term above the diagonal and cost the triangular error map the
     * stability argument rests on.
     */
    q16_t theta_corr;
    if (have_anchor) {
        q16_t comp = travel(omega_prev, e->half_dt_q32);
        q16_t err = q16_angle_delta(theta_hat, q16_sub(theta_meas, comp));
        theta_corr = q16_wrap_pi(q16_add(theta_hat,
                                         q16_mul(e->cfg.lambda_theta, err)));
    } else {
        /* First measurement: adopt it, there is nothing to correct against. */
        theta_corr = theta_meas;
    }

    if (dir != 0) {
        e->dir = (dir > 0) ? 1 : -1;
    }
    set_band(e, e->dir);

    /* Store: re-anchor on the measurement and carry the correction residual. */
    e->omega_hat = omega;
    e->theta_anchor = theta_meas;
    e->adv = clamp_adv(e, q16_angle_delta(theta_meas, theta_corr));
    e->theta_hat = q16_wrap_pi(q16_add(e->theta_anchor, e->adv));
    e->t_last = ticks;
    e->have_time = true;
    e->have_anchor = true;
    e->idle_periods = 0;
    e->edges++;
}
