/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#include <math.h>
#include <string.h>

#include "espFoC/motor_control/esp_foc_rotor_pll.h"
#include "espFoC/utils/esp_foc_angle.h"

/*
 * Discretisation ceilings, one per path, both expressed as the size of the
 * correction a single step is allowed to make.
 *
 * Integral: (ωn·T_s)² ≤ (2π/10)², i.e. ωn·T_s ≤ 0.63, which is the point past
 * which the discrete poles leave the neighbourhood of the continuous ones and
 * the requested ζ stops describing the answer. Squared because ki_ts·T_s is
 * (ωn·T_s)² and that keeps the check free of a square root.
 *
 * Proportional: k_p·T_s ≤ 1. Above 1 the angle correction overshoots the
 * measurement it is chasing; above 2 it diverges outright.
 *
 * At ζ = 1 the proportional gate is the binding one and puts the ceiling near
 * step_hz/12.
 */
#define KI_TS_DT_MAX ((int64_t)25876) /* (2π/10)² in Q16.16 */
#define KP_DT_MAX    ((int64_t)Q16_ONE)

/**
 * ω [Q16.16 rad/s] × t [Q32 s] → angle [Q16.16 rad].
 */
static inline q16_t travel(q16_t omega, uint32_t t_q32)
{
    return (q16_t)(((int64_t)omega * (int64_t)t_q32) >> 32);
}

void esp_foc_rotor_pll_config_default(esp_foc_rotor_pll_config_t *cfg,
                                      uint32_t step_hz,
                                      float bw_hz,
                                      float zeta,
                                      float omega_max_rads)
{
    if (cfg == NULL) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));

    if (step_hz == 0u) {
        return;
    }

    const float ts = 1.0f / (float)step_hz;
    const float wn = 2.0f * (float)M_PI * bw_hz;

    cfg->kp = q16_from_float(2.0f * zeta * wn);
    cfg->ki_ts = q16_from_float(wn * wn * ts);
    cfg->dt_q32 = (uint32_t)((4294967296.0 / (double)step_hz) + 0.5);
    cfg->omega_max = q16_from_float(omega_max_rads);
    cfg->domain = ESP_FOC_ROTOR_PLL_MECH;
}

esp_err_t esp_foc_rotor_pll_init(esp_foc_rotor_pll_t *p,
                                 const esp_foc_rotor_pll_config_t *cfg)
{
    if (p == NULL || cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->kp <= 0 || cfg->ki_ts <= 0 || cfg->dt_q32 == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->omega_max < 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->domain != ESP_FOC_ROTOR_PLL_MECH &&
        cfg->domain != ESP_FOC_ROTOR_PLL_ELEC) {
        return ESP_ERR_INVALID_ARG;
    }
    /*
     * Gate the gains the caller actually passed, not the floats that may have
     * produced them, so a hand-built config is held to the same limits. The
     * bound is applied as a division so the product cannot overflow int64 for
     * an absurd ki_ts.
     */
    const int64_t dt = (int64_t)cfg->dt_q32;
    if ((int64_t)cfg->ki_ts > ((KI_TS_DT_MAX << 32) / dt)) {
        return ESP_ERR_INVALID_ARG;
    }
    if ((int64_t)cfg->kp > ((KP_DT_MAX << 32) / dt)) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(p, 0, sizeof(*p));
    p->cfg = *cfg;
    p->inited = true;
    return ESP_OK;
}

void esp_foc_rotor_pll_reset(esp_foc_rotor_pll_t *p)
{
    if (p == NULL || !p->inited) {
        return;
    }
    p->theta_hat = 0;
    p->omega_hat = 0;
    p->err = 0;
    p->seq_last = 0;
    p->updates = 0;
    p->coasted = 0;
    p->clamped = 0;
    p->have_seq = false;
}

void esp_foc_rotor_pll_seed(esp_foc_rotor_pll_t *p, q16_t theta, q16_t omega)
{
    if (p == NULL || !p->inited) {
        return;
    }
    p->theta_hat = q16_wrap_pi(theta);
    p->omega_hat = omega;
    p->err = 0;
}

void esp_foc_rotor_pll_step(esp_foc_rotor_pll_t *p, q16_t theta_meas, bool fresh)
{
    if (p == NULL || !p->inited) {
        return;
    }

    /* Load. */
    const q16_t kp = p->cfg.kp;
    const q16_t ki_ts = p->cfg.ki_ts;
    const q16_t omega_max = p->cfg.omega_max;
    const uint32_t dt_q32 = p->cfg.dt_q32;
    q16_t theta = p->theta_hat;
    q16_t omega = p->omega_hat;
    uint32_t clamped = p->clamped;

    const q16_t err = fresh ? q16_angle_delta(theta, theta_meas) : 0;

    /*
     * Semi-implicit: the angle integrates the *corrected* rate. Using the
     * pre-update ω̂ costs one sample of lag inside the loop and drops the
     * discrete damping well below the ζ that was designed for.
     */
    omega = q16_add(omega, q16_mul(ki_ts, err));
    if (omega_max > 0) {
        if (omega > omega_max) {
            omega = omega_max;
            clamped++;
        } else if (omega < q16_neg(omega_max)) {
            omega = q16_neg(omega_max);
            clamped++;
        }
    }
    theta = q16_wrap_pi(
        q16_add(theta, travel(q16_add(omega, q16_mul(kp, err)), dt_q32)));

    /* Store. */
    p->theta_hat = theta;
    p->omega_hat = omega;
    p->err = err;
    p->clamped = clamped;
    if (fresh) {
        p->updates++;
    } else {
        p->coasted++;
    }
}

void esp_foc_rotor_pll_update(esp_foc_rotor_pll_t *p,
                              const esp_foc_rotor_sensor_t *sensor)
{
    if (p == NULL || !p->inited) {
        return;
    }

    esp_foc_rotor_state_t st;
    esp_foc_rotor_sensor_snapshot(sensor, &st);

    const bool fresh = st.valid && (!p->have_seq || st.seq != p->seq_last);
    const q16_t theta = (p->cfg.domain == ESP_FOC_ROTOR_PLL_ELEC) ? st.theta_e
                                                                  : st.theta_m;

    esp_foc_rotor_pll_step(p, theta, fresh);

    if (fresh) {
        p->seq_last = st.seq;
        p->have_seq = true;
    }
}
