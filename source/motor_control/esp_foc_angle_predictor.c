/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include "esp_attr.h"
#include "espFoC/utils/angle_predictor_q16.h"
#include "espFoC/utils/foc_math_q16.h"

/* Q16 representation of pi. Q16_TWO_PI is exposed in esp_foc_q16.h
 * (411775); pi sits at half that. Defined locally so the predictor
 * does not pollute the public q16 header with a single-purpose const. */
#define Q16_PI ((q16_t)(Q16_TWO_PI / 2))

/* Wrap a signed angle delta to (-pi, +pi]. Used on the residual so
 * the 0 / 2*pi seam never produces a 2*pi-sized error. */
static inline q16_t wrap_pi_q16(q16_t a)
{
    while (a > Q16_PI) {
        a = (q16_t)((int32_t)a - (int32_t)Q16_TWO_PI);
    }
    while (a <= -Q16_PI) {
        a = (q16_t)((int32_t)a + (int32_t)Q16_TWO_PI);
    }
    return a;
}

/* Convert a microsecond delta into Q16 seconds. Caps at INT32_MAX
 * which corresponds to ~32k seconds — the predictor is always going
 * to be called way more often than that. */
static inline q16_t dt_q16_from_us(uint64_t us)
{
    int64_t raw = ((int64_t)us * (int64_t)Q16_ONE + 500000LL) / 1000000LL;
    if (raw > (int64_t)INT32_MAX) {
        return (q16_t)INT32_MAX;
    }
    return (q16_t)raw;
}

void esp_foc_angle_predictor_init_q16(esp_foc_angle_predictor_q16_t *p,
                                      float alpha, float beta,
                                      uint64_t t_now_us)
{
    if (p == NULL) {
        return;
    }
    if (alpha < 0.0f) {
        alpha = 0.0f;
    } else if (alpha > 1.0f) {
        alpha = 1.0f;
    }
    if (beta < 0.0f) {
        beta = 0.0f;
    } else if (beta > 1.0f) {
        beta = 1.0f;
    }
    p->alpha = q16_from_float(alpha);
    p->beta  = q16_from_float(beta);
    p->theta_est = 0;
    p->omega_est = 0;
    p->t_last_us = t_now_us;
    p->initialised = false;
}

void esp_foc_angle_predictor_set_gains_q16(esp_foc_angle_predictor_q16_t *p,
                                           float alpha, float beta)
{
    if (p == NULL) {
        return;
    }
    if (alpha < 0.0f) {
        alpha = 0.0f;
    } else if (alpha > 1.0f) {
        alpha = 1.0f;
    }
    if (beta < 0.0f) {
        beta = 0.0f;
    } else if (beta > 1.0f) {
        beta = 1.0f;
    }
    p->alpha = q16_from_float(alpha);
    p->beta  = q16_from_float(beta);
}

void esp_foc_angle_predictor_update_q16(esp_foc_angle_predictor_q16_t *p,
                                        q16_t theta_meas,
                                        uint64_t t_meas_us)
{
    if (p == NULL) {
        return;
    }

    /* Make sure theta_meas is normalised so the residual wrap below
     * is honest about its [0, 2*pi) input. Cheap if it already is. */
    theta_meas = q16_normalize_angle_rad(theta_meas);

    if (!p->initialised) {
        /* First measurement seeds the state: jump theta_est to the
         * observed angle and leave omega_est at zero. The next call
         * gets a real dt and starts tracking. */
        p->theta_est = theta_meas;
        p->omega_est = 0;
        p->t_last_us = t_meas_us;
        p->initialised = true;
        return;
    }

    uint64_t dt_us = (t_meas_us > p->t_last_us)
                       ? (t_meas_us - p->t_last_us)
                       : 0u;
    if (dt_us == 0u) {
        /* No time advanced — nothing meaningful to integrate. Update
         * theta_est anyway so a paused-then-resumed encoder does not
         * lose alignment. */
        p->theta_est = theta_meas;
        p->t_last_us = t_meas_us;
        return;
    }

    q16_t dt = dt_q16_from_us(dt_us);
    /* theta_pred = wrap(theta_est + omega_est * dt) */
    q16_t theta_pred = q16_normalize_angle_rad(
        q16_add(p->theta_est, q16_mul(p->omega_est, dt)));
    q16_t err = wrap_pi_q16(q16_sub(theta_meas, theta_pred));

    p->theta_est = q16_normalize_angle_rad(
        q16_add(theta_pred, q16_mul(p->alpha, err)));

    /* omega_est += (beta / dt) * err. Done as (beta * err) / dt to
     * stay inside Q16's int32 range; intermediate uses int64. */
    if (dt > 0) {
        int64_t num = (int64_t)p->beta * (int64_t)err; /* Q32 */
        /* (Q32 / Q16) = Q16 with a shift */
        int64_t step_q16 = (num / (int64_t)dt);
        int64_t new_omega = (int64_t)p->omega_est + step_q16;
        if (new_omega > (int64_t)INT32_MAX) {
            new_omega = INT32_MAX;
        } else if (new_omega < (int64_t)INT32_MIN) {
            new_omega = INT32_MIN;
        }
        p->omega_est = (q16_t)new_omega;
    }

    p->t_last_us = t_meas_us;
}

IRAM_ATTR q16_t esp_foc_angle_predictor_predict_q16(
    const esp_foc_angle_predictor_q16_t *p,
    uint64_t t_now_us)
{
    if (p == NULL || !p->initialised) {
        return 0;
    }
    uint64_t dt_us = (t_now_us > p->t_last_us)
                       ? (t_now_us - p->t_last_us)
                       : 0u;
    q16_t dt = dt_q16_from_us(dt_us);
    return q16_normalize_angle_rad(
        q16_add(p->theta_est, q16_mul(p->omega_est, dt)));
}
