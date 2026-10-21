/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Speed / position laws of the sensored stack: cascade PI, LQR and LQR+I.
 */
#include "espFoC/motor_control/esp_foc_motion_ctrl.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "espFoC/utils/esp_foc_lqr.h"
#include "espFoC/utils/esp_foc_pid.h"

#define MC_TWO_PI    6.28318530718
#define MC_Q32       4294967296.0
#define MC_DESCENT   0.8
#define MC_DESCENT_N 24

static inline q16_t mc_sat64(int64_t x)
{
    if (x > (int64_t)INT32_MAX) {
        return INT32_MAX;
    }
    if (x < -(int64_t)INT32_MAX) {
        return -INT32_MAX;
    }
    return (q16_t)x;
}

/* ------------------------------------------------------------------------ */
/* Cascade PI                                                                */
/* ------------------------------------------------------------------------ */

esp_err_t esp_foc_motion_pi_set_speed_gains(esp_foc_motion_pi_t *pi, float kp, float ki)
{
    if ((pi == NULL) || !(pi->fs_hz > 0.0f) || !(kp >= 0.0f) || !(ki >= 0.0f)) {
        return ESP_ERR_INVALID_ARG;
    }
    pi->kp_w_q32 = (int64_t)((double)kp * MC_Q32 + 0.5);
    pi->ki_ts_q32 = (int64_t)((double)ki / (double)pi->fs_hz * MC_Q32 + 0.5);
    pi->iface.tune.kp_w = kp;
    pi->iface.tune.ki_w = ki;
    return ESP_OK;
}

esp_err_t esp_foc_motion_pi_set_position_kp(esp_foc_motion_pi_t *pi, float kp)
{
    if ((pi == NULL) || !(kp >= 0.0f)) {
        return ESP_ERR_INVALID_ARG;
    }
    pi->kp_pos = q16_from_float(kp);
    pi->iface.tune.kp_pos = kp;
    return ESP_OK;
}

static esp_err_t pi_design(esp_foc_motion_ctrl_t *self, const esp_foc_motion_ctrl_design_t *d)
{
    esp_foc_motion_pi_t *pi = (esp_foc_motion_pi_t *)self;
    if (!(d->fs_hz > 0.0f) || (d->pole_pairs == 0u) || !(d->i_max_a > 0.0f) ||
        !(d->pos_sep > 0.0f) || !(d->corr_max_rads > 0.0f) || !(d->wm_max_rads > 0.0f)) {
        return ESP_ERR_INVALID_ARG;
    }
    float kp = d->kp_w;
    float ki = d->ki_w;
    if ((kp == 0.0f) && (ki == 0.0f)) {
        const esp_err_t err =
            esp_foc_pid_design_integrator(d->k_rads2_a, d->fs_hz, d->bw_hz, d->zeta, &kp, &ki);
        if (err != ESP_OK) {
            return err;
        }
    }
    pi->fs_hz = d->fs_hz;
    (void)esp_foc_motion_pi_set_speed_gains(pi, kp, ki);
    pi->i_max = q16_from_float(d->i_max_a);
    pi->wi_lim_q32 = (int64_t)pi->i_max << 16;
    pi->corr_max = q16_from_float(d->corr_max_rads);
    pi->wm_max = q16_from_float(d->wm_max_rads);
    pi->pp = (int32_t)d->pole_pairs;
    const float fc = 2.0f * d->zeta * d->bw_hz;
    (void)esp_foc_motion_pi_set_position_kp(
        pi, (d->kp_pos > 0.0f) ? d->kp_pos : (float)MC_TWO_PI * fc / d->pos_sep);
    return ESP_OK;
}

static void pi_reset(esp_foc_motion_ctrl_t *self, esp_foc_motion_mode_t mode, q16_t held)
{
    (void)mode;
    esp_foc_motion_pi_t *pi = (esp_foc_motion_pi_t *)self;
    pi->wi_q32 = (int64_t)held << 16;
}

/* Integral gain in Q32: Ki·Ts of a speed loop is under one Q16 LSB. */
static q16_t pi_update(esp_foc_motion_ctrl_t *self, esp_foc_motion_mode_t mode,
                       const esp_foc_motion_in_t *in, q16_t *w_ref_out)
{
    esp_foc_motion_pi_t *pi = (esp_foc_motion_pi_t *)self;
    const int64_t kp = pi->kp_w_q32;
    const int64_t ki = pi->ki_ts_q32;
    const int64_t i_old = pi->wi_q32;
    const int64_t i_lim = pi->wi_lim_q32;
    const int64_t lim = (int64_t)pi->i_max;
    const q16_t ff = in->iq_ff;
    const q16_t w = in->w;

    q16_t w_ref;
    if (mode == ESP_FOC_MOTION_POSITION) {
        const q16_t cmax = pi->corr_max;
        const q16_t wmax = pi->wm_max;
        const q16_t corr =
            q16_clamp(q16_mul(pi->kp_pos, mc_sat64(in->e_th)), q16_neg(cmax), cmax);
        const q16_t wm = q16_clamp(q16_add(in->w_ff_m, corr), q16_neg(wmax), wmax);
        w_ref = (q16_t)(wm * pi->pp);
    } else {
        w_ref = in->w_ref_e;
    }
    const q16_t e = q16_sub(w_ref, w);

    const int64_t inc = (ki * (int64_t)e) >> 16;
    int64_t i_new = i_old + inc;
    if (i_new > i_lim) {
        i_new = i_lim;
    } else if (i_new < -i_lim) {
        i_new = -i_lim;
    }
    int64_t u = ((kp * (int64_t)e) >> 32) + (i_new >> 16) + (int64_t)ff;
    bool hold = false;
    if (u > lim) {
        u = lim;
        hold = (inc > 0);
    } else if (u < -lim) {
        u = -lim;
        hold = (inc < 0);
    }
    if (!hold) {
        pi->wi_q32 = i_new;
    }
    *w_ref_out = w_ref;
    return (q16_t)u;
}

esp_foc_motion_ctrl_t *esp_foc_motion_pi_bind(esp_foc_motion_pi_t *pi)
{
    if (pi == NULL) {
        return NULL;
    }
    memset(pi, 0, sizeof(*pi));
    pi->iface.design = pi_design;
    pi->iface.reset = pi_reset;
    pi->iface.update = pi_update;
    return &pi->iface;
}

/* ------------------------------------------------------------------------ */
/* LQR / LQR+I                                                               */
/* ------------------------------------------------------------------------ */

/*
 * Bryson weights for natural frequency wn: the position error that i_max
 * corrects in about 1/wn^2, the speed error it corrects in about 1/wn, the
 * integral over ti. The current and delay states carry no weight.
 */
static esp_err_t lqr_design_mode(bool integral, const esp_foc_motion_ctrl_design_t *d,
                                 bool position, double wn, double *k, uint8_t *n)
{
    const esp_foc_lqr_servo_t p = {
        .k_rads2_a = d->k_rads2_a,
        .pole_pairs = d->pole_pairs,
        .ts_s = 1.0 / (double)d->fs_hz,
        .tau_i_s = 1.0 / (MC_TWO_PI * (double)d->i_bw_hz),
        .position = position,
        .integral = integral,
    };
    esp_foc_lqr_model_t m;
    esp_err_t err = esp_foc_lqr_servo_model(&p, &m);
    if (err != ESP_OK) {
        return err;
    }
    const double imax = d->i_max_a;
    const double kk = d->k_rads2_a;
    const double th_max = (d->theta_max_rad > 0.0f) ? (double)d->theta_max_rad
                                                    : kk / (double)d->pole_pairs * imax / (wn * wn);
    const double w_max = (d->w_max_rads > 0.0f) ? (double)d->w_max_rads : kk * imax / wn;
    const double ti = (d->ti_s > 0.0f) ? (double)d->ti_s : 3.0 / wn;
    double q[ESP_FOC_LQR_N_MAX] = { 0 };
    const uint8_t iw = position ? 1u : 0u;
    if (position) {
        q[0] = 1.0 / (th_max * th_max);
    }
    q[iw] = 1.0 / (w_max * w_max);
    if (integral) {
        const double zmax = (position ? th_max : w_max) * ti;
        q[iw + 3u] = 1.0 / (zmax * zmax);
    }
    err = esp_foc_lqr_dare(&m, q, 1.0 / (imax * imax), k);
    *n = m.n;
    return err;
}

static int64_t lqr_q32(double k)
{
    return (int64_t)llround(k * MC_Q32);
}

static void lqr_take(esp_foc_motion_lqr_t *l, esp_foc_motion_mode_t mode, bool integral,
                     const double *k, double imax)
{
    esp_foc_motion_lqr_gains_t *g = &l->g[mode];
    const uint8_t iw = (mode == ESP_FOC_MOTION_POSITION) ? 1u : 0u;
    memset(g, 0, sizeof(*g));
    if (iw != 0u) {
        g->kth = lqr_q32(k[0]);
        const double e = (k[0] != 0.0) ? fmin(4.0 * imax / fabs(k[0]), 32000.0) : 32000.0;
        g->e_lim = (q16_t)(e * 65536.0);
    }
    g->kw = lqr_q32(k[iw]);
    g->ki = lqr_q32(k[iw + 1u]);
    g->ku = lqr_q32(k[iw + 2u]);
    if (integral) {
        g->kz = (int64_t)llround(k[iw + 3u] * 65536.0);
        if (g->kz != 0) {
            const int64_t lim = (int64_t)l->i_max << 32;
            g->z_lim = lim / ((g->kz < 0) ? -g->kz : g->kz);
        }
    }
}

static esp_err_t lqr_design(esp_foc_motion_ctrl_t *self, const esp_foc_motion_ctrl_design_t *d)
{
    esp_foc_motion_lqr_t *l = (esp_foc_motion_lqr_t *)self;
    if (!(d->fs_hz > 0.0f) || (d->pole_pairs == 0u) || !(d->i_max_a > 0.0f) ||
        !(d->k_rads2_a > 0.0f) || !(d->i_bw_hz > 0.0f) || !(d->wn_hz > 0.0f) ||
        !(d->wn_hz < 0.5f * d->fs_hz) || !(d->ripple_rads >= 0.0f) ||
        ((d->ripple_rads > 0.0f) && !(d->fb_iq_budget_a > 0.0f))) {
        return ESP_ERR_INVALID_ARG;
    }
    const bool descend = (d->ripple_rads > 0.0f) && !(d->w_max_rads > 0.0f);
    double wn = MC_TWO_PI * (double)d->wn_hz;
    double kv[ESP_FOC_LQR_N_MAX] = { 0 };
    double kp[ESP_FOC_LQR_N_MAX] = { 0 };
    uint8_t nv = 0;
    uint8_t np = 0;
    for (int it = 0;; it++) {
        esp_err_t err = lqr_design_mode(true, d, false, wn, kv, &nv);
        if ((err == ESP_OK) && d->position) {
            err = lqr_design_mode(l->integral, d, true, wn, kp, &np);
        }
        if (err != ESP_OK) {
            return err;
        }
        double kw = fabs(kv[0]);
        if (d->position) {
            kw = fmax(kw, fabs(kp[1]));
        }
        if (!descend || (kw * (double)d->ripple_rads <= (double)d->fb_iq_budget_a) ||
            (it >= MC_DESCENT_N)) {
            break;
        }
        wn *= MC_DESCENT;
    }

    const double ts = 1.0 / (double)d->fs_hz;
    l->i_max = q16_from_float(d->i_max_a);
    l->pp = (int32_t)d->pole_pairs;
    l->ts_q32 = lqr_q32(ts);
    l->alpha_q32 = lqr_q32(exp(-ts * MC_TWO_PI * (double)d->i_bw_hz));
    lqr_take(l, ESP_FOC_MOTION_VELOCITY, true, kv, d->i_max_a);
    if (d->position) {
        lqr_take(l, ESP_FOC_MOTION_POSITION, l->integral, kp, d->i_max_a);
    } else {
        memset(&l->g[ESP_FOC_MOTION_POSITION], 0, sizeof(l->g[0]));
    }

    esp_foc_motion_ctrl_tune_t *t = &self->tune;
    memset(t, 0, sizeof(*t));
    t->wn_hz = (float)(wn / MC_TWO_PI);
    t->n_vel = nv;
    for (uint8_t i = 0; i < nv; i++) {
        t->k_vel[i] = (float)kv[i];
    }
    if (d->position) {
        t->n_pos = np;
        for (uint8_t i = 0; i < np; i++) {
            t->k_pos[i] = (float)kp[i];
        }
    }
    return ESP_OK;
}

/*
 * Held is the iq the loop already commands beyond its feedforward: the lag
 * and delay states take it as settled, and with the integral the integral
 * state supplies it, so the first output is held + iq_ff.
 */
static void lqr_reset(esp_foc_motion_ctrl_t *self, esp_foc_motion_mode_t mode, q16_t held)
{
    esp_foc_motion_lqr_t *l = (esp_foc_motion_lqr_t *)self;
    const esp_foc_motion_lqr_gains_t *g = &l->g[mode];
    l->di = held;
    l->dup = held;
    l->z = 0;
    if (g->kz != 0) {
        const int64_t need = ((int64_t)1 << 32) + g->ki + g->ku;
        int64_t z = -(need * (int64_t)held) / g->kz;
        if (z > g->z_lim) {
            z = g->z_lim;
        } else if (z < -g->z_lim) {
            z = -g->z_lim;
        }
        l->z = z;
    }
}

/*
 * u = ff - k·x in Q48 (Q32 gains on Q16 states; Q16 kz on the Q32 integral).
 * The lag and delay states see only δu = u_clamped - ff: the feedforward
 * moves the reference and the plant together and is not a deviation.
 */
static q16_t lqr_update(esp_foc_motion_ctrl_t *self, esp_foc_motion_mode_t mode,
                        const esp_foc_motion_in_t *in, q16_t *w_ref_out)
{
    esp_foc_motion_lqr_t *l = (esp_foc_motion_lqr_t *)self;
    const esp_foc_motion_lqr_gains_t *g = &l->g[mode];
    const int64_t kth = g->kth;
    const int64_t kw = g->kw;
    const int64_t ki = g->ki;
    const int64_t ku = g->ku;
    const int64_t kz = g->kz;
    const int64_t z_lim = g->z_lim;
    const q16_t e_lim = g->e_lim;
    const int64_t alpha = l->alpha_q32;
    const int64_t ts = l->ts_q32;
    const int64_t z = l->z;
    const q16_t di = l->di;
    const q16_t dup = l->dup;
    const int64_t lim = (int64_t)l->i_max;
    const int32_t pp = l->pp;
    const bool integral = (kz != 0);
    const q16_t ff = in->iq_ff;

    q16_t ref;
    q16_t xi;
    int64_t acc;
    if (mode == ESP_FOC_MOTION_POSITION) {
        ref = (q16_t)(in->w_ff_m * pp);
        int64_t eth = -in->e_th;
        if (eth > (int64_t)e_lim) {
            eth = e_lim;
        } else if (eth < -(int64_t)e_lim) {
            eth = -(int64_t)e_lim;
        }
        const q16_t xw = q16_sub(in->w, ref);
        acc = kth * eth + kw * (int64_t)xw;
        xi = (q16_t)eth;
    } else {
        ref = in->w_ref_e;
        const q16_t xw = q16_sub(in->w, ref);
        acc = kw * (int64_t)xw;
        xi = xw;
    }
    acc += ki * (int64_t)di + ku * (int64_t)dup + kz * z;

    int64_t u = (int64_t)ff - (acc >> 32);
    int sat = 0;
    if (u > lim) {
        u = lim;
        sat = 1;
    } else if (u < -lim) {
        u = -lim;
        sat = -1;
    }

    int64_t z1 = z;
    if (integral) {
        const int64_t dz = ((int64_t)xi * ts) >> 16;
        /* The output moves by -kz·dz: hold when that is into the clamp. */
        const bool up = ((kz > 0) && (dz < 0)) || ((kz < 0) && (dz > 0));
        const bool down = ((kz > 0) && (dz > 0)) || ((kz < 0) && (dz < 0));
        if (!((sat > 0) && up) && !((sat < 0) && down)) {
            z1 = z + dz;
            if (z1 > z_lim) {
                z1 = z_lim;
            } else if (z1 < -z_lim) {
                z1 = -z_lim;
            }
        }
    }
    const q16_t du = (q16_t)(u - (int64_t)ff);
    const q16_t di1 =
        (q16_t)((int64_t)di + ((((int64_t)dup - (int64_t)di) * (((int64_t)1 << 32) - alpha)) >> 32));

    l->z = z1;
    l->di = di1;
    l->dup = du;
    *w_ref_out = ref;
    return (q16_t)u;
}

esp_foc_motion_ctrl_t *esp_foc_motion_lqr_bind(esp_foc_motion_lqr_t *lqr, bool integral)
{
    if (lqr == NULL) {
        return NULL;
    }
    memset(lqr, 0, sizeof(*lqr));
    lqr->integral = integral;
    lqr->iface.design = lqr_design;
    lqr->iface.reset = lqr_reset;
    lqr->iface.update = lqr_update;
    return &lqr->iface;
}
