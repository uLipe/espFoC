/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Voltage-model stator-flux observer in αβ (Q16.16) plus a tracking PLL on ψ̂_r.
 *
 *   dψ̂_s/dt = v − Rs i − λ (ψ̂_s − Ls i)
 *   ψ̂_r     = ψ̂_s − Ls i
 *   ê       = ω̂ × ψ̂_r     (eα = −ω ψβ, eβ = ω ψα)
 *
 * λ is a washout (not a Gopinath restore toward ψf∠θ̂). Restoring toward
 * the PLL angle makes ψ̂_r copy Park once the inverter uses θ̂, and the
 * loop runs away. Washout kills integrator DC; the rotating magnet flux
 * still passes above blend_hz.
 */
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_check.h"
#include "espFoC/motor_control/esp_foc_observer_flux.h"
#include "espFoC/utils/esp_foc_angle.h"
#include "espFoC/utils/esp_foc_pid.h"
#include "espFoC/utils/esp_foc_trig.h"

/* ψ is stored in mWb so Q16.16 has LSBs at this magnet flux (~2.5 mWb). */
#define FLUX_MWB ((q16_t)65536000)

/* alpha-max-plus-beta-min: |v| ≈ max + 0.4142·min, within ~4% of L2. */
#define FLUX_BETA_MIN ((q16_t)27146)

static esp_foc_observer_flux_t *flux_of(esp_foc_observer_t *self)
{
    return (esp_foc_observer_flux_t *)((char *)self -
                                       offsetof(esp_foc_observer_flux_t, iface));
}

static const esp_foc_observer_flux_t *flux_of_c(const esp_foc_observer_t *self)
{
    return (const esp_foc_observer_flux_t *)((const char *)self -
                                             offsetof(esp_foc_observer_flux_t, iface));
}

static q16_t lpf1(q16_t *w, q16_t x, q16_t b0)
{
    q16_t y = *w;
    y = q16_add(y, q16_mul(b0, q16_sub(x, y)));
    *w = y;
    return y;
}

static q16_t flux_dpsi_mwb(q16_t v, q16_t inv_ts)
{
    if (inv_ts == 0) {
        return 0;
    }
    int64_t r = (((int64_t)v * 1000) << 16) / (int64_t)inv_ts;
    if (r > INT32_MAX) {
        return (q16_t)INT32_MAX;
    }
    if (r < INT32_MIN) {
        return (q16_t)INT32_MIN;
    }
    return (q16_t)r;
}

static void flux_seed_psi(esp_foc_observer_flux_t *o, q16_t i_a, q16_t i_b)
{
    q16_t s;
    q16_t c;
    esp_foc_sincos(o->theta, &s, &c);
    q16_t psi_m_a = q16_mul(o->psi_f_mwb, c);
    q16_t psi_m_b = q16_mul(o->psi_f_mwb, s);
    o->psi_s_a = q16_add(q16_mul(o->ls_mh, i_a), psi_m_a);
    o->psi_s_b = q16_add(q16_mul(o->ls_mh, i_b), psi_m_b);
    o->psi_r_a = psi_m_a;
    o->psi_r_b = psi_m_b;
    o->psi_f_a = psi_m_a;
    o->psi_f_b = psi_m_b;
    o->i_a = i_a;
    o->i_b = i_b;
    o->theta_psi = o->theta;
    o->have_theta_psi = true;
    o->have_psi = true;
    o->pll_settle_left = o->pll_settle_need;
    o->phase_err = 0;
}

static void flux_reset(esp_foc_observer_t *self)
{
    if (self == NULL) {
        return;
    }
    esp_foc_observer_flux_t *o = flux_of(self);
    o->psi_s_a = 0;
    o->psi_s_b = 0;
    o->psi_r_a = 0;
    o->psi_r_b = 0;
    o->psi_f_a = 0;
    o->psi_f_b = 0;
    o->e_a = 0;
    o->e_b = 0;
    o->theta = 0;
    o->omega = 0;
    esp_foc_pid_reset(&o->pll_pi);
    esp_foc_pid_set_applied(&o->pll_pi, 0);
    o->i_a = 0;
    o->i_b = 0;
    o->theta_psi = 0;
    o->w_atan_f = 0;
    o->lock_acc = 0;
    o->unlock_acc = 0;
    o->locked = false;
    o->have_psi = false;
    o->have_theta_psi = false;
    o->pll_settle_left = 0;
    o->phase_err = 0;
    o->pll_enable = true;
    o->psi_clamp_count = 0u;
    o->psi_peak = 0;
}

static void flux_set_theta(esp_foc_observer_t *self, q16_t theta_e)
{
    if (self == NULL) {
        return;
    }
    esp_foc_observer_flux_t *o = flux_of(self);
    o->theta = q16_wrap_pi(theta_e);
    o->theta_psi = o->theta;
    o->have_theta_psi = true;
    /* Do not reseed ψ̂: I-f snap sets θ̂:=θ_ol so Park is continuous, but
     * ψ̂_r must keep pointing at the magnet. Planting ψf∠θ_ol puts the
     * estimate in the Park frame and the PLL then tracks itself. */
}

static void flux_set_omega(esp_foc_observer_t *self, q16_t omega_e)
{
    if (self == NULL) {
        return;
    }
    esp_foc_observer_flux_t *o = flux_of(self);
    o->omega = omega_e;
    esp_foc_pid_set_applied(&o->pll_pi, omega_e);
    o->w_atan_f = omega_e;
}

static void flux_set_extract(esp_foc_observer_t *self, esp_foc_angle_extract_t extract)
{
    if (self == NULL) {
        return;
    }
    flux_of(self)->extract = extract;
}

static void flux_set_pll_enable(esp_foc_observer_t *self, bool enable)
{
    if (self == NULL) {
        return;
    }
    flux_of(self)->pll_enable = enable;
}

static void flux_update(esp_foc_observer_t *self,
                        q16_t i_alpha,
                        q16_t i_beta,
                        q16_t v_alpha,
                        q16_t v_beta)
{
    if (self == NULL) {
        return;
    }
    esp_foc_observer_flux_t *o = flux_of(self);

    const q16_t rs = o->rs;
    const q16_t ls = o->ls_mh;
    const q16_t inv_ts = o->inv_ts;

    if (!o->have_psi) {
        flux_seed_psi(o, i_alpha, i_beta);
        return;
    }

    q16_t va_drop = q16_sub(v_alpha, q16_mul(rs, i_alpha));
    q16_t vb_drop = q16_sub(v_beta, q16_mul(rs, i_beta));
    q16_t psi_r_a = q16_sub(o->psi_s_a, q16_mul(ls, i_alpha));
    q16_t psi_r_b = q16_sub(o->psi_s_b, q16_mul(ls, i_beta));

    q16_t dpsi_a = flux_dpsi_mwb(va_drop, inv_ts);
    q16_t dpsi_b = flux_dpsi_mwb(vb_drop, inv_ts);
    o->psi_s_a = q16_sub(q16_add(o->psi_s_a, dpsi_a),
                         q16_mul(o->lambda_ts, psi_r_a));
    o->psi_s_b = q16_sub(q16_add(o->psi_s_b, dpsi_b),
                         q16_mul(o->lambda_ts, psi_r_b));

    q16_t psi_r_raw_a = q16_sub(o->psi_s_a, q16_mul(ls, i_alpha));
    q16_t psi_r_raw_b = q16_sub(o->psi_s_b, q16_mul(ls, i_beta));

    /* Rescale rather than saturate per axis: the direction is what the PLL
     * consumes and it is still the observer's best guess, so only the
     * magnitude — which the magnet does not grant it — is taken away. The
     * integrator state is corrected too, otherwise ψ̂_s keeps climbing and the
     * ceiling only hides it. */
    {
        const q16_t ra = (psi_r_raw_a < 0) ? q16_neg(psi_r_raw_a) : psi_r_raw_a;
        const q16_t rb = (psi_r_raw_b < 0) ? q16_neg(psi_r_raw_b) : psi_r_raw_b;
        const q16_t hi = (ra > rb) ? ra : rb;
        const q16_t lo = (ra > rb) ? rb : ra;
        const q16_t mag = q16_add(hi, q16_mul(FLUX_BETA_MIN, lo));
        if (mag > o->psi_peak) {
            o->psi_peak = mag;
        }
        if ((o->psi_max_mag > 0) && (mag > o->psi_max_mag)) {
            const q16_t k = q16_div(o->psi_max_mag, mag);
            psi_r_raw_a = q16_mul(psi_r_raw_a, k);
            psi_r_raw_b = q16_mul(psi_r_raw_b, k);
            o->psi_s_a = q16_add(psi_r_raw_a, q16_mul(ls, i_alpha));
            o->psi_s_b = q16_add(psi_r_raw_b, q16_mul(ls, i_beta));
            o->psi_clamp_count++;
        }
    }

    o->psi_r_a = lpf1(&o->psi_f_a, psi_r_raw_a, o->lpf_b0);
    o->psi_r_b = lpf1(&o->psi_f_b, psi_r_raw_b, o->lpf_b0);

    if (o->extract == ESP_FOC_ANGLE_ATAN2) {
        q16_t th_psi = esp_foc_atan2(o->psi_r_b, o->psi_r_a);
        if (!o->have_theta_psi) {
            o->theta_psi = th_psi;
            o->have_theta_psi = true;
        }
        q16_t dth_psi = q16_angle_delta(o->theta_psi, th_psi);
        o->theta_psi = th_psi;
        q16_t w_psi = q16_mul(dth_psi, inv_ts);
        w_psi = q16_clamp(w_psi, q16_neg(o->w_max), o->w_max);
        o->w_atan_f = lpf1(&o->w_atan_f, w_psi, o->w_lpf_b0);
        o->theta = th_psi;
        o->omega = o->w_atan_f;
        o->phase_err = 0;
    } else {
        q16_t s;
        q16_t c;
        esp_foc_sincos(o->theta, &s, &c);

        q16_t pa_abs = (o->psi_r_a < 0) ? q16_neg(o->psi_r_a) : o->psi_r_a;
        q16_t pb_abs = (o->psi_r_b < 0) ? q16_neg(o->psi_r_b) : o->psi_r_b;
        q16_t psi_scale = q16_add(pa_abs, pb_abs);
        if (psi_scale < o->psi_lock_min) {
            psi_scale = o->psi_lock_min;
        }
        q16_t phase_err = q16_sub(q16_mul(o->psi_r_b, c), q16_mul(o->psi_r_a, s));
        phase_err = q16_div(phase_err, psi_scale);
        phase_err = q16_clamp(phase_err, q16_neg(Q16_ONE), Q16_ONE);
        o->phase_err = phase_err;

        if (o->pll_enable) {
            if (o->pll_settle_left > 0) {
                o->pll_settle_left--;
            } else {
                q16_t w = esp_foc_pid_update(&o->pll_pi, phase_err, 0);
                w = q16_clamp(w, q16_neg(o->w_max), o->w_max);
                esp_foc_pid_set_applied(&o->pll_pi, w);
                o->omega = w;
            }
        }
        o->theta = q16_wrap_pi(q16_add(o->theta, q16_div(o->omega, inv_ts)));
    }

    o->e_a = q16_div(q16_mul(q16_neg(o->omega), o->psi_r_b), FLUX_MWB);
    o->e_b = q16_div(q16_mul(o->omega, o->psi_r_a), FLUX_MWB);

    o->i_a = i_alpha;
    o->i_b = i_beta;

    q16_t psi_sq = q16_add(q16_mul(o->psi_r_a, o->psi_r_a),
                           q16_mul(o->psi_r_b, o->psi_r_b));
    q16_t psi_min_sq = q16_mul(o->psi_lock_min, o->psi_lock_min);
    if (psi_sq >= psi_min_sq) {
        o->unlock_acc = 0;
        if (o->lock_acc < 0xffffu) {
            o->lock_acc++;
        }
    } else {
        o->lock_acc = 0;
        if (o->unlock_acc < 0xffffu) {
            o->unlock_acc++;
        }
        if (o->unlock_acc >= o->unlock_need) {
            o->locked = false;
        }
    }
    if (o->lock_acc >= o->lock_need) {
        o->locked = true;
    }
}

static q16_t flux_get_theta(const esp_foc_observer_t *self)
{
    return self != NULL ? flux_of_c(self)->theta : 0;
}

static q16_t flux_get_omega(const esp_foc_observer_t *self)
{
    return self != NULL ? flux_of_c(self)->omega : 0;
}

q16_t esp_foc_observer_flux_get_omega_psi(const esp_foc_observer_flux_t *o)
{
    return o != NULL ? o->w_atan_f : 0;
}

uint16_t esp_foc_observer_flux_get_pll_settle_left(const esp_foc_observer_flux_t *o)
{
    return o != NULL ? o->pll_settle_left : 0;
}

uint32_t esp_foc_observer_flux_get_psi_clamp_count(const esp_foc_observer_flux_t *o)
{
    return o != NULL ? o->psi_clamp_count : 0u;
}

q16_t esp_foc_observer_flux_get_psi_peak(const esp_foc_observer_flux_t *o)
{
    return o != NULL ? q16_div(o->psi_peak, FLUX_MWB) : 0;
}

static q16_t flux_get_e_alpha(const esp_foc_observer_t *self)
{
    return self != NULL ? flux_of_c(self)->e_a : 0;
}

static q16_t flux_get_e_beta(const esp_foc_observer_t *self)
{
    return self != NULL ? flux_of_c(self)->e_b : 0;
}

static q16_t flux_get_psi_alpha(const esp_foc_observer_t *self)
{
    return self != NULL ? q16_div(flux_of_c(self)->psi_r_a, FLUX_MWB) : 0;
}

static q16_t flux_get_psi_beta(const esp_foc_observer_t *self)
{
    return self != NULL ? q16_div(flux_of_c(self)->psi_r_b, FLUX_MWB) : 0;
}

static q16_t flux_get_phase_err(const esp_foc_observer_t *self)
{
    return self != NULL ? flux_of_c(self)->phase_err : 0;
}

static bool flux_is_locked(const esp_foc_observer_t *self)
{
    return self != NULL && flux_of_c(self)->locked;
}

static void flux_bind(esp_foc_observer_flux_t *o)
{
    o->iface.update = flux_update;
    o->iface.reset = flux_reset;
    o->iface.set_theta = flux_set_theta;
    o->iface.set_omega = flux_set_omega;
    o->iface.set_extract = flux_set_extract;
    o->iface.set_pll_enable = flux_set_pll_enable;
    o->iface.get_theta = flux_get_theta;
    o->iface.get_omega = flux_get_omega;
    o->iface.get_e_alpha = flux_get_e_alpha;
    o->iface.get_e_beta = flux_get_e_beta;
    o->iface.get_psi_alpha = flux_get_psi_alpha;
    o->iface.get_psi_beta = flux_get_psi_beta;
    o->iface.get_phase_err = flux_get_phase_err;
    o->iface.is_locked = flux_is_locked;
}

esp_err_t esp_foc_observer_flux_init(esp_foc_observer_flux_t *o,
                                     const esp_foc_observer_flux_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(o != NULL && cfg != NULL, ESP_ERR_INVALID_ARG, "foc_flux",
                        "null");
    ESP_RETURN_ON_FALSE(cfg->rs_ohm > 0.0f && cfg->ls_h > 0.0f, ESP_ERR_INVALID_ARG,
                        "foc_flux", "Rs/Ls");
    ESP_RETURN_ON_FALSE(cfg->psi_f_wb > 0.0f, ESP_ERR_INVALID_ARG, "foc_flux", "psi_f");
    ESP_RETURN_ON_FALSE(cfg->ts_s > 0.0f && cfg->ts_s < 0.01f, ESP_ERR_INVALID_ARG,
                        "foc_flux", "Ts");

    const float ts = cfg->ts_s;
    const float f_ny = 0.1f / ts;
    const float obs_bw = cfg->obs_bw_hz;
    const float track_bw = cfg->track_bw_hz;
    const float blend_hz = cfg->blend_hz;
    const float obs_z = cfg->obs_zeta > 0.0f ? cfg->obs_zeta : 0.70f;
    const float tr_z = cfg->track_zeta > 0.0f ? cfg->track_zeta : 0.70f;

    ESP_RETURN_ON_FALSE(obs_bw > 0.0f && track_bw > 0.0f && blend_hz > 0.0f,
                        ESP_ERR_INVALID_ARG, "foc_flux", "bw");
    ESP_RETURN_ON_FALSE(track_bw < obs_bw && obs_bw < f_ny, ESP_ERR_INVALID_ARG,
                        "foc_flux", "bw stack");
    ESP_RETURN_ON_FALSE(obs_z > 0.4f && obs_z < 1.2f, ESP_ERR_INVALID_ARG,
                        "foc_flux", "obs_zeta");
    ESP_RETURN_ON_FALSE(tr_z > 0.4f && tr_z < 1.2f, ESP_ERR_INVALID_ARG,
                        "foc_flux", "track_zeta");

    float pll_kp = 0.0f;
    float pll_ki = 0.0f;
    ESP_RETURN_ON_ERROR(
        esp_foc_pid_design_integrator(1.0f, 1.0f / ts, track_bw, tr_z, &pll_kp, &pll_ki),
        "foc_flux", "pll");

    float b0 = 1.0f - expf(-2.0f * (float)M_PI * obs_bw * ts);
    if (b0 < 0.001f) {
        b0 = 0.001f;
    }
    if (b0 > 0.5f) {
        b0 = 0.5f;
    }

    float frac = cfg->psi_lock_frac;
    if (frac <= 0.0f) {
        frac = 0.5f;
    }
    if (frac > 1.0f) {
        frac = 1.0f;
    }

    memset(o, 0, sizeof(*o));
    o->rs = q16_from_float(cfg->rs_ohm);
    o->ls_mh = q16_from_float(cfg->ls_h * 1000.0f);
    o->psi_f_mwb = q16_from_float(cfg->psi_f_wb * 1000.0f);
    o->inv_ts = q16_from_float(1.0f / ts);
    o->lambda_ts = q16_from_float(2.0f * (float)M_PI * blend_hz * ts);
    o->lpf_b0 = q16_from_float(b0);
    ESP_RETURN_ON_ERROR(esp_foc_pid_init(&o->pll_pi, pll_kp, pll_ki, 0.0f, 0.0f, ts),
                        "foc_flux", "pll_init");
    o->w_max = q16_from_float(cfg->w_max_rads > 0.0f
                                  ? cfg->w_max_rads
                                  : 2.0f * (float)M_PI * 1000.0f);
    o->psi_lock_min = q16_from_float(frac * cfg->psi_f_wb * 1000.0f);
    {
        const float psi_max_frac = (cfg->psi_max_frac == 0.0f)
                                       ? ESP_FOC_OBSERVER_PSI_MAX_FRAC_DEFAULT
                                       : cfg->psi_max_frac;
        o->psi_max_mag = psi_max_frac > 0.0f
                             ? q16_from_float(psi_max_frac * cfg->psi_f_wb * 1000.0f)
                             : 0;
    }
    o->psi_clamp_count = 0u;
    o->psi_peak = 0;
    o->lock_need = cfg->lock_count == 0u ? 200u : cfg->lock_count;
    o->unlock_need = cfg->unlock_count == 0u
                         ? (uint16_t)((o->lock_need <= 6553u)
                                          ? o->lock_need * 10u
                                          : 65535u)
                         : cfg->unlock_count;
    o->pll_enable = true;
    o->extract = ESP_FOC_ANGLE_PLL;
    {
        float settle_s = cfg->pll_settle_ms * 0.001f;
        unsigned n = 0u;
        if (settle_s > 0.0f) {
            n = (unsigned)(settle_s / ts + 0.5f);
            if (n > 65535u) {
                n = 65535u;
            }
        }
        o->pll_settle_need = (uint16_t)n;
        o->pll_settle_left = 0;
    }
    {
        float wb0 = 1.0f - expf(-2.0f * (float)M_PI * 80.0f * ts);
        if (wb0 < 0.001f) {
            wb0 = 0.001f;
        }
        if (wb0 > 0.5f) {
            wb0 = 0.5f;
        }
        o->w_lpf_b0 = q16_from_float(wb0);
    }

    flux_bind(o);
    return ESP_OK;
}

esp_err_t esp_foc_observer_flux_set_bw(esp_foc_observer_flux_t *o,
                                       float obs_bw_hz,
                                       float track_bw_hz,
                                       float track_zeta)
{
    ESP_RETURN_ON_FALSE(o != NULL && o->inv_ts > 0, ESP_ERR_INVALID_ARG, "foc_flux",
                        "uninit");
    ESP_RETURN_ON_FALSE(obs_bw_hz > 0.0f && track_bw_hz > 0.0f, ESP_ERR_INVALID_ARG,
                        "foc_flux", "bw");
    ESP_RETURN_ON_FALSE(track_zeta > 0.4f && track_zeta < 1.2f, ESP_ERR_INVALID_ARG,
                        "foc_flux", "track_zeta");

    const float inv_ts_f = q16_to_float(o->inv_ts);
    ESP_RETURN_ON_FALSE(inv_ts_f > 1.0f, ESP_ERR_INVALID_ARG, "foc_flux", "inv_ts");
    const float ts = 1.0f / inv_ts_f;
    const float f_ny = 0.10f / ts;
    ESP_RETURN_ON_FALSE(track_bw_hz < obs_bw_hz && obs_bw_hz < f_ny, ESP_ERR_INVALID_ARG,
                        "foc_flux", "bw stack");

    float kp_f = 0.0f;
    float ki_f = 0.0f;
    ESP_RETURN_ON_ERROR(
        esp_foc_pid_design_integrator(1.0f, inv_ts_f, track_bw_hz, track_zeta, &kp_f, &ki_f),
        "foc_flux", "pll");
    ESP_RETURN_ON_ERROR(esp_foc_pid_set_kp(&o->pll_pi, kp_f), "foc_flux", "pll_kp");
    ESP_RETURN_ON_ERROR(esp_foc_pid_set_ki(&o->pll_pi, ki_f), "foc_flux", "pll_ki");

    float b0 = 1.0f - expf(-2.0f * (float)M_PI * obs_bw_hz * ts);
    if (b0 < 0.001f) {
        b0 = 0.001f;
    }
    if (b0 > 0.5f) {
        b0 = 0.5f;
    }
    o->lpf_b0 = q16_from_float(b0);
    return ESP_OK;
}
