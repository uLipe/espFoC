/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Mid/high-speed BEMF observer for PMSM (Q16.16).
 *
 * Voltage-model BEMF on measured i: ê = v − Rs i − Ls di/dt, then LPF.
 * PMSM: eα = −ψf·ωe·sinθe, eβ = ψf·ωe·cosθe
 *   atan2: θe = atan2(−êα, êβ)
 *   PLL:   phase detector on êαβ (no atan2 on hot path)
 *
 * θ is tracked on the lagged êαβ and corrected on output by atan(ωe/ωc) plus π
 * when ωe < 0; keeping both corrections outside the loops is what keeps ω from
 * feeding back into itself.
 */
#include <math.h>
#include <stddef.h>
#include <string.h>

#include "esp_check.h"
#include "espFoC/motor_control/esp_foc_observer_bemf.h"
#include "espFoC/utils/esp_foc_angle.h"
#include "espFoC/utils/esp_foc_trig.h"

static esp_foc_observer_bemf_t *bemf_of(esp_foc_observer_t *self)
{
    return (esp_foc_observer_bemf_t *)((char *)self -
                                       offsetof(esp_foc_observer_bemf_t, iface));
}

static const esp_foc_observer_bemf_t *bemf_of_c(const esp_foc_observer_t *self)
{
    return (const esp_foc_observer_bemf_t *)((const char *)self -
                                             offsetof(esp_foc_observer_bemf_t, iface));
}

static q16_t lpf1(q16_t *w, q16_t x, q16_t b0)
{
    q16_t y = *w;
    y = q16_add(y, q16_mul(b0, q16_sub(x, y)));
    *w = y;
    return y;
}

static void bemf_reset(esp_foc_observer_t *self)
{
    if (self == NULL) {
        return;
    }
    esp_foc_observer_bemf_t *o = bemf_of(self);
    o->i_hat_a = 0;
    o->i_hat_b = 0;
    o->emf_hat_a = 0;
    o->emf_hat_b = 0;
    o->e_a = 0;
    o->e_b = 0;
    o->theta = 0;
    o->omega = 0;
    o->pll_integ = 0;
    o->theta_prev = 0;
    o->theta_comp = 0;
    o->w_slow = 0;
    o->lpf_w_a = 0;
    o->lpf_w_b = 0;
    o->lock_acc = 0;
    o->unlock_acc = 0;
    o->locked = false;
    o->reverse = false;
    o->have_theta_prev = false;
    o->have_i_prev = false;
    o->pll_enable = true;
}

static void bemf_set_extract(esp_foc_observer_t *self, esp_foc_angle_extract_t extract)
{
    if (self == NULL) {
        return;
    }
    bemf_of(self)->extract = extract;
}

static void bemf_set_pll_enable(esp_foc_observer_t *self, bool enable)
{
    if (self == NULL) {
        return;
    }
    bemf_of(self)->pll_enable = enable;
}

static void bemf_set_theta(esp_foc_observer_t *self, q16_t theta_e)
{
    if (self == NULL) {
        return;
    }
    esp_foc_observer_bemf_t *o = bemf_of(self);
    o->theta = q16_wrap_pi(q16_sub(theta_e, o->theta_comp));
    o->theta_prev = o->theta;
    o->have_theta_prev = true;
}

static void bemf_set_omega(esp_foc_observer_t *self, q16_t omega_e)
{
    if (self == NULL) {
        return;
    }
    esp_foc_observer_bemf_t *o = bemf_of(self);
    o->omega = omega_e;
    o->pll_integ = omega_e;
    o->w_slow = omega_e;
}

static void bemf_update(esp_foc_observer_t *self,
                        q16_t i_alpha,
                        q16_t i_beta,
                        q16_t v_alpha,
                        q16_t v_beta)
{
    if (self == NULL) {
        return;
    }
    esp_foc_observer_bemf_t *o = bemf_of(self);

    const q16_t rs = o->alpha;
    const q16_t ls = o->beta;
    if (!o->have_i_prev) {
        o->i_hat_a = i_alpha;
        o->i_hat_b = i_beta;
        o->have_i_prev = true;
        return;
    }

    /* Voltage-model BEMF on measured i: e = v − Rs i − Ls di/dt.
     * A high-gain current-model ê copies the applied voltage and the PLL
     * then tracks its own Park frame. */
    q16_t dia = q16_mul(q16_sub(i_alpha, o->i_hat_a), o->inv_ts);
    q16_t dib = q16_mul(q16_sub(i_beta, o->i_hat_b), o->inv_ts);
    o->i_hat_a = i_alpha;
    o->i_hat_b = i_beta;
    q16_t e_raw_a =
        q16_sub(q16_sub(v_alpha, q16_mul(rs, i_alpha)), q16_mul(ls, dia));
    q16_t e_raw_b =
        q16_sub(q16_sub(v_beta, q16_mul(rs, i_beta)), q16_mul(ls, dib));
    o->e_a = lpf1(&o->lpf_w_a, e_raw_a, o->lpf_b0);
    o->e_b = lpf1(&o->lpf_w_b, e_raw_b, o->lpf_b0);

    /* LPF lag seen by êαβ at the current speed, from a slow ω estimate. */
    q16_t phi = 0;
    o->w_slow = q16_add(o->w_slow, q16_mul(o->comp_b0, q16_sub(o->omega, o->w_slow)));
    q16_t w = o->w_slow;
    q16_t w_abs = (w < 0) ? q16_neg(w) : w;
    if (o->lpf_wc > 0) {
        phi = esp_foc_atan2(w_abs, o->lpf_wc);
    }
    if (w < 0) {
        phi = q16_neg(phi);
    }

    /* BEMF gives the angle modulo π: eαβ = ψ·ω·(−sinθ, cosθ) flips with the
     * direction, so both extractors settle on θ+π when ω < 0. Resolve it with
     * the (hysteretic) sign of ω, which stays valid through the flip. */
    if (o->w_slow < q16_neg(o->w_sign_hyst)) {
        o->reverse = true;
    } else if (o->w_slow > o->w_sign_hyst) {
        o->reverse = false;
    }
    o->theta_comp = o->reverse
                        ? q16_wrap_pi(q16_add(phi, Q16_PI))
                        : q16_wrap_pi(phi);

    /* Both extractors track the *lagged* êαβ angle; φ is applied on the way
     * out only. Feeding φ back into the loop would make ω its own input. */
    if (o->extract == ESP_FOC_ANGLE_PLL) {
        if (o->pll_enable) {
            q16_t s;
            q16_t c;
            esp_foc_sincos(o->theta, &s, &c);
            q16_t phase_err = q16_sub(q16_mul(q16_neg(o->e_a), c), q16_mul(o->e_b, s));
            q16_t ea_abs = (o->e_a < 0) ? q16_neg(o->e_a) : o->e_a;
            q16_t eb_abs = (o->e_b < 0) ? q16_neg(o->e_b) : o->e_b;
            q16_t e_mag = q16_add(ea_abs, eb_abs);
            if (e_mag < o->e_lock_min) {
                e_mag = o->e_lock_min;
            }
            phase_err = q16_div(phase_err, e_mag);
            o->pll_integ = q16_add(o->pll_integ, q16_mul(o->pll_ki_ts, phase_err));
            o->omega = q16_add(q16_mul(o->pll_kp, phase_err), o->pll_integ);
        }
        o->theta = q16_wrap_pi(q16_add(o->theta, q16_mul(o->omega, o->ts)));
    } else {
        q16_t th_raw = esp_foc_atan2(q16_neg(o->e_a), o->e_b);
        if (o->have_theta_prev) {
            q16_t dth = q16_angle_delta(o->theta_prev, th_raw);
            o->omega = q16_mul(dth, o->inv_ts);
        }
        o->theta_prev = th_raw;
        o->have_theta_prev = true;
        o->theta = th_raw;
    }

    o->omega = q16_clamp(o->omega, q16_neg(o->w_max), o->w_max);
    o->pll_integ = q16_clamp(o->pll_integ, q16_neg(o->w_max), o->w_max);

    q16_t e_mag_sq = q16_add(q16_mul(o->e_a, o->e_a), q16_mul(o->e_b, o->e_b));
    q16_t e_min_sq = q16_mul(o->e_lock_min, o->e_lock_min);
    if (e_mag_sq >= e_min_sq) {
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

static q16_t bemf_get_theta(const esp_foc_observer_t *self)
{
    if (self == NULL) {
        return 0;
    }
    const esp_foc_observer_bemf_t *o = bemf_of_c(self);
    return q16_wrap_pi(q16_add(o->theta, o->theta_comp));
}

static q16_t bemf_get_omega(const esp_foc_observer_t *self)
{
    return self != NULL ? bemf_of_c(self)->omega : 0;
}

static q16_t bemf_get_e_alpha(const esp_foc_observer_t *self)
{
    return self != NULL ? bemf_of_c(self)->e_a : 0;
}

static q16_t bemf_get_e_beta(const esp_foc_observer_t *self)
{
    return self != NULL ? bemf_of_c(self)->e_b : 0;
}

static bool bemf_is_locked(const esp_foc_observer_t *self)
{
    return self != NULL && bemf_of_c(self)->locked;
}

static void bemf_bind(esp_foc_observer_bemf_t *o)
{
    o->iface.update = bemf_update;
    o->iface.reset = bemf_reset;
    o->iface.set_theta = bemf_set_theta;
    o->iface.set_omega = bemf_set_omega;
    o->iface.set_extract = bemf_set_extract;
    o->iface.set_pll_enable = bemf_set_pll_enable;
    o->iface.get_theta = bemf_get_theta;
    o->iface.get_omega = bemf_get_omega;
    o->iface.get_e_alpha = bemf_get_e_alpha;
    o->iface.get_e_beta = bemf_get_e_beta;
    o->iface.get_psi_alpha = NULL;
    o->iface.get_psi_beta = NULL;
    o->iface.get_phase_err = NULL;
    o->iface.is_locked = bemf_is_locked;
}

esp_err_t esp_foc_observer_bemf_init(esp_foc_observer_bemf_t *o,
                                     const esp_foc_observer_bemf_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(o != NULL && cfg != NULL, ESP_ERR_INVALID_ARG, "foc_obs",
                        "null");
    ESP_RETURN_ON_FALSE(cfg->rs_ohm > 0.0f && cfg->ls_h > 0.0f, ESP_ERR_INVALID_ARG,
                        "foc_obs", "Rs/Ls");
    ESP_RETURN_ON_FALSE(cfg->ts_s > 0.0f && cfg->ts_s < 0.01f, ESP_ERR_INVALID_ARG,
                        "foc_obs", "Ts");
    ESP_RETURN_ON_FALSE(cfg->emf_lpf_hz > 0.0f, ESP_ERR_INVALID_ARG, "foc_obs", "lpf");

    memset(o, 0, sizeof(*o));

    const float ts = cfg->ts_s;
    const float current_hz =
        cfg->current_model_hz > 0.0f ? cfg->current_model_hz : 500.0f;
    const float current_wc = 2.0f * (float)M_PI * current_hz;
    float current_gain = 2.0f * current_wc - cfg->rs_ohm / cfg->ls_h;
    if (current_gain < 0.0f) {
        current_gain = 0.0f;
    }

    o->alpha = q16_from_float(cfg->rs_ohm);
    o->beta = q16_from_float(cfg->ls_h);
    o->ts_over_l = q16_from_float(ts / cfg->ls_h);
    o->current_corr = q16_from_float(current_gain * ts);
    o->emf_corr =
        q16_from_float(cfg->ls_h * current_wc * current_wc * ts);
    o->current_wc = q16_from_float(current_wc);
    o->e_lock_min = q16_from_float(cfg->e_lock_min_v > 0.0f ? cfg->e_lock_min_v : 0.05f);
    o->inv_ts = q16_from_float(1.0f / ts);
    o->ts = q16_from_float(ts);

    float pll_kp;
    float pll_ki;
    if (cfg->pll_bw_hz > 0.0f) {
        const float z = cfg->pll_zeta > 0.0f ? cfg->pll_zeta : 0.707f;
        const float wn = 2.0f * (float)M_PI * cfg->pll_bw_hz;
        pll_kp = 2.0f * z * wn;
        pll_ki = wn * wn;
    } else {
        pll_kp = cfg->pll_kp;
        pll_ki = cfg->pll_ki;
    }
    ESP_RETURN_ON_FALSE(pll_kp >= 0.0f && pll_kp < 32000.0f, ESP_ERR_INVALID_ARG,
                        "foc_obs", "pll_kp");
    ESP_RETURN_ON_FALSE(pll_ki >= 0.0f && (pll_ki * ts) < 32000.0f,
                        ESP_ERR_INVALID_ARG, "foc_obs", "pll_ki");
    o->pll_kp = q16_from_float(pll_kp);
    o->pll_ki_ts = q16_from_float(pll_ki * ts);
    o->lock_need = cfg->lock_count == 0u ? 200u : cfg->lock_count;
    o->unlock_need = cfg->unlock_count == 0u
                         ? (uint16_t)((o->lock_need <= 6553u)
                                          ? o->lock_need * 10u
                                          : 65535u)
                         : cfg->unlock_count;
    o->extract = cfg->extract;
    o->pll_enable = true;
    o->w_max = q16_from_float(cfg->w_max_rads > 0.0f
                                  ? cfg->w_max_rads
                                  : 2.0f * (float)M_PI * 1000.0f);
    o->lpf_wc = q16_from_float(2.0f * (float)M_PI * cfg->emf_lpf_hz);
    o->w_sign_hyst = q16_from_float(2.0f * (float)M_PI * 5.0f);
    /* atan2 ω is a raw per-sample difference; φ(ω) must not follow that jitter. */
    o->comp_b0 = q16_from_float(1.0f - expf(-2.0f * (float)M_PI * 20.0f * ts));

    float b0 = 1.0f - expf(-2.0f * (float)M_PI * cfg->emf_lpf_hz * ts);
    if (b0 < 0.001f) {
        b0 = 0.001f;
    }
    if (b0 > 0.5f) {
        b0 = 0.5f;
    }
    o->lpf_b0 = q16_from_float(b0);

    bemf_bind(o);
    return ESP_OK;
}
