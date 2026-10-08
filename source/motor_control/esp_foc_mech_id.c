/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#include <math.h>
#include <string.h>

#include "esp_foc_mech_id.h"

#define MECH_ID_TWO_PI     6.28318530718f
#define MECH_ID_CAP_FRAC   0.90f
#define MECH_ID_MIN_DIQ    1.0e-4f
#define MECH_ID_MIN_PTS    8u

void esp_foc_mech_id_default_config(esp_foc_mech_id_config_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));
    cfg->iq_lo_frac = 0.15f;
    cfg->iq_hi_frac = 0.50f;
    cfg->win_ms = 250;
    cfg->sample_ms = 2;
    cfg->settle_ms = 20;
    cfg->we_entry_min_hz = 40.0f;
    cfg->we_max_hz = 500.0f;
    cfg->k_min = 800.0f;
    cfg->k_max = 40000.0f;
}

static void restore(const esp_foc_mech_id_ops_t *ops, q16_t id, q16_t iq)
{
    ops->set_idq(ops->ctx, id, iq);
}

static esp_err_t bail(const esp_foc_mech_id_ops_t *ops, q16_t id, q16_t iq,
                      esp_foc_mech_id_result_t *out, esp_err_t err)
{
    memset(out, 0, sizeof(*out));
    restore(ops, id, iq);
    return err;
}

static bool is_faulted(const esp_foc_mech_id_ops_t *ops)
{
    return (ops->faulted != NULL) && ops->faulted(ops->ctx);
}

static esp_err_t cfg_ok(const esp_foc_mech_id_ops_t *ops,
                        const esp_foc_mech_id_config_t *cfg)
{
    if (ops == NULL || cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (ops->set_idq == NULL || ops->get_omega_e == NULL ||
        ops->sleep_ms == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!(cfg->i_max_a > 0.0f) ||
        !(cfg->iq_lo_frac > 0.0f) ||
        !(cfg->iq_hi_frac > cfg->iq_lo_frac) ||
        !(cfg->iq_hi_frac <= 1.0f)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->win_ms == 0u || cfg->sample_ms == 0u ||
        (cfg->win_ms / cfg->sample_ms) < 4u ||
        !(cfg->we_entry_min_hz > 0.0f) ||
        !(cfg->we_max_hz > cfg->we_entry_min_hz) ||
        !(cfg->k_min > 0.0f) ||
        !(cfg->k_max > cfg->k_min)) {
        return ESP_ERR_INVALID_ARG;
    }
    float diq = cfg->i_max_a * (cfg->iq_hi_frac - cfg->iq_lo_frac);
    if (!(diq >= MECH_ID_MIN_DIQ)) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

esp_err_t esp_foc_mech_id_run(const esp_foc_mech_id_ops_t *ops,
                              const esp_foc_mech_id_config_t *cfg,
                              esp_foc_mech_id_result_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));

    esp_err_t err = cfg_ok(ops, cfg);
    if (err != ESP_OK) {
        return err;
    }

    q16_t id_ent = 0;
    q16_t iq_ent = 0;
    if (ops->get_idq != NULL) {
        ops->get_idq(ops->ctx, &id_ent, &iq_ent);
    }

    if (is_faulted(ops)) {
        return bail(ops, id_ent, iq_ent, out, ESP_ERR_INVALID_STATE);
    }

    float we0 = q16_to_float(ops->get_omega_e(ops->ctx));
    float we_min = cfg->we_entry_min_hz * MECH_ID_TWO_PI;
    float we_max = cfg->we_max_hz * MECH_ID_TWO_PI;
    if (!(fabsf(we0) >= we_min) || fabsf(we0) >= we_max) {
        return bail(ops, id_ent, iq_ent, out, ESP_ERR_INVALID_STATE);
    }

    /*
     * lo, hi, lo: the shaft speeds up through the sequence, so a speed-
     * dependent drag differs between any two windows. Averaging the lo
     * windows either side of hi puts their mean speed on hi's and cancels the
     * viscous term to first order; lo -> hi alone read K ~9% low at b = 0.4/s.
     */
    float sgn = (we0 >= 0.0f) ? 1.0f : -1.0f;
    const float iq_lo = sgn * cfg->i_max_a * cfg->iq_lo_frac;
    const float iq_hi = sgn * cfg->i_max_a * cfg->iq_hi_frac;
    const float iq_step[3] = {iq_lo, iq_hi, iq_lo};
    float acc[3];
    const uint32_t n_max = cfg->win_ms / cfg->sample_ms + 1u;
    const double ts = (double)cfg->sample_ms * 0.001;
    const float we_cap = we_max * MECH_ID_CAP_FRAC;

    for (int i = 0; i < 3; i++) {
        const float w_start = q16_to_float(ops->get_omega_e(ops->ctx));
        if (fabsf(w_start) >= we_cap) {
            return bail(ops, id_ent, iq_ent, out, ESP_ERR_INVALID_STATE);
        }
        /* Settle travel is charged to the window, so it is measured from here. */
        const float budget = (we_cap - fabsf(w_start)) / (float)(3 - i);

        q16_t iq_q = q16_from_float(iq_step[i]);
        ops->set_idq(ops->ctx, id_ent, iq_q);
        if (cfg->settle_ms > 0u) {
            ops->sleep_ms(ops->ctx, cfg->settle_ms);
        }
        if (is_faulted(ops)) {
            return bail(ops, id_ent, iq_ent, out, ESP_ERR_INVALID_STATE);
        }

        double s_t = 0.0;
        double s_tt = 0.0;
        double s_w = 0.0;
        double s_tw = 0.0;
        uint32_t n = 0;
        for (uint32_t k = 0; k < n_max; k++) {
            if (k > 0u) {
                ops->sleep_ms(ops->ctx, cfg->sample_ms);
                if (is_faulted(ops)) {
                    return bail(ops, id_ent, iq_ent, out, ESP_ERR_INVALID_STATE);
                }
            }
            const float w = q16_to_float(ops->get_omega_e(ops->ctx));
            if (fabsf(w) >= we_max) {
                return bail(ops, id_ent, iq_ent, out, ESP_ERR_INVALID_STATE);
            }
            const double t = (double)k * ts;
            s_t += t;
            s_tt += t * t;
            s_w += (double)w;
            s_tw += t * (double)w;
            n = k + 1u;
            if ((n >= MECH_ID_MIN_PTS) &&
                ((fabsf(w - w_start) >= budget) || (fabsf(w) >= we_cap) ||
                 (fabsf(w) <= we_min))) {
                break;
            }
        }
        const double den = ((double)n * s_tt) - (s_t * s_t);
        if (!(den > 0.0)) {
            return bail(ops, id_ent, iq_ent, out, ESP_ERR_INVALID_STATE);
        }
        acc[i] = (float)((((double)n * s_tw) - (s_t * s_w)) / den);
    }
    out->accel[0] = 0.5f * (acc[0] + acc[2]);
    out->accel[1] = acc[1];
    out->iq_a[0] = q16_to_float(q16_from_float(iq_lo));
    out->iq_a[1] = q16_to_float(q16_from_float(iq_hi));

    float diq = out->iq_a[1] - out->iq_a[0];
    if (!(fabsf(diq) >= MECH_ID_MIN_DIQ)) {
        return bail(ops, id_ent, iq_ent, out, ESP_ERR_INVALID_ARG);
    }
    float k = (out->accel[1] - out->accel[0]) / diq;
    if (!isfinite(k) || k < cfg->k_min || k > cfg->k_max) {
        /* The slopes stay: a refused K is the evidence the caller reports. */
        restore(ops, id_ent, iq_ent);
        out->k_rad_s2_per_a = 0.0f;
        out->j_kgm2 = 0.0f;
        out->valid_mask = 0u;
        return ESP_ERR_INVALID_RESPONSE;
    }

    out->k_rad_s2_per_a = k;
    out->valid_mask = ESP_FOC_MECH_ID_VALID_K;
    if (cfg->pole_pairs > 0 && cfg->psi_f_wb > 0.0f) {
        out->j_kgm2 = 1.5f * (float)(cfg->pole_pairs * cfg->pole_pairs) *
                      cfg->psi_f_wb / k;
        if (isfinite(out->j_kgm2) && out->j_kgm2 > 0.0f) {
            out->valid_mask |= ESP_FOC_MECH_ID_VALID_J;
        } else {
            out->j_kgm2 = 0.0f;
        }
    }

    restore(ops, id_ent, iq_ent);
    return ESP_OK;
}
