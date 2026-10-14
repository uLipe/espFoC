/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * PID coefficient design and setters; update() is inline in the header.
 */
#include "espFoC/utils/esp_foc_pid.h"

#include <math.h>

static esp_err_t redesign(esp_foc_pid_t *p)
{
    float ts = p->ts;
    float kp = p->kp;
    float ki = p->ki;
    float kd = p->kd;
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;

    if (!(ts > 0.0f)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (kd == 0.0f && ki == 0.0f) {
        b0 = kp;
        b1 = 0.0f;
        b2 = 0.0f;
        a1 = 0.0f;
        a2 = 0.0f;
    } else if (kd == 0.0f) {
        float alpha = 2.0f / ts;
        b0 = kp + ki / alpha;
        b1 = ki / alpha - kp;
        b2 = 0.0f;
        a1 = -1.0f;
        a2 = 0.0f;
    } else {
        float n = (p->n_hz > 0.0f) ? (2.0f * (float)M_PI * p->n_hz) : (10.0f / ts);
        float alpha = 2.0f / ts;
        float a_s = kp + kd * n;
        float b_s = kp * n + ki;
        float c_s = ki * n;
        float d_s = n;
        float den = alpha * alpha + d_s * alpha;
        if (fabsf(den) < 1e-12f) {
            return ESP_ERR_INVALID_ARG;
        }
        b0 = (a_s * alpha * alpha + b_s * alpha + c_s) / den;
        b1 = (-2.0f * a_s * alpha * alpha + 2.0f * c_s) / den;
        b2 = (a_s * alpha * alpha - b_s * alpha + c_s) / den;
        a1 = (-2.0f * alpha * alpha) / den;
        a2 = (alpha * alpha - d_s * alpha) / den;
    }

    p->b0 = q16_from_float(b0);
    p->b1 = q16_from_float(b1);
    p->b2 = q16_from_float(b2);
    p->a1 = q16_from_float(a1);
    p->a2 = q16_from_float(a2);
    return ESP_OK;
}

esp_err_t esp_foc_pid_init(esp_foc_pid_t *p, float kp, float ki, float kd, float n_hz, float ts)
{
    if (p == NULL || !(ts > 0.0f)) {
        return ESP_ERR_INVALID_ARG;
    }
    *p = (esp_foc_pid_t){0};
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->n_hz = n_hz;
    p->ts = ts;
    return redesign(p);
}

esp_err_t esp_foc_pid_design_imc_zoh(float k_plant, float tau_s, float fs_hz,
                                     float bw_hz, float *kp, float *ki)
{
    if (kp == NULL || ki == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!(k_plant > 0.0f) || !(tau_s > 0.0f) || !(fs_hz > 0.0f) ||
        !(bw_hz > 0.0f) || !(bw_hz < 0.5f * fs_hz)) {
        return ESP_ERR_INVALID_ARG;
    }

    float ts = 1.0f / fs_hz;
    float a = expf(-ts / tau_s);
    float p = expf(-2.0f * (float)M_PI * bw_hz * ts);
    float one_m_a = 1.0f - a;
    if (one_m_a < 1.0e-8f) {
        return ESP_ERR_INVALID_ARG;
    }

    float kc = (1.0f - p) / (k_plant * one_m_a);
    *kp = kc * (1.0f + a) * 0.5f;
    *ki = kc * one_m_a / ts;
    return ESP_OK;
}

esp_err_t esp_foc_pid_design_integrator(float k_plant, float fs_hz, float bw_hz,
                                        float zeta, float *kp, float *ki)
{
    if (kp == NULL || ki == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!(k_plant > 0.0f) || !(fs_hz > 0.0f) || !(bw_hz > 0.0f) ||
        !(bw_hz < 0.5f * fs_hz) || !(zeta > 0.4f) || !(zeta < 1.2f)) {
        return ESP_ERR_INVALID_ARG;
    }

    const float ts = 1.0f / fs_hz;
    const float wn = 2.0f * (float)M_PI * bw_hz;
    const float half = 0.5f * wn * ts;
    if (half >= 1.2f) {
        return ESP_ERR_INVALID_ARG;
    }
    const float wn_w = (2.0f / ts) * tanf(half);
    *kp = 2.0f * zeta * wn_w / k_plant;
    *ki = (wn_w * wn_w) / k_plant;
    if (!(*kp > 0.0f) || !(*ki > 0.0f) || (*kp >= 32000.0f) ||
        ((*ki) * ts >= 32000.0f)) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

void esp_foc_pid_reset(esp_foc_pid_t *p)
{
    if (p == NULL) {
        return;
    }
    p->e1 = 0;
    p->e2 = 0;
    p->u1 = 0;
    p->u2 = 0;
}

void esp_foc_pid_set_pmsm_ff(esp_foc_pid_t *pd, esp_foc_pid_t *pq, q16_t we,
                             q16_t id, q16_t iq, q16_t ls, q16_t psi_f,
                             q16_t inv_vdc)
{
    if (pd == NULL || pq == NULL) {
        return;
    }
    const q16_t w_ls = q16_mul(we, ls);
    pd->ff = q16_mul(q16_neg(q16_mul(w_ls, iq)), inv_vdc);
    pq->ff = q16_mul(q16_add(q16_mul(w_ls, id), q16_mul(we, psi_f)), inv_vdc);
}

void esp_foc_pid_set_bypass(esp_foc_pid_t *p, bool on)
{
    if (p == NULL) {
        return;
    }
    p->bypass = on;
}

esp_err_t esp_foc_pid_set_kp(esp_foc_pid_t *p, float kp)
{
    if (p == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    p->kp = kp;
    return redesign(p);
}

esp_err_t esp_foc_pid_set_ki(esp_foc_pid_t *p, float ki)
{
    if (p == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    p->ki = ki;
    return redesign(p);
}

esp_err_t esp_foc_pid_set_kd(esp_foc_pid_t *p, float kd)
{
    if (p == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    p->kd = kd;
    return redesign(p);
}
