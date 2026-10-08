/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Public PID API — software 2p2z by default; optional HW with soft fallback.
 */
#include "espFoC/utils/esp_foc_pid.h"

#include <math.h>
#include <stdbool.h>

#include "sdkconfig.h"
#include "esp_foc_pid_soft.h"

#if CONFIG_ESP_FOC_USE_HW_ACCEL_PID
bool esp_foc_pid_hw_available(void);
q16_t esp_foc_pid_hw_update(esp_foc_pid_t *p, q16_t sp, q16_t meas);
#endif

esp_err_t esp_foc_pid_init(esp_foc_pid_t *p, float kp, float ki, float kd, float n_hz, float ts)
{
    return esp_foc_pid_soft_init(p, kp, ki, kd, n_hz, ts);
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
    esp_foc_pid_soft_reset(p);
}

q16_t esp_foc_pid_update(esp_foc_pid_t *p, q16_t sp, q16_t meas)
{
#if CONFIG_ESP_FOC_USE_HW_ACCEL_PID
    if (esp_foc_pid_hw_available()) {
        return esp_foc_pid_hw_update(p, sp, meas);
    }
#endif
    return esp_foc_pid_soft_update(p, sp, meas);
}

void esp_foc_pid_set_applied(esp_foc_pid_t *p, q16_t u_applied)
{
    esp_foc_pid_soft_set_applied(p, u_applied);
}

void esp_foc_pid_set_ff(esp_foc_pid_t *p, q16_t ff)
{
    esp_foc_pid_soft_set_ff(p, ff);
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
    esp_foc_pid_soft_set_bypass(p, on);
}

esp_err_t esp_foc_pid_set_kp(esp_foc_pid_t *p, float kp)
{
    return esp_foc_pid_soft_set_kp(p, kp);
}

esp_err_t esp_foc_pid_set_ki(esp_foc_pid_t *p, float ki)
{
    return esp_foc_pid_soft_set_ki(p, ki);
}

esp_err_t esp_foc_pid_set_kd(esp_foc_pid_t *p, float kd)
{
    return esp_foc_pid_soft_set_kd(p, kd);
}
