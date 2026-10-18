/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Current controllers behind esp_foc_current_ctrl_t: PI and ADRC.
 */
#include "espFoC/motor_control/esp_foc_current_ctrl.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define ADRC_TWO_PI     6.28318530718f
#define ADRC_WO_POLE_K  0.8f
#define ADRC_WO_FS_DIV  16.0f

static bool plant_ok(const esp_foc_current_ctrl_design_t *d)
{
    return (d->rs_ohm > 0.0f) && (d->ls_h > 0.0f) && (d->vdc_v > 0.0f) && (d->fs_hz > 0.0f) &&
           (d->bw_hz > 0.0f) && (d->gain_scale > 0.0f) && (d->gain_scale <= 1.0f);
}

/* --- PI ------------------------------------------------------------------ */

static esp_foc_current_pi_t *pi_from(esp_foc_current_ctrl_t *self)
{
    return (esp_foc_current_pi_t *)((char *)self - offsetof(esp_foc_current_pi_t, iface));
}

static esp_err_t pi_design(esp_foc_current_ctrl_t *self, const esp_foc_current_ctrl_design_t *d)
{
    esp_foc_current_pi_t *pi = pi_from(self);
    if (!plant_ok(d) || !(d->kp >= 0.0f) || !(d->ki >= 0.0f)) {
        return ESP_ERR_INVALID_ARG;
    }
    float kp = d->kp;
    float ki = d->ki;
    if ((kp == 0.0f) && (ki == 0.0f)) {
        esp_err_t err = esp_foc_pid_design_imc_zoh(d->vdc_v / d->rs_ohm, d->ls_h / d->rs_ohm,
                                                   d->fs_hz, d->bw_hz, &kp, &ki);
        if (err != ESP_OK) {
            return err;
        }
        kp *= d->gain_scale;
        ki *= d->gain_scale;
    }
    esp_err_t err = esp_foc_pid_init(&pi->d, kp, ki, 0.0f, 0.0f, 1.0f / d->fs_hz);
    if (err == ESP_OK) {
        err = esp_foc_pid_init(&pi->q, kp, ki, 0.0f, 0.0f, 1.0f / d->fs_hz);
    }
    if (err != ESP_OK) {
        return err;
    }
    memset(&self->tune, 0, sizeof(self->tune));
    self->tune.kp = kp;
    self->tune.ki = ki;
    return ESP_OK;
}

static void pi_reset(esp_foc_current_ctrl_t *self)
{
    esp_foc_current_pi_t *pi = pi_from(self);
    esp_foc_pid_reset(&pi->d);
    esp_foc_pid_reset(&pi->q);
}

static void pi_update(esp_foc_current_ctrl_t *self, q16_t id_ref, q16_t iq_ref, q16_t id,
                      q16_t iq, q16_t *vd, q16_t *vq)
{
    esp_foc_current_pi_t *pi = pi_from(self);
    *vd = esp_foc_pid_update(&pi->d, id_ref, id);
    *vq = esp_foc_pid_update(&pi->q, iq_ref, iq);
}

static void pi_set_applied(esp_foc_current_ctrl_t *self, q16_t vd, q16_t vq)
{
    esp_foc_current_pi_t *pi = pi_from(self);
    esp_foc_pid_set_applied(&pi->d, vd);
    esp_foc_pid_set_applied(&pi->q, vq);
}

esp_foc_current_ctrl_t *esp_foc_current_pi_bind(esp_foc_current_pi_t *pi)
{
    if (pi == NULL) {
        return NULL;
    }
    memset(pi, 0, sizeof(*pi));
    pi->iface.design = pi_design;
    pi->iface.reset = pi_reset;
    pi->iface.update = pi_update;
    pi->iface.set_applied = pi_set_applied;
    return &pi->iface;
}

/* --- ADRC ---------------------------------------------------------------- */

static esp_foc_current_adrc_t *adrc_from(esp_foc_current_ctrl_t *self)
{
    return (esp_foc_current_adrc_t *)((char *)self - offsetof(esp_foc_current_adrc_t, iface));
}

static esp_err_t adrc_design(esp_foc_current_ctrl_t *self, const esp_foc_current_ctrl_design_t *d)
{
    esp_foc_current_adrc_t *ad = adrc_from(self);
    if (!plant_ok(d) || !(d->wo_ratio > 1.0f)) {
        return ESP_ERR_INVALID_ARG;
    }
    const float b0 = d->vdc_v / d->ls_h;
    const float pole = d->rs_ohm / d->ls_h;
    const float wc = d->bw_hz * d->gain_scale;
    /* An ESO slower than Rs/(4π·Ls) has l1 < 0 and corrects against the
     * measurement; on silicon that is a limit cycle. Keep wo near the plant
     * pole, capped where noise and the bridge delay take over. */
    const float wo_pole = fminf(ADRC_WO_POLE_K * pole / ADRC_TWO_PI, d->fs_hz / ADRC_WO_FS_DIV);
    const float wo = fmaxf(wc * d->wo_ratio, wo_pole);
    const float ts = 1.0f / d->fs_hz;
    esp_err_t err = esp_foc_adrc_init(&ad->d, b0, pole, wc, wo, ts, d->delay_ts);
    if (err == ESP_OK) {
        err = esp_foc_adrc_init(&ad->q, b0, pole, wc, wo, ts, d->delay_ts);
    }
    if (err != ESP_OK) {
        return err;
    }
    memset(&self->tune, 0, sizeof(self->tune));
    self->tune.wc_hz = wc;
    self->tune.wo_hz = wo;
    self->tune.b0 = b0;
    return ESP_OK;
}

static void adrc_reset(esp_foc_current_ctrl_t *self)
{
    esp_foc_current_adrc_t *ad = adrc_from(self);
    esp_foc_adrc_reset(&ad->d);
    esp_foc_adrc_reset(&ad->q);
}

static void adrc_update(esp_foc_current_ctrl_t *self, q16_t id_ref, q16_t iq_ref, q16_t id,
                        q16_t iq, q16_t *vd, q16_t *vq)
{
    esp_foc_current_adrc_t *ad = adrc_from(self);
    *vd = esp_foc_adrc_update(&ad->d, id_ref, id);
    *vq = esp_foc_adrc_update(&ad->q, iq_ref, iq);
}

static void adrc_set_applied(esp_foc_current_ctrl_t *self, q16_t vd, q16_t vq)
{
    esp_foc_current_adrc_t *ad = adrc_from(self);
    esp_foc_adrc_set_applied(&ad->d, vd);
    esp_foc_adrc_set_applied(&ad->q, vq);
}

esp_foc_current_ctrl_t *esp_foc_current_adrc_bind(esp_foc_current_adrc_t *adrc)
{
    if (adrc == NULL) {
        return NULL;
    }
    memset(adrc, 0, sizeof(*adrc));
    adrc->iface.design = adrc_design;
    adrc->iface.reset = adrc_reset;
    adrc->iface.update = adrc_update;
    adrc->iface.set_applied = adrc_set_applied;
    return &adrc->iface;
}
