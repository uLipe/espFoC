/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Software 2p2z PID. Float only in coefficient design.
 */
#include "esp_foc_pid_soft.h"

#include <math.h>
#include <limits.h>

static q16_t sat_shift16(int64_t acc)
{
    int64_t v = acc >> 16;
    if (v > INT32_MAX) {
        return (q16_t)INT32_MAX;
    }
    if (v < INT32_MIN) {
        return (q16_t)INT32_MIN;
    }
    return (q16_t)v;
}

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

esp_err_t esp_foc_pid_soft_init(esp_foc_pid_t *p, float kp, float ki, float kd, float n_hz, float ts)
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

void esp_foc_pid_soft_reset(esp_foc_pid_t *p)
{
    if (p == NULL) {
        return;
    }
    p->e1 = 0;
    p->e2 = 0;
    p->u1 = 0;
    p->u2 = 0;
}

q16_t esp_foc_pid_soft_update(esp_foc_pid_t *p, q16_t sp, q16_t meas)
{
    if (p == NULL) {
        return 0;
    }

    if (p->bypass) {
        return q16_add(sp, p->ff);
    }

    q16_t b0 = p->b0;
    q16_t b1 = p->b1;
    q16_t b2 = p->b2;
    q16_t a1 = p->a1;
    q16_t a2 = p->a2;
    q16_t e1 = p->e1;
    q16_t e2 = p->e2;
    q16_t u1 = p->u1;
    q16_t u2 = p->u2;
    q16_t ff = p->ff;

    q16_t e = q16_sub(sp, meas);
    q16_t u = sat_shift16(((int64_t)b0 * (int64_t)e) +
                          ((int64_t)b1 * (int64_t)e1) +
                          ((int64_t)b2 * (int64_t)e2) -
                          ((int64_t)a1 * (int64_t)u1) -
                          ((int64_t)a2 * (int64_t)u2));
    u = q16_add(u, ff);

    p->e2 = e1;
    p->e1 = e;
    p->u2 = u1;
    p->u1 = u;
    return u;
}

void esp_foc_pid_soft_set_applied(esp_foc_pid_t *p, q16_t u_applied)
{
    if (p == NULL) {
        return;
    }
    p->u1 = u_applied;
}

void esp_foc_pid_soft_set_ff(esp_foc_pid_t *p, q16_t ff)
{
    if (p == NULL) {
        return;
    }
    p->ff = ff;
}

void esp_foc_pid_soft_set_bypass(esp_foc_pid_t *p, bool on)
{
    if (p == NULL) {
        return;
    }
    p->bypass = on;
}

esp_err_t esp_foc_pid_soft_set_kp(esp_foc_pid_t *p, float kp)
{
    if (p == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    p->kp = kp;
    return redesign(p);
}

esp_err_t esp_foc_pid_soft_set_ki(esp_foc_pid_t *p, float ki)
{
    if (p == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    p->ki = ki;
    return redesign(p);
}

esp_err_t esp_foc_pid_soft_set_kd(esp_foc_pid_t *p, float kd)
{
    if (p == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    p->kd = kd;
    return redesign(p);
}
