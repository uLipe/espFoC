/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Trapezoidal position profile, one O(1) integer step per control sample.
 */
#include "espFoC/motor_control/esp_foc_traj.h"

#include <math.h>
#include <stddef.h>

#define TRAJ_Q32 4294967296.0

esp_err_t esp_foc_traj_set_limits(esp_foc_traj_t *t, float a_max, float v_max)
{
    if ((t == NULL) || (t->fs_hz == 0u) || !(a_max > 0.0f) || !(v_max > 0.0f)) {
        return ESP_ERR_INVALID_ARG;
    }
    const double ts = 1.0 / (double)t->fs_hz;
    const double a = (double)a_max * ts * ts * TRAJ_Q32;
    const double v = (double)v_max * ts * TRAJ_Q32;
    /* v^2 must fit int64 with headroom: under half a rad per sample. */
    if (!(a >= 1.0) || !(v >= 1.0) || (v >= 0.5 * TRAJ_Q32)) {
        return ESP_ERR_INVALID_ARG;
    }
    t->a_max = (int64_t)a;
    t->v_max = (int64_t)v;
    t->d_cap = (int64_t)(v * v / a) + t->v_max;
    return ESP_OK;
}

esp_err_t esp_foc_traj_init(esp_foc_traj_t *t, uint32_t fs_hz, float a_max, float v_max)
{
    if ((t == NULL) || (fs_hz == 0u)) {
        return ESP_ERR_INVALID_ARG;
    }
    t->fs_hz = fs_hz;
    const esp_err_t err = esp_foc_traj_set_limits(t, a_max, v_max);
    if (err != ESP_OK) {
        t->fs_hz = 0u;
        return err;
    }
    t->th = 0;
    t->w = 0;
    t->acc = 0;
    t->tgt = 0;
    t->active = false;
    return ESP_OK;
}

void esp_foc_traj_seed(esp_foc_traj_t *t, int64_t th_q16, q16_t w)
{
    t->th = th_q16 * 65536;
    t->tgt = t->th;
    t->w = ((int64_t)w << 16) / (int64_t)t->fs_hz;
    t->acc = 0;
    t->active = false;
}

void esp_foc_traj_set_target(esp_foc_traj_t *t, int64_t tgt_q16)
{
    t->tgt = tgt_q16 * 65536;
    t->active = true;
}

void esp_foc_traj_stop(esp_foc_traj_t *t)
{
    t->active = false;
    t->w = 0;
    t->acc = 0;
}

void esp_foc_traj_step(esp_foc_traj_t *t)
{
    const int64_t th = t->th;
    const int64_t w = t->w;
    const int64_t tgt = t->tgt;
    const int64_t a = t->a_max;
    const int64_t vmax = t->v_max;
    const int64_t d_cap = t->d_cap;

    if (!t->active) {
        t->acc = 0;
        return;
    }
    const int64_t d = tgt - th;
    if ((d == 0) && (w == 0)) {
        t->acc = 0;
        return;
    }
    /* On the target but moving: it is behind the motion. */
    const int64_t s = (d > 0) ? 1 : ((d < 0) ? -1 : ((w > 0) ? -1 : 1));
    const int64_t v = s * w;
    const int64_t ad = s * d;

    int64_t as = (v < vmax) ? ((vmax - v < a) ? (vmax - v) : a)
                            : ((v - vmax < a) ? (vmax - v) : -a);
    bool land = false;
    if (v > 0) {
        if (ad <= v) {
            /* Within one sample of the target. */
            land = true;
        } else if (ad < d_cap) {
            const int64_t v1 = v + as;
            const int64_t ad1 = ad - v - as / 2;
            if ((ad1 <= 0) || ((v1 > 0) && (v1 * v1 > 2 * a * ad1))) {
                as = -(v * v) / (2 * ad);
                land = (v + as <= 0);
            }
        }
    } else if ((v == 0) && (ad <= a / 2)) {
        land = true;
    }

    if (land) {
        t->acc = -w;
        t->w = 0;
        t->th = tgt;
        return;
    }
    const int64_t acc = s * as;
    const int64_t w1 = w + acc;
    t->acc = acc;
    t->w = w1;
    t->th = th + (w + w1) / 2;
}
