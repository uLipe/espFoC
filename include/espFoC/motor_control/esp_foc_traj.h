/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Rest-to-rest position profile, trapezoidal in speed, one O(1) step per
 * control sample. State in Q32 per sample (rad, rad/sample, rad/sample^2)
 * so sub-LSB speeds still integrate.
 *
 * Each step accelerates toward the target, cruises at v_max, or brakes with
 * the deceleration that lands exactly on it, v^2 / (2 d), decided one step
 * ahead so it never exceeds a_max by more than rounding. A new target, a new
 * limit or a seed with the shaft moving is taken from the current state:
 * there is no precomputed plan to invalidate.
 */
typedef struct {
    int64_t th;
    int64_t w;
    int64_t acc;
    int64_t tgt;
    int64_t a_max;
    int64_t v_max;
    int64_t d_cap;    /* beyond it no braking can be due, keeps v^2 vs a·d in int64 */
    uint32_t fs_hz;
    bool active;
} esp_foc_traj_t;

/* a_max in rad/s^2, v_max in rad/s, both > 0; v_max under fs/2 rad/s. */
esp_err_t esp_foc_traj_init(esp_foc_traj_t *t, uint32_t fs_hz, float a_max, float v_max);
esp_err_t esp_foc_traj_set_limits(esp_foc_traj_t *t, float a_max, float v_max);

/* Reference at (th, w), target th, inactive. th Q16 rad, w Q16 rad/s. */
void esp_foc_traj_seed(esp_foc_traj_t *t, int64_t th_q16, q16_t w);
/* Starts or redirects the motion; th Q16 rad. */
void esp_foc_traj_set_target(esp_foc_traj_t *t, int64_t tgt_q16);
void esp_foc_traj_stop(esp_foc_traj_t *t);

void esp_foc_traj_step(esp_foc_traj_t *t);

static inline int64_t esp_foc_traj_theta_q16(const esp_foc_traj_t *t)
{
    return t->th >> 16;
}

/* rad/s, Q16. */
static inline q16_t esp_foc_traj_w(const esp_foc_traj_t *t)
{
    return (q16_t)((t->w * (int64_t)t->fs_hz) >> 16);
}

/* True once the reference rests on the target. */
static inline bool esp_foc_traj_done(const esp_foc_traj_t *t)
{
    return (t->th == t->tgt) && (t->w == 0);
}

#ifdef __cplusplus
}
#endif
