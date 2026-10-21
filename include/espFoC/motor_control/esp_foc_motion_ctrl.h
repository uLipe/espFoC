/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "espFoC/utils/esp_foc_pi_q32.h"
#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct esp_foc_motion_ctrl_s esp_foc_motion_ctrl_t;

typedef enum {
    ESP_FOC_MOTION_VELOCITY = 0,
    ESP_FOC_MOTION_POSITION,
} esp_foc_motion_mode_t;

/* Plant, limits and targets, shared by every controller. Float, task context. */
typedef struct {
    float k_rads2_a;          /* dω_e/dt per amp of iq */
    uint8_t pole_pairs;
    float fs_hz;              /* rate update() runs at */
    float i_max_a;            /* |iq| the controller may command */
    bool position;            /* also design the position law */

    /* PI: speed PI on bw_hz, position P on its crossover over pos_sep. */
    float bw_hz;
    float zeta;
    float kp_w;               /* A per rad/s electrical; kp = ki = 0 designs */
    float ki_w;
    float kp_pos;             /* 1/s; 0 = 2π·2ζ·bw/pos_sep */
    float pos_sep;
    float corr_max_rads;      /* |P correction|, mechanical */
    float wm_max_rads;        /* |ω*|, mechanical */

    /* LQR: Bryson weights sized for natural frequency wn_hz, lowered until
     * the speed gain spends at most fb_iq_budget_a on one ripple sigma. A
     * physical knob > 0 replaces its derived weight and stops the descent. */
    float wn_hz;
    float i_bw_hz;            /* closed current loop, first-order lag */
    float ripple_rads;        /* speed feedback sigma, electrical; 0 = unknown */
    float fb_iq_budget_a;
    float theta_max_rad;      /* position error worth i_max_a */
    float w_max_rads;         /* speed error worth i_max_a, electrical */
    float ti_s;               /* integral action time */
} esp_foc_motion_ctrl_design_t;

/* What design() settled on; fields the controller does not use stay 0. */
typedef struct {
    float kp_w;
    float ki_w;
    float kp_pos;
    float wn_hz;
    /* u = -k·x, SI. Position: θ mech rad, ω_e rad/s, iq A, u_prev A, ∫θ
     * rad·s. Velocity: ω_e, iq, u_prev, ∫ω rad. */
    float k_pos[5];
    float k_vel[4];
    uint8_t n_pos;
    uint8_t n_vel;
} esp_foc_motion_ctrl_tune_t;

/* One slot. References already carry the trajectory; iq_ff is every
 * feedforward the stack adds to the controller output. */
typedef struct {
    int64_t e_th;             /* θ* - θ, mechanical rad Q16 (position) */
    q16_t w_ff_m;             /* ω*, mechanical rad/s (position) */
    q16_t w_ref_e;            /* ω*, electrical rad/s (velocity) */
    q16_t w;                  /* measured ω_e */
    q16_t iq_ff;
} esp_foc_motion_in_t;

/**
 * Speed and position law of the sensored stack, between the encoder slot
 * and the iq reference. update() runs in the control interrupt, O(1), Q16;
 * returns the iq reference clamped to ±i_max_a and writes the electrical
 * speed reference it tracked. reset() makes the next output equal held +
 * iq_ff with the references on the measurement (bumpless).
 *
 * Concrete controllers embed this struct first and bind it in their init;
 * the hot-path helpers below do not check for NULL.
 */
struct esp_foc_motion_ctrl_s {
    esp_err_t (*design)(esp_foc_motion_ctrl_t *self, const esp_foc_motion_ctrl_design_t *d);
    void (*reset)(esp_foc_motion_ctrl_t *self, esp_foc_motion_mode_t mode, q16_t held);
    q16_t (*update)(esp_foc_motion_ctrl_t *self, esp_foc_motion_mode_t mode,
                    const esp_foc_motion_in_t *in, q16_t *w_ref);
    esp_foc_motion_ctrl_tune_t tune;
};

static inline esp_err_t esp_foc_motion_ctrl_design(esp_foc_motion_ctrl_t *c,
                                                   const esp_foc_motion_ctrl_design_t *d)
{
    return ((c == NULL) || (d == NULL)) ? ESP_ERR_INVALID_ARG : c->design(c, d);
}

static inline void esp_foc_motion_ctrl_reset(esp_foc_motion_ctrl_t *c, esp_foc_motion_mode_t mode,
                                             q16_t held)
{
    c->reset(c, mode, held);
}

static inline q16_t esp_foc_motion_ctrl_update(esp_foc_motion_ctrl_t *c,
                                               esp_foc_motion_mode_t mode,
                                               const esp_foc_motion_in_t *in, q16_t *w_ref)
{
    return c->update(c, mode, in, w_ref);
}

/* Cascade: position P -> speed PI. The I share only integrates when it does
 * not push the output further into the clamp. */
typedef struct {
    esp_foc_motion_ctrl_t iface;
    esp_foc_pi_q32_t w;
    q16_t kp_pos;
    q16_t corr_max;
    q16_t wm_max;
    q16_t i_max;
    int32_t pp;
    float fs_hz;
} esp_foc_motion_pi_t;

/* LQR on the double integrator with the current-loop lag and one sample of
 * delay as states. The speed law always carries ∫ω: without it any drag is
 * a standing speed error. The position law carries ∫θ when bound with
 * integral; without it the rotor parks inside the band where the
 * proportional share stays under the static friction. The lag and delay
 * states are predicted from the controller's own output, not measured. */
typedef struct {
    int64_t kth;              /* Q32 */
    int64_t kw;
    int64_t ki;
    int64_t ku;
    int64_t kz;               /* Q16: ∫ error in Q32 */
    int64_t z_lim;            /* |kz·z| <= i_max */
    q16_t e_lim;              /* |θ error| fed to kth, bounds kth·e in int64 */
} esp_foc_motion_lqr_gains_t;

typedef struct {
    esp_foc_motion_ctrl_t iface;
    esp_foc_motion_lqr_gains_t g[2];   /* by esp_foc_motion_mode_t */
    int64_t alpha_q32;        /* e^(-Ts/τ_i) */
    int64_t ts_q32;
    int64_t z;
    q16_t di;
    q16_t dup;
    q16_t i_max;
    int32_t pp;
    bool integral;            /* position law only */
} esp_foc_motion_lqr_t;

esp_foc_motion_ctrl_t *esp_foc_motion_pi_bind(esp_foc_motion_pi_t *pi);
esp_foc_motion_ctrl_t *esp_foc_motion_lqr_bind(esp_foc_motion_lqr_t *lqr, bool integral);

/* PI runtime retunes; fs_hz from the last design or bind. */
esp_err_t esp_foc_motion_pi_set_speed_gains(esp_foc_motion_pi_t *pi, float kp, float ki);
esp_err_t esp_foc_motion_pi_set_position_kp(esp_foc_motion_pi_t *pi, float kp);

#ifdef __cplusplus
}
#endif
