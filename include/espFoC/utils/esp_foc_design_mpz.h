/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

/**
 * @file esp_foc_design_mpz.h
 * @brief Matched Pole-Zero (MPZ) design helpers for current-loop PI gains.
 *
 * Single source of truth for the closed-form discrete PI synthesis used by
 * espFoC. Both the build-time Python generator and the runtime tuner share
 * this exact math, so any divergence is detected via cross-validation.
 *
 * Math (first-order R+L plant, ZOH discretized, espFoC PID structure):
 *   alpha = exp(-R * Ts / L)         -- discrete plant pole
 *   beta  = exp(-omega_bw * Ts)      -- desired closed-loop pole
 *   Kp    = R * (1 - beta) / (1 - alpha)
 *   Ki    = R * (1 - beta) / Ts
 * with omega_bw = 2*pi*bandwidth_hz.
 *
 * All inputs/outputs are Q16.16 except ts_us which is plain microseconds.
 * No floating-point operations are performed.
 *
 * Precision note: motor inductance below ~0.5 mH suffers from Q16
 * quantization (1 LSB ~= 15 uH). For ultra-low-L motors plan a future
 * path using a Q24/Q32 intermediate; current users with such motors
 * should fall back on manual gain entry via esp_foc_axis_set_current_pi_*.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ESP_FOC_DESIGN_OK = 0,
    ESP_FOC_DESIGN_ERR_INVALID_ARG = -1,   /* zero/negative R, L, Ts or bw */
    ESP_FOC_DESIGN_ERR_OUT_OF_RANGE = -2,  /* bw above Nyquist (bw*Ts >= 0.5) */
} esp_foc_design_status_t;

typedef struct {
    q16_t motor_r_ohm;       /* phase resistance R [Ohm] */
    q16_t motor_l_h;         /* phase inductance L [H] */
    uint32_t loop_ts_us;     /* sample period of the current loop [us] */
    q16_t bandwidth_hz;      /* desired closed-loop bandwidth [Hz] */
    q16_t v_max;             /* output saturation [V] (for anti-windup) */
} esp_foc_pi_design_input_t;

typedef struct {
    q16_t kp;                /* PI proportional gain (espFoC PID convention) */
    q16_t ki;                /* PI integral gain (espFoC PID convention) */
    q16_t integrator_limit;  /* anti-windup clamp on integrator state */
    q16_t alpha;             /* discrete plant pole exp(-R*Ts/L) */
    q16_t beta;              /* designed closed-loop pole exp(-w_bw*Ts) */
    bool valid;
} esp_foc_pi_design_output_t;

/**
 * @brief Compute exp(-x) in Q16.16 for x >= 0.
 *
 * Uses Pade [3/3] approximation with range reduction by repeated halving:
 *   exp(-x) = (exp(-x/2))^2
 * This keeps the Pade argument in the [0, 1] range where its relative
 * error stays below ~5e-5. Saturates to 0 for very large x and to Q16_ONE
 * for x == 0. Negative inputs are treated as 0 (function contract: x >= 0).
 *
 * @param x non-negative Q16 value
 * @return exp(-x) as Q16, in [0, Q16_ONE]
 */
q16_t esp_foc_q16_exp_neg(q16_t x);

/**
 * @brief MPZ design of current-loop PI gains.
 *
 * @param in  design inputs (motor params, loop period, target bandwidth)
 * @param out resulting gains and design artifacts (alpha, beta)
 * @return ESP_FOC_DESIGN_OK or a negative status on validation failure
 */
esp_foc_design_status_t esp_foc_design_pi_current_mpz_q16(
    const esp_foc_pi_design_input_t *in,
    esp_foc_pi_design_output_t *out);

#ifdef __cplusplus
}
#endif
