/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Discrete LQR for small single-input plants. Double precision, design time
 * only: nothing here runs in the control interrupt.
 */
#define ESP_FOC_LQR_N_MAX 5

typedef struct {
    uint8_t n;
    double a[ESP_FOC_LQR_N_MAX][ESP_FOC_LQR_N_MAX];
    double b[ESP_FOC_LQR_N_MAX];
} esp_foc_lqr_model_t;

/*
 * Servo plant behind a current loop, ZOH at ts_s:
 *   theta' = w / pp          (mechanical rad, position models only)
 *   w'     = k · i           (electrical rad/s, k in rad/s^2 per A)
 *   i'     = (u_prev - i) / tau_i
 * u_prev is the iq reference of the previous sample: one sample from the
 * speed measurement to the reference acting. The integral state, when
 * present, is the time integral of theta (position) or of w (speed).
 *
 * State order: [theta,] w, i, u_prev[, z].
 */
typedef struct {
    double k_rads2_a;
    uint8_t pole_pairs;
    double ts_s;
    double tau_i_s;
    bool position;
    bool integral;
} esp_foc_lqr_servo_t;

esp_err_t esp_foc_lqr_servo_model(const esp_foc_lqr_servo_t *p, esp_foc_lqr_model_t *m);

/*
 * Solves the DARE for diag(q) and r by structured doubling, then
 * k = (r + b'Pb)^-1 b'P a, u = -k x. Refuses (ESP_ERR_INVALID_STATE) a
 * solution that does not converge or whose closed loop a - b k is not
 * strictly stable. No heap; about 1.2 KB of stack, soft double on chips
 * without a double FPU (tens of ms on a 160 MHz RISC-V).
 *
 * @return ESP_OK, ESP_ERR_INVALID_ARG (NULL, n out of range, q < 0, r <= 0,
 *         non-finite entries) or ESP_ERR_INVALID_STATE.
 */
esp_err_t esp_foc_lqr_dare(const esp_foc_lqr_model_t *m, const double *q, double r, double *k);

/* Spectral-radius proxy of a - b k: the infinity norm of (a - b k)^(2^14),
 * or 1e12 once it diverges. Below 1 means it decays over that horizon. */
double esp_foc_lqr_cl_decay(const esp_foc_lqr_model_t *m, const double *k);

#ifdef __cplusplus
}
#endif
