/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Discrete 2p2z PID in Q16.16 (Tustin of Kp + Ki/s + Kd N s/(s+N)).
 *
 * update() does not saturate. After an external clamp, call set_applied() so
 * the a1/a2 delays track the plant input (anti-windup). If set_applied() is
 * skipped, update() tracks its own unsaturated output (no AW).
 *
 * Optional HW via CONFIG_ESP_FOC_USE_HW_ACCEL_PID (soft fallback).
 */
typedef struct {
    q16_t b0;
    q16_t b1;
    q16_t b2;
    q16_t a1;
    q16_t a2;
    q16_t e1;
    q16_t e2;
    q16_t u1;
    q16_t u2;
    q16_t ff;
    bool bypass;
    float kp;
    float ki;
    float kd;
    float n_hz;
    float ts;
} esp_foc_pid_t;

esp_err_t esp_foc_pid_init(esp_foc_pid_t *p, float kp, float ki, float kd, float n_hz, float ts);
/**
 * Discrete IMC PI for a ZOH-sampled 1st-order plant G(z)=K(1-a)/(z-a),
 * a=exp(-Ts/τ). Desired T(z)=(1-p)/(z-p) with p=exp(-2π·bw·Ts).
 *
 * Returns Kp/Ki of the parallel PI whose Tustin form (used by init) equals
 * C(z)=Kc(z-a)/(z-1). bw_hz must stay below fs/2 (and well below any
 * measurement LPF).
 */
esp_err_t esp_foc_pid_design_imc_zoh(float k_plant, float tau_s, float fs_hz,
                                     float bw_hz, float *kp, float *ki);
/**
 * Discrete type-2 PI for an integrator plant G(s)=K/s (VCO / tracking PLL).
 *
 * Analog C(s)=kp+ki/s with wn=2π·bw, then Tustin (used by init) sees a
 * prewarped wn so the closed-loop bandwidth lands at bw_hz, not below it.
 * zeta is damping (0.7). Not IMC-zoh: that helper is first-order lag only.
 */
esp_err_t esp_foc_pid_design_integrator(float k_plant, float fs_hz, float bw_hz,
                                        float zeta, float *kp, float *ki);
void esp_foc_pid_reset(esp_foc_pid_t *p);
q16_t esp_foc_pid_update(esp_foc_pid_t *p, q16_t sp, q16_t meas);
void esp_foc_pid_set_applied(esp_foc_pid_t *p, q16_t u_applied);
void esp_foc_pid_set_ff(esp_foc_pid_t *p, q16_t ff);
void esp_foc_pid_set_bypass(esp_foc_pid_t *p, bool on);
/* Rotor-frame decoupling in pu of Vdc: vd* = −ω Ls iq, vq* = ω Ls id + ω ψf. */
void esp_foc_pid_set_pmsm_ff(esp_foc_pid_t *pd, esp_foc_pid_t *pq, q16_t we,
                             q16_t id, q16_t iq, q16_t ls, q16_t psi_f,
                             q16_t inv_vdc);
esp_err_t esp_foc_pid_set_kp(esp_foc_pid_t *p, float kp);
esp_err_t esp_foc_pid_set_ki(esp_foc_pid_t *p, float ki);
esp_err_t esp_foc_pid_set_kd(esp_foc_pid_t *p, float kd);

#ifdef __cplusplus
}
#endif
