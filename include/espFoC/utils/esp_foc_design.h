/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdint.h>

#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Design-time helpers shared by the stacks: float in, fixed point or float
 * out. Not for the control interrupt.
 */

#define ESP_FOC_Q32_ONE 4294967296.0

/** Rounded to nearest, halves away from zero. */
int64_t esp_foc_q32_from_double(double x);
int64_t esp_foc_q16_from_double(double x);

/** Per-tick step of a rate given per second, at least one LSB. */
q16_t esp_foc_step_q16(float per_s, float tick_hz);

/** Electrical speed [Hz] where the back-EMF reaches the linear SVPWM limit Vdc/√3. */
float esp_foc_base_speed_hz(float vdc_v, float psi_wb);

/**
 * Speed-loop bandwidth ceilings for a type-2 PI with crossover 2ζ·bw:
 * noise — Kp·ripple stays within iq_budget_a on a plant of gain k_rads2_a;
 * phase — the crossover stays sep_min below half the speed PLL bandwidth.
 */
float esp_foc_speed_bw_noise_hz(float iq_budget_a, float ripple_rads, float k_rads2_a,
                                float zeta);
float esp_foc_speed_bw_phase_hz(float pll_bw_hz, float sep_min, float zeta);

#ifdef __cplusplus
}
#endif
