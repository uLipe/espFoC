/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include "esp_err.h"
#include "espFoC/motor_control/esp_foc_observer.h"
#include "espFoC/utils/esp_foc_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Voltage-model stator-flux observer in αβ + tracking PLL on rotor flux.
 *
 *   dψ̂_s/dt = v − Rs i − λ (ψ̂_s − Ls i)
 *   ψ̂_r     = ψ̂_s − Ls i
 *   ê       = ω̂ × ψ̂_r
 *
 * λ is a washout on ψ̂_r (DC reject). Do not restore toward ψf∠θ̂: that
 * copies the PLL angle and, once Park uses θ̂, the estimate tracks itself.
 *
 * Tuning (Hz / ζ, not raw gains) — NXP MCAT / AN12214 naming:
 *
 *   obs_bw_hz     LPF on ψ̂_r. Above current-loop BW, below PWM/20.
 *                 Start 2…4× current-loop BW (this HIL: 800–1200 Hz).
 *   obs_zeta      unused placeholder for a future Luenberger i-loop; keep 0.7.
 *   track_bw_hz   PLL wn/2π on ψ̂_r, designed once and frozen. Must
 *                 track Iq steps (accel and dump), not only slow I-f
 *                 ramps. Below obs_bw, well under PWM/10 (Nyquist stack).
 *   track_zeta    PLL damping. 0.7.
 *   blend_hz      λ/2π washout. 10–30 Hz (I-f / handoff). Voltage model
 *                 dominates the rotating component above this.
 *   psi_lock_frac lock when |ψ̂_r| ≥ frac·ψf. 0.5…0.8.
 *
 * Gains: washout λ=2π·blend_hz. PLL is a discrete type-2 PI on the VCO
 * (esp_foc_pid_design_integrator, Tustin). Do not jump BW at catch.
 *
 * θ̂ and ω̂ are electrical. ω* does not enter the VCO.
 */
#define ESP_FOC_OBSERVER_PSI_MAX_FRAC_DEFAULT 4.0f

typedef struct {
    float rs_ohm;
    float ls_h;
    float psi_f_wb;
    float ts_s;
    float obs_bw_hz;
    float obs_zeta;
    float track_bw_hz;
    float track_zeta;
    float blend_hz;
    float psi_lock_frac;
    /* Ceiling on |ψ̂_r| as a multiple of ψf. The magnet cannot change, so a
     * larger estimate is integrator drift, and drift here is self-sustaining:
     * the PLL error is divided by |ψ̂_r|, so an inflated estimate collapses
     * loop gain until Park slips and the saturated current loops feed more DC
     * back into the integrand. 4.0 is far above anything a real ψ̂_r reaches and
     * well below an observed runaway.
     *
     * 0 means that default, not "no ceiling": leaving the field unset is what
     * every caller but one did, so the guard existed and was off everywhere it
     * mattered. Pass a negative value to opt out deliberately. */
    float psi_max_frac;
    float w_max_rads;
    uint16_t lock_count;
    uint16_t unlock_count;
    /* Hold ω̂ at the seed for this many ms after first ψ (PI off).
     * 0 = tracking PI on immediately. Do not snap θ̂; that jump rings the VCO. */
    float pll_settle_ms;
} esp_foc_observer_flux_config_t;

typedef struct {
    esp_foc_observer_t iface;
    q16_t rs;
    q16_t ls_mh;
    q16_t psi_f_mwb;
    q16_t inv_ts;
    q16_t lambda_ts;
    q16_t lpf_b0;
    esp_foc_pid_t pll_pi;
    q16_t w_max;
    q16_t psi_lock_min;
    /* Ceiling on |ψ̂_r|, compared against max+0.4142·min. L1 would be cheaper
     * still but it is [1, √2] off L2 depending on the vector's angle, so a
     * clamped ψ̂ would carry a 29% magnitude ripple as it rotates. This
     * approximation is within ~4% and costs one multiply. */
    q16_t psi_max_mag;
    /* Envelope of |ψ̂_r| since the last reset, same approximation. Sets the
     * ceiling from measurement instead of taste: the gap between what healthy
     * runs peak at and what a drifting one reaches is the whole margin. */
    q16_t psi_peak;
    uint32_t psi_clamp_count;
    q16_t psi_s_a;
    q16_t psi_s_b;
    q16_t psi_r_a;
    q16_t psi_r_b;
    q16_t psi_f_a;
    q16_t psi_f_b;
    q16_t e_a;
    q16_t e_b;
    q16_t theta;
    q16_t omega;
    q16_t i_a;
    q16_t i_b;
    q16_t theta_psi;
    q16_t w_atan_f;
    q16_t w_lpf_b0;
    q16_t phase_err;
    uint16_t lock_need;
    uint16_t unlock_need;
    uint16_t lock_acc;
    uint16_t unlock_acc;
    uint16_t pll_settle_need;
    uint16_t pll_settle_left;
    bool locked;
    bool have_psi;
    bool pll_enable;
    bool have_theta_psi;
    esp_foc_angle_extract_t extract;
} esp_foc_observer_flux_t;

esp_err_t esp_foc_observer_flux_init(esp_foc_observer_flux_t *o,
                                     const esp_foc_observer_flux_config_t *cfg);

/* Recompute washout + PLL gains without resetting ψ̂/θ̂. For design
 * iteration only — do not jump BW at runtime on a spinning plant. */
esp_err_t esp_foc_observer_flux_set_bw(esp_foc_observer_flux_t *o,
                                       float obs_bw_hz,
                                       float track_bw_hz,
                                       float track_zeta);

/* PLL VCO ω̂. Diagnostic: d/dt atan2(ψ̂_r) is get_omega_psi(). */
q16_t esp_foc_observer_flux_get_omega_psi(const esp_foc_observer_flux_t *o);
uint16_t esp_foc_observer_flux_get_pll_settle_left(const esp_foc_observer_flux_t *o);

/* Ticks the |ψ̂_r| ceiling had to pull back. Nonzero means the voltage
 * integrator was drifting, so treat the run as suspect even if it locked. */
uint32_t esp_foc_observer_flux_get_psi_clamp_count(const esp_foc_observer_flux_t *o);

/* Peak |ψ̂_r| since reset, in Wb. */
q16_t esp_foc_observer_flux_get_psi_peak(const esp_foc_observer_flux_t *o);

#ifdef __cplusplus
}
#endif
