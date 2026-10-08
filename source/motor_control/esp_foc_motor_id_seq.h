/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "espFoC/motor_control/esp_foc_ident.h"
#include "espFoC/motor_control/esp_foc_motor_id.h"
#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Electrical identification sequence, private to esp_foc_motor_id.
 *
 * It drives actuation and sensing through the ops below and never touches a
 * peripheral; esp_foc_motor_id_run() binds them to the inverter.
 *
 * State order follows the InstaSPIN identification flow, whose dependency chain
 * is what makes it safe: everything that spins the rotor needs a tuned current
 * loop, and tuning needs the impedance, so the standstill probes come first and
 * a failure there stops the run before any gain is designed.
 *
 * Two states are additions. TERMINAL is a winding symmetry gate, the cheapest
 * state in the sequence and the one that refuses a damaged machine up front.
 * ROVERL_FINE re-probes near R/L, where arg(Z) is 45 degrees and the R/L split
 * is least sensitive to residual phase error: on a 1.9 ohm / 500 uH machine one
 * uncompensated degree costs 11% of L at 100 Hz but only 2% at 605 Hz.
 */

/**
 * Actuation and sensing the sequence needs. Runs in thread context, so the
 * blocking probe helpers are deliberate: the caller owns the PWM ISR and the
 * per-sample accumulation, and only hands back solved results.
 *
 * All voltages are volts and all currents amps, both q16. Duty conversion and
 * the pu-of-Vdc domain belong to the caller.
 */
typedef struct {
    void *ctx;
    void (*set_theta)(void *ctx, q16_t theta);
    void (*set_vdq)(void *ctx, q16_t vd, q16_t vq);
    void (*set_idq)(void *ctx, q16_t id, q16_t iq);
    void (*set_fe_hz)(void *ctx, q16_t fe_hz);
    void (*fetch_dq)(void *ctx, q16_t *vd, q16_t *vq, q16_t *id, q16_t *iq);
    /**
     * Run an AC impedance probe to completion with Park forced to @p theta.
     * theta is a parameter so probing the q axis later needs a state, not an
     * API change.
     */
    bool (*probe_z)(void *ctx, q16_t theta, const esp_foc_ident_excite_t *e,
                    esp_foc_ident_z_t *out);
    /** Hold @p vd on axis @p theta; return the settled mean current in mA. */
    bool (*probe_dc)(void *ctx, q16_t theta, q16_t vd, int32_t *i_ma);
    /** Drive one bridge terminal against the other two; mean current in mA. */
    bool (*probe_terminal)(void *ctx, int terminal, q16_t v, int32_t *i_ma);
    /**
     * Step @p vd on axis @p theta and hand back the per-sample response, with
     * @p n_pre quiet samples ahead of the edge. Timing the delay needs the samples
     * themselves, not a mean: this is the one probe whose answer is a shape.
     */
    bool (*probe_step)(void *ctx, q16_t theta, q16_t vd, uint32_t n_pre,
                       q16_t *i_out, uint32_t n);
    void (*apply_gains)(void *ctx, float kp, float ki);
    void (*sleep_ms)(void *ctx, uint32_t ms);
    bool (*faulted)(void *ctx);
    void (*on_phase)(void *ctx, esp_foc_motor_id_phase_t ph);
    /**
     * Optional mechanical rotor. When bound, flux measures pole pairs as
     * round(fe* / fm) and psi from ωe = pp·ωm instead of commanded fe.
     * Standstill may pass pole_pairs=0. NULL keeps the sensorless sequence.
     */
    bool (*fetch_rotor)(void *ctx, q16_t *theta_m, q16_t *omega_m);
} esp_foc_motor_id_seq_ops_t;

typedef struct {
    float vdc;
    uint32_t pwm_hz;
    /** Pole pairs. 0 is allowed when fetch_rotor is bound: flux measures them. */
    int pole_pairs;

    /**
     * Starting AC excitation as a fraction of Vdc. It is only a seed: the
     * amplitude is then searched toward i_probe_target_a, because a level that
     * reads well on a 1.9 ohm winding starves a 40 mH one, whose impedance at
     * the coarse point is over 25 ohm.
     */
    float v_probe_frac;
    float v_probe_frac_max; /**< ceiling for the search, keeps modulation linear */
    float i_probe_target_a; /**< response the search aims for */
    float i_probe_max_a;    /**< abort if a probe response exceeds this */
    float v_dc_probe_frac;  /**< DC seed, superseded once R is known */
    /**
     * Width of the bridge's deadtime dead zone, volts.
     *
     * The AC probes are biased by this plus their own amplitude so the current
     * never changes sign, which is the only way the demodulated response is the
     * linear one. Too large only spends current budget; too small and R reads
     * high while L collapses. Measurable as the zero-current intercept of the
     * two-point DC slope.
     */
    float v_deadzone_v;
    uint32_t coarse_hz;
    uint32_t probe_periods;
    uint32_t settle_periods;
    /** Seed, and the value used outright when skip_lag_cal is set. */
    uint32_t lag_samples;
    int32_t phase_trim_cdeg;

    /**
     * Measure the sense pipeline delay instead of trusting lag_samples.
     *
     * Whether a duty lands on the next carrier period or the one after depends
     * on compare-register shadowing, so the delay is one or two samples and
     * guessing wrong is silent: two instead of one is 10.9 degrees at 605 Hz on
     * a 20 kHz carrier, which lands as roughly a quarter error on L and no error
     * anywhere else. The calibration leans on R being frequency-independent
     * while a lag error is not, because a wrong lag adds phase proportional to
     * omega.
     */
    bool skip_lag_cal;
    int32_t lag_cal_limit_permil;

    int32_t sym_limit_permil;

    /**
     * Skip the two states that inject DC in open loop.
     *
     * Both need a restrained shaft. A DC current vector digs a magnetic well and
     * a free rotor rings in it; with many pole pairs a small mechanical swing is
     * a large electrical one, so the BEMF disturbance is large. On the 13
     * pole-pair bench a 0.72 V injection settling at 160 mA swung the phase
     * current to 1.0-1.5 A, which no averaging window fixes because the machine
     * really is drawing it. The AC probes are immune: a sinusoid on a fixed axis
     * makes no net torque, so the rotor does not move and R, L and the sense
     * delay are all still measured.
     */
    bool skip_terminal;
    bool skip_rs;

    /**
     * Plant from an earlier run, ohm and henry. When both are positive the
     * impedance states are skipped and the gains are designed from these.
     *
     * Re-probing is not free — it is the part of the sequence that pushes
     * current with no torque to show for it — so the flux leg is handed the
     * standstill plant instead of exciting the winding a second time.
     */
    float known_r_loop_ohm;
    float known_ls_h;

    float i_bw_hz;
    float tune_backoff;

    bool do_flux;           /**< false keeps the whole run at standstill */
    float i_flux_a;
    int flux_hz;
    uint32_t ramp_ms;
    uint32_t flux_hold_ms;

    uint32_t dt_ms;
} esp_foc_motor_id_seq_config_t;

typedef struct {
    esp_foc_motor_id_seq_config_t cfg;
    esp_foc_motor_id_seq_ops_t ops;
    esp_foc_motor_id_phase_t phase;
    esp_foc_motor_id_result_t result;
    uint32_t lag_used; /**< resolved delay; cfg stays the caller's input */
    int32_t trim_used; /**< resolved sub-sample remainder, added to cfg's trim */
} esp_foc_motor_id_seq_t;

/** Config with bench-safe defaults; caller still supplies vdc, pwm_hz and pp. */
void esp_foc_motor_id_seq_default_config(esp_foc_motor_id_seq_config_t *cfg);

/**
 * Run the sequence. Always leaves the outputs at zero, including on failure.
 * Inspect @c result.valid_mask for which parameters may be used and
 * @c result.failed_at for where a partial run stopped.
 */
esp_err_t esp_foc_motor_id_seq_run(esp_foc_motor_id_seq_t *s);

/**
 * Current-loop gains for a given plant, the same path TUNE_I takes. Also what
 * the sensored stages design their spinning current loop with.
 */
esp_err_t esp_foc_motor_id_seq_gains_for(const esp_foc_motor_id_seq_config_t *cfg,
                                         float r_ohm, float l_h,
                                         float *kp, float *ki);

#ifdef __cplusplus
}
#endif
