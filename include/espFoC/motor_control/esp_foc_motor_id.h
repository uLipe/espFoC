/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "espFoC/drivers/esp_foc_inverter.h"
#include "espFoC/drivers/esp_foc_rotor_sensor.h"
#include "espFoC/motor_control/esp_foc_ident.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Machine identification, self-contained.
 *
 * esp_foc_motor_id_run() takes the inverter, an optional rotor sensor and the
 * two numbers no measurement can supply (pole pairs, Vdc), and hands back the
 * plant. For the duration of the call it owns the inverter's PWM, DMA and fault
 * callbacks, the task the sequence runs on and, with a rotor, one task that
 * reads it; all are released on every return. The FoC core does not depend on
 * it.
 *
 * Two modes, picked by whether a rotor is given:
 *
 * - **Sensorless** (rotor NULL): standstill impedance probes (R, L, sense
 *   delay, dead zone, current-loop gains), then an I-f flux leg on the known
 *   plant for psi_f. Everything a sensorless observer needs.
 * - **Sensored**: the same electrical stages, then on the encoder angle a +Iq
 *   direction check, the mechanical plant (K, J), a Park-angle probe that fits
 *   the encoder offset and latency (e = d0 + tau·ω) under an internal speed
 *   hold, and the mechanical plant again on the compensated angle, since K
 *   scales with cos of the Park error. What the application does with angle
 *   and speed afterwards is its own business.
 *
 * Each electrical leg is retried and checked for plausibility inside the
 * block. In sensored mode a failed flux leg is not fatal: psi_f is then taken
 * from the back-EMF the angle probe reads on the encoder frame.
 *
 * # Discover the phase map first
 *
 * Run esp_foc_phase_discover before this, every time the motor is unplugged
 * and reconnected. Every measurement here is taken in the dq frame the map
 * defines: a wrong permutation puts the excitation somewhere other than the
 * axis it names, and the impedance comes back plausible and wrong. This bench
 * read 8.3 ohm for a 2.2 ohm winding that way. With a rotor, the encoder zero
 * and direction it sets are what the sensored stages spin on.
 *
 * # Thermal drift
 *
 * Rs is a temperature reading as much as a machine constant. Back-to-back runs
 * on this bench walked it from 2.43 to 3.18 ohm in a couple of minutes — the DC
 * bias the probes ride on is what heats the winding. Identify from cold.
 */
typedef enum {
    ESP_FOC_MOTOR_ID_IDLE = 0,
    ESP_FOC_MOTOR_ID_BIAS,
    ESP_FOC_MOTOR_ID_TERMINAL,
    ESP_FOC_MOTOR_ID_ROVERL_COARSE,
    ESP_FOC_MOTOR_ID_LAGCAL,
    ESP_FOC_MOTOR_ID_ROVERL_FINE,
    ESP_FOC_MOTOR_ID_TUNE_I,
    ESP_FOC_MOTOR_ID_RS,
    ESP_FOC_MOTOR_ID_RAMPUP,
    ESP_FOC_MOTOR_ID_RATED_FLUX,
    ESP_FOC_MOTOR_ID_RAMPDOWN,
    ESP_FOC_MOTOR_ID_DIRECTION,
    ESP_FOC_MOTOR_ID_MECH,
    ESP_FOC_MOTOR_ID_PARK_ANGLE,
    ESP_FOC_MOTOR_ID_DONE,
    ESP_FOC_MOTOR_ID_FAIL,
} esp_foc_motor_id_phase_t;

#define ESP_FOC_MOTOR_ID_VALID_SYMMETRY (1u << 0)
#define ESP_FOC_MOTOR_ID_VALID_R_LOOP   (1u << 1)
#define ESP_FOC_MOTOR_ID_VALID_LS       (1u << 2)
#define ESP_FOC_MOTOR_ID_VALID_RS       (1u << 3)
#define ESP_FOC_MOTOR_ID_VALID_GAINS    (1u << 4)
#define ESP_FOC_MOTOR_ID_VALID_PSI_F    (1u << 5)
#define ESP_FOC_MOTOR_ID_VALID_PP       (1u << 6)
#define ESP_FOC_MOTOR_ID_VALID_DIR      (1u << 7)
#define ESP_FOC_MOTOR_ID_VALID_K        (1u << 8)
#define ESP_FOC_MOTOR_ID_VALID_J        (1u << 9)
#define ESP_FOC_MOTOR_ID_VALID_PARK     (1u << 10)

typedef struct {
    /** DC resistance from the two-point slope, with the bridge's fixed
     *  deadtime/Vds offset removed. */
    float rs_ohm;
    /** Series resistance at the probe frequency, which is the plant the current
     *  loop operates against and therefore what the gains are designed from. */
    float r_loop_ohm;
    float ls_h;
    float roverl_rad_s;
    float psi_f_wb;
    /** Mechanical Hz and electrical rad/s at the flux hold (0 if no rotor). */
    float flux_fm_hz;
    float flux_we_rad_s;
    float kp;
    float ki;
    float probe_hz;
    /** Bridge dead zone from the DC intercept, volts; 0 if Rs was not measured. */
    float v_deadzone_v;
    float symmetry_spread;
    int pole_pairs;

    int32_t terminal_ma[3];
    int32_t probe_phase_cdeg;
    int32_t probe_i_ma;
    int32_t probe_v_mv;     /**< excitation the amplitude search settled on */

    /** The coarse probe, kept because it is the anchor the delay sweep and the
     *  fine frequency are both derived from: a refusal downstream is only
     *  readable next to it. */
    esp_foc_ident_z_t coarse;

    /** Delay the run actually used. */
    uint32_t lag_samples;
    /** Sub-sample remainder the sweep solved for, centi-degrees at the fine point. */
    int32_t phase_trim_cdeg;
    /**
     * Per-candidate R disagreement between the coarse and fine probe, in permil,
     * or -1 for a candidate whose probe did not resolve.
     */
    int32_t lag_r_permil[ESP_FOC_IDENT_LAG_MAX];

    /* Sensored stages; zero without a rotor. */
    /** Mechanical Hz the shaft reached under the +Iq direction check. */
    float dir_fm_hz;
    /** dω_e/dt per amp of measured iq, on the compensated angle when it applied. */
    float k_rad_s2_per_a;
    /** K on the encoder angle as the phase map left it. */
    float k_raw_rad_s2_per_a;
    float j_kgm2;
    /** iq that holds the speed the mechanical stage ran at. */
    float drag_a;
    /** Add to the encoder θe: θ_park = θe + park_offset_rad + ω_e·park_lead_s. */
    float park_offset_rad;
    float park_lead_s;
    /** Residual of the last angle fit, rad. */
    float park_fit_rms_rad;

    uint32_t valid_mask;
    esp_foc_motor_id_phase_t failed_at;
} esp_foc_motor_id_result_t;

typedef enum {
    /** A phase was entered; @c phase. */
    ESP_FOC_MOTOR_ID_EV_PHASE = 0,
    /** An electrical leg ended; @c pass 0 standstill / 1 flux, @c attempt,
     *  @c err, @c result as the sequence left it. */
    ESP_FOC_MOTOR_ID_EV_ATTEMPT,
    /** v[0] fm Hz under +Iq, v[1] the iq that produced it. */
    ESP_FOC_MOTOR_ID_EV_DIRECTION,
    /** @c pass 0 raw / 1 compensated, @c err; v[0] K, v[1] drag A,
     *  v[2] ω_e Hz at entry, v[3] J. */
    ESP_FOC_MOTOR_ID_EV_MECH,
    /** Speed hold designed for the angle probe; v[0] feedback sigma rad/s,
     *  v[1] bw Hz, v[2] Kp A/(rad/s), v[3] Ki. */
    ESP_FOC_MOTOR_ID_EV_HOLD,
    /** @c pass, v[0] ω_e Hz, v[1] Park error rad, v[2] psi Wb, v[3] iq A. */
    ESP_FOC_MOTOR_ID_EV_ANGLE_POINT,
    /** @c pass, @c err (ESP_OK applied, ESP_ERR_INVALID_RESPONSE refused);
     *  v[0] d0 rad, v[1] tau s, v[2] fit rms rad (this pass), v[3] psi Wb. */
    ESP_FOC_MOTOR_ID_EV_ANGLE_FIT,
} esp_foc_motor_id_ev_t;

typedef struct {
    esp_foc_motor_id_ev_t ev;
    esp_foc_motor_id_phase_t phase;
    uint8_t pass;
    uint8_t attempt;
    esp_err_t err;
    float v[4];
    const esp_foc_motor_id_result_t *result;
} esp_foc_motor_id_event_t;

typedef struct {
    /** Rate the rotor is fetched at; must divide the PWM rate. */
    uint32_t fetch_hz;
    float pll_bw_hz;
    float pll_zeta;
    /** Current-loop bandwidth while spinning on the encoder. */
    float i_bw_hz;
    float iq_dir_a;
    uint32_t dir_ramp_ms;
    float fm_min_hz;
    /** iq ceiling of the mechanical stage and of the speed hold. */
    float i_max_a;
    float k_max;
    /** Fallback K the speed hold is designed on if the raw stage refuses. */
    float k_fallback;
    /** |ω_e| band the mechanical stage enters at, Hz. */
    float band_lo_hz;
    float band_hi_hz;
    float nudge_a;
    float still_hz;
    float hold_zeta;
    /** Hold bandwidth = frac · Vdc/(√3·2π·psi), clamped. */
    float hold_bw_frac;
    float hold_bw_min_hz;
    float hold_bw_max_hz;
    /** iq one sigma of feedback noise may command; the noise ceiling. */
    float fb_iq_budget_a;
    /** PLL pole (bw/2) over the hold crossover; the phase ceiling. */
    float pll_sep_min;
    uint32_t ripple_ms;
    /** false stops after the raw mechanical stage. */
    bool angle_comp;
    /** Probe points are ±{1,2,3,4}·this, electrical Hz. */
    int probe_base_hz;
    uint32_t probe_step_ms;
    uint32_t probe_settle_ms;
    uint32_t probe_avg_ms;
    float d0_max_rad;
    float tau_max_s;
    /** Inverse-Park lead for the PWM update delay, in PWM periods. */
    float pwm_delay_ts;
    float overspeed_hz;
} esp_foc_motor_id_sensored_config_t;

typedef struct {
    /** Required. With a rotor, also the pole pairs its θe is reported for. */
    int pole_pairs;
    /** Required, volts. */
    float vdc;

    /* Electrical stages. */
    float i_probe_target_a;
    float i_probe_max_a;
    /** Guard average a probe window is discarded at; keep it under the trip. */
    float i_abort_a;
    float v_deadzone_v;
    int32_t sym_limit_permil;
    uint32_t probe_periods;
    uint32_t settle_periods;
    /** Skip the terminal symmetry gate, which needs a restrained shaft. */
    bool skip_terminal;
    float i_bw_hz;
    float tune_backoff;
    float i_flux_a;
    int flux_hz;
    /** Plausibility windows; loose, the dq plant carries the bridge and wiring. */
    float r_min_ohm;
    float r_max_ohm;
    float l_min_h;
    float l_max_h;
    float psi_min_wb;
    float psi_max_wb;
    uint8_t tries;
    /** Rest before each standstill attempt: BEMF from a swinging rotor spoils
     *  the AC probes. */
    uint32_t settle_ms;
    /** Bridge off after every attempt. */
    uint32_t coast_ms;
    /** Extra rest after a failed one: the probe bias heats the winding. */
    uint32_t retry_ms;
    int cal_rounds;

    /* Sensored stages; read only with a rotor. */
    esp_foc_motor_id_sensored_config_t sensored;

    /** Optional. Called on the identification task, whose stack is
     *  CONFIG_ESP_FOC_MOTOR_ID_TASK_STACK, not on the caller's. */
    void (*on_event)(void *ctx, const esp_foc_motor_id_event_t *ev);
    void *ctx;
} esp_foc_motor_id_config_t;

/** Defaults from Kconfig; the caller still supplies pole_pairs and vdc. */
void esp_foc_motor_id_default_config(esp_foc_motor_id_config_t *cfg);

/**
 * Identify the machine behind @p inv. Blocking, task context only.
 *
 * The sequence runs on its own task while the caller sleeps until it ends, so
 * the caller's stack only holds this frame. Installs its own inverter
 * callbacks and, with @p rotor, a rotor task; tasks and callbacks are removed
 * and the bridge disabled on every return. One identification at a time: a
 * second concurrent call gets ESP_ERR_INVALID_STATE.
 *
 * @p out is filled as far as the run got: check @c valid_mask, and
 * @c failed_at on error.
 */
esp_err_t esp_foc_motor_id_run(esp_foc_inverter_t *inv,
                               esp_foc_rotor_sensor_t *rotor,
                               const esp_foc_motor_id_config_t *cfg,
                               esp_foc_motor_id_result_t *out);

const char *esp_foc_motor_id_phase_name(esp_foc_motor_id_phase_t ph);

#ifdef __cplusplus
}
#endif
