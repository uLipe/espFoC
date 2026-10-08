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

/**
 * Open-loop lock-in spin: align the rotor, then drag it up to a plateau.
 *
 * Iq=0 throughout. The two stages do not overlap, and that is the whole point:
 *   ALIGN        ω=0, Id → i_align_a in align_ms
 *   ALIGN_SETTLE ω=0, Id held until the rotor has stopped moving
 *   (lead_rad)   θ_ol steps ahead of the parked rotor, once
 *   VF_BREAKAWAY ω → 2π·vf_hz on a *voltage* command, current loop out of the
 *                path (optional, needs set_vdq)
 *   CREEP        ω → 2π·f_max at constant accel_rads2 (elec. rad/s²), on current
 *
 * The lead step is not cosmetic. A successful align leaves the rotor exactly on
 * the commanded d axis, which is the *zero torque* point for Id with Iq=0 — so
 * the ramp starts with nothing accelerating the rotor and has to wait for the
 * field to run ahead on its own. On silicon that lost the rotor outright often
 * enough to matter: vq stayed at 0 from 2 Hz to 33 Hz, meaning no BEMF and a
 * shaft that never turned, while the frame swept the whole ramp. Stepping the
 * frame lead_rad ahead of the parked rotor puts Id·sin(lead_rad) of torque on it
 * at the first tick. Same geometry SimpleFOC's open-loop uses by keeping the
 * current on q relative to the aligned position.
 *
 * Slewing Id and ω together (the ODrive-style overlap this module used to do)
 * means the field sweeps while the current that is supposed to capture the rotor
 * is still building: at accel=31 rad/s² and align_ms=400 the field turns 141°
 * electrical before Id is at target, so the rotor is left behind by an unknown
 * angle and the ramp starts from a load angle that can be past pull-out. On
 * silicon that showed up as a rough start, 3 A on the supply and a handoff that
 * locked onto a rotor which was not following. Hold ω at zero until the rotor is
 * actually on the d axis.
 *
 * Park stays on the caller's θ_ol. on_plateau fires at |fe|>=f_max so the caller
 * can start the PLL. lock_ok / do_handoff stay in the caller.
 *
 * ω* does not enter the observer VCO from this module.
 */
typedef enum {
    ESP_FOC_IF_PHASE_ALIGN = 0,
    ESP_FOC_IF_PHASE_ALIGN_SETTLE,
    ESP_FOC_IF_PHASE_VF_BREAKAWAY,
    ESP_FOC_IF_PHASE_CREEP,
    ESP_FOC_IF_PHASE_HANDOFF,
    ESP_FOC_IF_PHASE_DONE,
    ESP_FOC_IF_PHASE_FAIL,
} esp_foc_if_phase_t;

typedef struct {
    void *ctx;
    void (*set_idq)(void *ctx, q16_t id, q16_t iq);
    void (*set_fe_hz)(void *ctx, q16_t fe_hz);
    void (*sleep_ms)(void *ctx, uint32_t ms);
    void (*poll)(void *ctx);
    q16_t (*get_dang)(void *ctx);
    bool (*lock_ok)(void *ctx);
    bool (*do_handoff)(void *ctx);
    void (*lock_timeout)(void *ctx);
    void (*on_plateau)(void *ctx);
    /* Optional. "Has the rotor stopped moving?", asked once per tick while the
     * field is held still. NULL turns ALIGN_SETTLE into a plain dwell of
     * align_settle_ms. The caller owns the witness because this module never
     * sees a measurement — Iq is commanded to zero during the align, so anything
     * the current loop has to reject on q there is the rotor's own BEMF. */
    bool (*at_rest)(void *ctx);
    /* Optional. Asked exactly once, when the align window closes — by settle or
     * by align_timeout_ms, which are not the same outcome. false fails the run
     * before anything rotates, so a caller whose witness says the rotor was never
     * captured can align again instead of ramping it. NULL = always proceed. */
    bool (*align_ok)(void *ctx);
    /* Optional. Asked once per step of the current-mode ramp. false aborts the
     * run there. Dragging the frame up while the caller's current loop is railed
     * and measuring nothing only delivers a plateau with no flux in it, and the
     * observer is then asked to lock onto an angle that does not exist.
     * NULL = never abort. */
    bool (*ramp_ok)(void *ctx);
    /* Optional, required for cfg.lead_rad to do anything. Step the caller's open
     * loop angle by d_rad; called exactly once, between the align and the ramp.
     * This module does not own θ_ol, so it cannot apply the lead itself. */
    void (*advance_theta)(void *ctx, q16_t d_rad);
    /* Required if cfg.vf_hz is set. Drive the bridge from a voltage command,
     * per-unit of Vdc, with the current loop out of the path. */
    void (*set_vdq)(void *ctx, q16_t vd, q16_t vq);
    /* Optional. Called once when the V/f stage hands back, so the caller can seed
     * its current PIs from the voltage they are about to inherit — otherwise the
     * loop resumes from a stale integrator and steps the bridge. */
    void (*to_current_mode)(void *ctx);
} esp_foc_if_ops_t;

typedef struct {
    float i_align_a;
    uint32_t align_ms;
    float i_cycle_a;
    int f_cycle_hz;
    float cycle_revs;
    uint32_t cycle_accel_ms;
    uint32_t cycle_timeout_ms;
    float i_creep_a;
    int i_hold_hz;
    int f_min_hz;
    int f_max_hz;
    uint32_t creep_ramp_ms;
    uint32_t lock_timeout_ms;
    float delta_pause_rad;
    int delta_min_hz;
    uint32_t dt_ms;
    /* Electrical rad/s². 0 = 2π·f_max / (creep_ramp_ms/1000). */
    float accel_rads2;
    /* How long ops.at_rest has to keep saying yes, consecutively, before the ω
     * ramp is allowed to start. 0 = start as soon as Id is at target. With
     * at_rest NULL this is a plain dwell. It is a persistence requirement on a
     * measurement, not a plant time constant, which is why it is allowed to be a
     * literal where a bandwidth would not be. */
    uint32_t align_settle_ms;
    /* Cap on the ALIGN_SETTLE wait — the Id slew is already bounded by align_ms
     * and is never cut short. On expiry the ramp starts anyway: a rotor that
     * will not settle is the caller's problem to gate at handoff, and standing
     * here with current in the winding is worse. Required when at_rest is set;
     * 0 means no cap. */
    uint32_t align_timeout_ms;
    /* Electrical radians the frame steps ahead of the parked rotor before the
     * ramp, applied with the sign of the run. π/2 is maximum torque per amp; less
     * trades torque for a smaller entry swing, since the rotor snaps toward the
     * new frame with an energy that goes as (1-cos(lead_rad)). 0 = no lead, which
     * is the zero-torque start described above. */
    float lead_rad;
    /*
     * Voltage-mode break-away. 0 = off, and then none of the vf_* fields matter.
     *
     * vf_hz is where the current loop takes over, and it is a safety limit rather
     * than a tuning knob: the applied voltage on a rotor that refuses to move goes
     * straight into boost/R, so the crossover has to sit low enough that a stall
     * is a survivable current. Must be below f_max_hz.
     *
     * The law is v = vf_boost + vf_per_hz·fe, per-unit of Vdc, applied on q. Size
     * vf_boost from the *measured* bridge dead zone plus Rs·i_align_a, and
     * vf_per_hz from 2π·ψf/Vdc so the ramp tracks the BEMF it is about to create.
     */
    float vf_hz;
    uint32_t vf_ramp_ms;
    float vf_boost;
    float vf_per_hz;
} esp_foc_if_config_t;

typedef struct {
    esp_foc_if_config_t cfg;
    esp_foc_if_ops_t ops;
    esp_foc_if_phase_t phase;
    q16_t fe_hz;
    q16_t id;
    q16_t iq;
} esp_foc_if_t;

esp_err_t esp_foc_if_run(esp_foc_if_t *s, int sign);

#ifdef __cplusplus
}
#endif
