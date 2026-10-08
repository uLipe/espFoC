/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "sdkconfig.h"
#include "esp_err.h"
#include "espFoC/drivers/esp_foc_inverter.h"
#include "espFoC/drivers/esp_foc_rotor_sensor.h"
#include "espFoC/motor_control/esp_foc_phase_discover.h"
#include "espFoC/utils/esp_foc_q16.h"

#if CONFIG_ESP_FOC_ENABLE_MOTOR_ID
#include "espFoC/motor_control/esp_foc_motor_id.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Sensored FoC stack, up to CONFIG_ESP_FOC_SD_MAX_AXES axes.
 *
 * Each axis is a statically allocated instance selected by cfg.axis at init()
 * and by the axis argument of every other call. An axis owns its inverter's
 * TEZ, DMA and fault callbacks from init() to deinit(), its rotor sensor, and
 * runs its own supervisor task. Park rides the encoder angle, compensated by
 * the identified offset and speed lead.
 *
 * Execution contexts:
 *  - TEZ (every PWM period): sensor step, Park angle from the encoder plus
 *    offset and lead, Clarke, Park, current PI d/q plus the user vdq
 *    feedforward, vlim, inverse Park rotated by the PWM delay, SVM. Every
 *    PWM/fetch_hz periods it wakes the supervisor for an encoder fetch.
 *  - Slot (inside the TEZ, once per fresh encoder sample): position P, speed
 *    PI, cogging feedforward by mechanical angle and the guards.
 *  - Supervisor task: encoder fetch and the PLL update that consumes it, the
 *    unwrapped position, the state machine, run/stop/learn requests and every
 *    event callback. Callbacks share the thread that feeds the encoder: keep
 *    them short, a long one shows up as a stale sensor.
 *
 * Control level (cfg.control) is nested: TORQUE < VELOCITY < POSITION. The
 * axis may switch mode at or under its level; the setters of a level above
 * the active mode return ESP_ERR_INVALID_STATE.
 *
 * run() enables the bridge, calibrates the current offsets and, above TORQUE,
 * waits for a still rotor, measures the speed feedback ripple and designs the
 * speed PI on it before RUNNING. ABORT and FAULT latch until clear_fault().
 *
 * Units: speed references are electrical Hz, positions mechanical radians
 * from the origin, joint speed mechanical rad/s.
 */

typedef enum {
    ESP_FOC_SD_CONTROL_TORQUE = 0,
    ESP_FOC_SD_CONTROL_VELOCITY,
    ESP_FOC_SD_CONTROL_POSITION,
} esp_foc_sensored_control_t;

typedef enum {
    ESP_FOC_SD_MODE_TORQUE = 0,
    ESP_FOC_SD_MODE_VELOCITY,
    ESP_FOC_SD_MODE_POSITION,
} esp_foc_sensored_mode_t;

typedef enum {
    ESP_FOC_SD_STATE_IDLE = 0,
    ESP_FOC_SD_STATE_ARMED,       /* bridge on, current loop at zero, speed PI design */
    ESP_FOC_SD_STATE_RUNNING,
    ESP_FOC_SD_STATE_LEARNING,    /* cogging sweep, position mode */
    ESP_FOC_SD_STATE_FAULT,
} esp_foc_sensored_state_t;

typedef enum {
    ESP_FOC_SD_EV_ARMED = 0,
    ESP_FOC_SD_EV_RUNNING,        /* mode */
    ESP_FOC_SD_EV_MODE,           /* mode */
    ESP_FOC_SD_EV_CUT,
    ESP_FOC_SD_EV_STOPPED,
    ESP_FOC_SD_EV_RUN_FAIL,       /* fail */
    ESP_FOC_SD_EV_LEARN_PASS,     /* pass, change_rms_a */
    ESP_FOC_SD_EV_LEARN_DONE,     /* pass, change_rms_a */
    ESP_FOC_SD_EV_LEARN_FAIL,     /* pass */
    ESP_FOC_SD_EV_ABORT,          /* abort */
    ESP_FOC_SD_EV_FAULT,          /* fault */
    ESP_FOC_SD_EV_FAULT_CLEARED,
} esp_foc_sensored_ev_t;

typedef enum {
    ESP_FOC_SD_FAIL_NONE = 0,
    ESP_FOC_SD_FAIL_ENABLE,       /* inverter refused enable() */
    ESP_FOC_SD_FAIL_NOT_STILL,    /* rotor never held under still_hz */
    ESP_FOC_SD_FAIL_DESIGN,       /* speed PI design refused the plant */
} esp_foc_sensored_fail_t;

typedef enum {
    ESP_FOC_SD_ABORT_NONE = 0,
    ESP_FOC_SD_ABORT_OVERSPEED,
    ESP_FOC_SD_ABORT_SENSOR_STALE, /* no fresh encoder sample for sensor_stale_slots */
    ESP_FOC_SD_ABORT_SENSOR_FAIL,  /* sensor_fail_max fetch errors in a row */
} esp_foc_sensored_abort_t;

typedef struct {
    esp_foc_sensored_ev_t ev;
    uint8_t axis;
    esp_foc_sensored_state_t state;
    esp_foc_sensored_mode_t mode;
    esp_foc_sensored_fail_t fail;
    esp_foc_sensored_abort_t abort;
    esp_foc_fault_reason_t fault;
    uint8_t pass;
    float change_rms_a;
} esp_foc_sensored_event_t;

typedef void (*esp_foc_sensored_event_cb_t)(void *ctx, const esp_foc_sensored_event_t *e);

/*
 * Fields marked "0 = derive" are designed from the plant; default_config()
 * leaves them 0 and fills the rest from Kconfig.
 */
typedef struct {
    uint8_t axis;                 /* instance, below CONFIG_ESP_FOC_SD_MAX_AXES */
    esp_foc_sensored_control_t control;

    float rs_ohm;
    float ls_h;
    float psi_wb;
    uint8_t pole_pairs;
    float k_rads2_a;              /* dω_e/dt per amp of iq; required above TORQUE */
    float j_kgm2;                 /* informative */

    /* θ_park = θe + park_offset_rad + ω_e·park_lead_s; inverse Park leads
     * a further ω_e·pwm_delay_ts PWM periods. */
    float park_offset_rad;
    float park_lead_s;
    float pwm_delay_ts;

    uint32_t fetch_hz;            /* encoder rate; must divide the PWM rate */
    float pll_bw_hz;
    float pll_zeta;

    float i_max_a;                /* |iq| ceiling of the speed loop and the setters */
    float i_bw_hz;
    float i_tune_backoff;         /* scales the IMC-zoh gains */
    float kp_i;                   /* 0 = IMC-zoh from R, L (pu of Vdc per A) */
    float ki_i;

    /* Speed PI at fetch_hz. bw = min(want, noise ceiling, phase ceiling):
     * want is speed_bw_hz or speed_bw_frac·f_base clamped to [min, max];
     * noise keeps Kp·sigma under fb_iq_budget_a; phase keeps the crossover
     * pll_sep_min under the PLL pole. */
    float speed_bw_hz;            /* 0 = derive */
    float speed_bw_frac;
    float speed_bw_min_hz;
    float speed_bw_max_hz;
    float speed_zeta;
    float fb_iq_budget_a;
    float pll_sep_min;
    uint32_t ripple_ms;           /* feedback ripple window at rest */
    float still_hz;               /* |ω_e| that counts as rest before the ripple */
    uint32_t still_timeout_ms;
    float kp_w;                   /* 0 = derive (A per rad/s electrical) */
    float ki_w;
    float wref_slew_hz_s;

    /* Position P: Kp = 2π·fc_speed/pos_sep unless kp_pos is set. */
    float pos_sep;
    float kp_pos;                 /* 0 = derive, 1/s */
    float corr_max_hz;            /* |P correction|, mechanical Hz */
    float wm_max_hz;              /* |ω*| ceiling, mechanical Hz */
    float inpos_rad;
    float inpos_w_hz;             /* electrical Hz */
    uint32_t inpos_ms;

    struct {
        float overspeed_hz;       /* |ω_e| trip, electrical Hz */
        uint32_t overspeed_hold_ms;
        uint32_t sensor_stale_slots;
        uint32_t sensor_fail_max;
    } guard;

    struct {
        float sweep_hz;           /* mechanical */
        float revs;
        uint32_t passes_max;
        float conv_a;             /* table change RMS that ends the learn, from pass 2 */
        uint32_t smooth;          /* box filter width, bins (odd) */
        uint32_t skip_ms;         /* sweep start not mapped */
        uint32_t rest_ms;
    } cogging;

    esp_foc_phase_map_t map;
    bool map_valid;

    esp_foc_sensored_event_cb_t on_event;
    void *ctx;
} esp_foc_sensored_config_t;

typedef struct {
    esp_foc_sensored_state_t state;
    esp_foc_sensored_mode_t mode;
    float theta_e_rad;            /* Park angle */
    float we_rads;                /* PLL, electrical */
    float w_ref_rads;             /* speed PI reference, electrical */
    float theta_m_rad;            /* unwrapped, from the origin */
    float theta_ref_rad;
    float w_ff_rads;              /* joint speed feedforward, mechanical */
    float id_a;
    float iq_a;
    float id_ref_a;
    float iq_ref_a;
    float vd_v;
    float vq_v;
    float vdc_v;
    bool inpos;
    uint32_t inpos_toggles;
    bool cogging_on;
    uint32_t tez;
    uint32_t fetch_n;
    uint32_t fetch_fail;
    esp_foc_sensored_fail_t fail;
    esp_foc_sensored_abort_t abort;
    esp_foc_fault_reason_t fault;
} esp_foc_sensored_status_t;

/*
 * Speed-loop window since the last read, sampled per fetch: what a trace
 * line or a tracking witness is built from. Read-and-rearm, one reader.
 */
typedef struct {
    uint32_t n;
    uint32_t t_ms;                /* at the read */
    float w_ref_hz;               /* last, electrical */
    float w_mean_hz;
    float werr_sum_hz;            /* sums of w - w_ref, for merging windows */
    float werr_sq_hz2;
    float werr_min_hz;
    float werr_max_hz;
    uint32_t werr_cross;          /* crossings of the previous window's mean */
    float iq_ref_mean_a;
    float iq_ref_min_a;
    float iq_ref_max_a;
    float iq_mean_a;
} esp_foc_sensored_window_t;

typedef struct {
    float kp_i;
    float ki_i;
    float f_base_hz;
    float ripple_rads;            /* feedback sigma at rest, electrical */
    float speed_bw_want_hz;
    float speed_bw_noise_hz;
    float speed_bw_phase_hz;
    float speed_bw_hz;
    float speed_fc_hz;
    float kp_w;
    float ki_w;
    float kp_pos;
    float slot_hz;
} esp_foc_sensored_tuning_t;

typedef struct {
    uint32_t passes;
    bool converged;
    float change_rms_a;           /* last pass against the one before */
    float p2p_a;
    uint32_t bins;
    uint32_t bins_mapped;         /* mapped both ways on the last pass */
} esp_foc_sensored_cogging_info_t;

void esp_foc_sensored_default_config(esp_foc_sensored_config_t *cfg);

#if CONFIG_ESP_FOC_ENABLE_MOTOR_ID
/* Copies what valid_mask marks valid: R (loop value first), L, psi, pole
 * pairs, K, J, and the Park offset and lead. */
void esp_foc_sensored_config_from_motor_id(esp_foc_sensored_config_t *cfg,
                                           const esp_foc_motor_id_result_t *r);
#endif

void esp_foc_sensored_config_from_phase_map(esp_foc_sensored_config_t *cfg,
                                            const esp_foc_phase_discover_result_t *r);

/**
 * Validates the config, designs the current PI and the PLL, applies the
 * phase map (bridge must be disabled), installs the inverter callbacks and
 * starts the supervisor. Ends in IDLE with the bridge untouched.
 *
 * @return ESP_OK, ESP_ERR_INVALID_ARG (NULL rotor, axis out of range, bad
 *         config), ESP_ERR_INVALID_STATE (axis already initialised),
 *         ESP_ERR_NO_MEM (task), or the design's error.
 */
esp_err_t esp_foc_sensored_init(esp_foc_inverter_t *inv, esp_foc_rotor_sensor_t *rotor,
                                const esp_foc_sensored_config_t *cfg);

/*
 * Calls below act on one axis. An axis out of range is ESP_ERR_INVALID_ARG
 * (IDLE for get_state, no-op for the void calls).
 */

/* Dry cut, stops the supervisor, removes the callbacks. Task context. */
void esp_foc_sensored_deinit(uint8_t axis);

/* IDLE -> ARMED -> RUNNING in the mode of cfg.control. Blocks until RUNNING
 * or the run failed (ESP_FAIL, the RUN_FAIL event names why). */
esp_err_t esp_foc_sensored_run(uint8_t axis);

/* Dry cut from any state, ends in IDLE. */
esp_err_t esp_foc_sensored_stop(uint8_t axis);

/* Clears the stack latch and the inverter's, then FAULT -> IDLE. */
esp_err_t esp_foc_sensored_clear_fault(uint8_t axis);

/* At or under cfg.control, bumpless: the speed integrator takes the iq in
 * flight, the speed reference the measured speed, the position reference the
 * measured position. */
esp_err_t esp_foc_sensored_set_mode(uint8_t axis, esp_foc_sensored_mode_t mode);

/* TORQUE: the iq reference. Above it: added to the speed PI output. */
esp_err_t esp_foc_sensored_set_iq(uint8_t axis, float a);
esp_err_t esp_foc_sensored_set_id(uint8_t axis, float a);
/* Added to the current PI outputs, every mode. */
esp_err_t esp_foc_sensored_set_vdq_ff(uint8_t axis, float vd_v, float vq_v);

/* VELOCITY mode, signed electrical Hz, slewed. */
esp_err_t esp_foc_sensored_set_speed_ref_hz(uint8_t axis, float fe_hz);
esp_err_t esp_foc_sensored_set_speed_slew(uint8_t axis, float hz_per_s);

/* POSITION mode. A position reference zeroes the joint speed feedforward. */
esp_err_t esp_foc_sensored_set_position_ref_rad(uint8_t axis, float theta_rad);
/* The current position becomes 0; the reference moves with it. */
esp_err_t esp_foc_sensored_set_origin(uint8_t axis);
/* Joint sample: position, speed feedforward (mechanical rad/s) and iq
 * feedforward, applied together on the next encoder sample. */
esp_err_t esp_foc_sensored_set_joint(uint8_t axis, float theta_rad, float w_ff_rads,
                                     float iq_ff_a);

esp_err_t esp_foc_sensored_set_current_pi(uint8_t axis, float kp, float ki);
esp_err_t esp_foc_sensored_set_speed_pi(uint8_t axis, float kp, float ki);
/* Redesigns the speed PI on K at this bandwidth, ceilings not applied. */
esp_err_t esp_foc_sensored_set_speed_bw(uint8_t axis, float hz);
esp_err_t esp_foc_sensored_set_position_kp(uint8_t axis, float kp);

/**
 * Learns the cogging feedforward: constant-speed sweeps at ±sweep_hz map the
 * iq the loop needs per mechanical-angle bin; (+map + −map)/2 cancels drag
 * and loop lag, the position-locked torque stays. Each pass re-maps on top of
 * the table, until the change falls under conv_a or passes_max. The table is
 * active on return. RUNNING in POSITION mode only; blocks up to timeout_ms
 * and ends back RUNNING holding where the sweep ended.
 *
 * @return ESP_OK (check info->converged), ESP_ERR_INVALID_STATE,
 *         ESP_ERR_TIMEOUT, or ESP_FAIL on an abort/fault during the sweep.
 */
esp_err_t esp_foc_sensored_cogging_learn(uint8_t axis, uint32_t timeout_ms,
                                         esp_foc_sensored_cogging_info_t *info);
esp_err_t esp_foc_sensored_cogging_enable(uint8_t axis, bool on);
void esp_foc_sensored_get_cogging_info(uint8_t axis, esp_foc_sensored_cogging_info_t *info);

esp_foc_sensored_state_t esp_foc_sensored_get_state(uint8_t axis);
void esp_foc_sensored_get_status(uint8_t axis, esp_foc_sensored_status_t *st);
void esp_foc_sensored_get_window(uint8_t axis, esp_foc_sensored_window_t *w);
void esp_foc_sensored_get_tuning(uint8_t axis, esp_foc_sensored_tuning_t *t);

#ifdef __cplusplus
}
#endif
