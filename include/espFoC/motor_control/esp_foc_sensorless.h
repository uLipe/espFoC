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
#include "espFoC/motor_control/esp_foc_phase_discover.h"
#include "espFoC/utils/esp_foc_q16.h"

#if CONFIG_ESP_FOC_ENABLE_MOTOR_ID
#include "espFoC/motor_control/esp_foc_motor_id.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Sensorless FoC stack, up to CONFIG_ESP_FOC_SL_MAX_AXES axes.
 *
 * Each axis is a statically allocated instance selected by cfg.axis at init()
 * and by the axis argument of every other call; axes share nothing but the
 * code. An axis owns its inverter's TEZ, DMA and fault callbacks from init()
 * to deinit(), and runs its own supervisor task. Park follows the flux observer once the startup has handed off;
 * before that it rides the I-f lock-in angle.
 *
 * Execution contexts:
 *  - TEZ (every PWM period): Clarke, observer, angle select, Park, current
 *    PI d/q plus the user vdq feedforward, vlim, inverse Park, SVM. Every
 *    SLOW_DIV periods the same ISR runs the speed PI (user iq added to its
 *    output) and the guards; a guard trip wakes the supervisor.
 *  - Supervisor task: the state machine below, the startup through
 *    esp_foc_if, the handoff, the speed/torque catch, the reversal ramps and
 *    every event callback.
 *
 * run() only arms. The startup is triggered by the reference: |w_ref| above
 * w_min_hz in speed mode, |iq| above iq_min_a in torque mode, the sign gives
 * the direction. A reference below the cut, in any active state, is a dry
 * cut (mid duty, bridge disabled) back to ARMED. A sign change while caught
 * or running decelerates (wref to the handoff frequency, or iq to 0), coasts
 * and relaunches the other way; during the startup it cuts and relaunches.
 *
 * STARTUP_FAILED, ABORT and FAULT latch: nothing starts until clear_fault().
 *
 * There is no position control and no rotor sensor: the observer is the
 * only angle source.
 */

typedef enum {
    ESP_FOC_SL_STATE_IDLE = 0,
    ESP_FOC_SL_STATE_ARMED,
    ESP_FOC_SL_STATE_ALIGN,
    ESP_FOC_SL_STATE_LOCKIN,
    ESP_FOC_SL_STATE_PLL_ACQUIRE,
    ESP_FOC_SL_STATE_HANDOFF,
    ESP_FOC_SL_STATE_CATCH,
    ESP_FOC_SL_STATE_RUNNING,
    ESP_FOC_SL_STATE_REVERSING,
    ESP_FOC_SL_STATE_FAULT,
} esp_foc_sensorless_state_t;

typedef enum {
    ESP_FOC_SL_EV_ARMED = 0,
    ESP_FOC_SL_EV_STARTUP,        /* dir */
    ESP_FOC_SL_EV_LOCKED,         /* we_rads, dang_rad */
    ESP_FOC_SL_EV_HANDOFF,        /* we_rads, dang_rad */
    ESP_FOC_SL_EV_RUNNING,        /* speed_mode, dir */
    ESP_FOC_SL_EV_REVERSING,      /* dir = the new direction */
    ESP_FOC_SL_EV_CUT,
    ESP_FOC_SL_EV_STOPPED,
    ESP_FOC_SL_EV_STARTUP_FAILED, /* fail, state */
    ESP_FOC_SL_EV_ABORT,          /* abort */
    ESP_FOC_SL_EV_FAULT,          /* fault */
    ESP_FOC_SL_EV_FAULT_CLEARED,
} esp_foc_sensorless_ev_t;

typedef enum {
    ESP_FOC_SL_FAIL_NONE = 0,
    ESP_FOC_SL_FAIL_ENABLE,       /* inverter refused enable() */
    ESP_FOC_SL_FAIL_SEQUENCE,     /* esp_foc_if refused its config */
    ESP_FOC_SL_FAIL_NO_CURRENT,   /* Id did not follow its reference on the ramp */
    ESP_FOC_SL_FAIL_NO_FOLLOW,    /* the shaft never made the BEMF the ramp implies */
    ESP_FOC_SL_FAIL_PLL_TIMEOUT,  /* PLL never held band and hunt */
    ESP_FOC_SL_FAIL_WE_LOW,       /* |we| under we_min after the Park blend */
    ESP_FOC_SL_FAIL_WE_DROP,      /* |we| fell under we_min during the settle */
} esp_foc_sensorless_fail_t;

typedef enum {
    ESP_FOC_SL_ABORT_NONE = 0,
    ESP_FOC_SL_ABORT_LOCK_LOSS,
    ESP_FOC_SL_ABORT_OVERSPEED,
    ESP_FOC_SL_ABORT_BEMF,        /* |e| under bemf_frac·psi·|w| */
    ESP_FOC_SL_ABORT_COLLAPSE,    /* |id|,|iq| near zero with iq asked for */
} esp_foc_sensorless_abort_t;

typedef struct {
    esp_foc_sensorless_ev_t ev;
    uint8_t axis;
    esp_foc_sensorless_state_t state;
    int8_t dir;
    bool speed_mode;
    esp_foc_sensorless_fail_t fail;
    esp_foc_sensorless_abort_t abort;
    esp_foc_fault_reason_t fault;
    float we_rads;
    float dang_rad;               /* theta_obs - theta_ol */
} esp_foc_sensorless_event_t;

typedef void (*esp_foc_sensorless_event_cb_t)(void *ctx,
                                              const esp_foc_sensorless_event_t *e);

/*
 * Frequencies are electrical. Fields marked "0 = derive" are designed at
 * init from the plant and fe_rated_hz; esp_foc_sensorless_default_config()
 * leaves them 0 and fills the rest from Kconfig.
 */
typedef struct {
    uint8_t axis;                 /* instance, below CONFIG_ESP_FOC_SL_MAX_AXES */
    float rs_ohm;
    float ls_h;
    float psi_wb;
    uint8_t pole_pairs;
    float fe_rated_hz;
    float v_deadzone_v;           /* bridge dead zone, adds to the V/f boost */

    float i_max_a;                /* |iq| ceiling of the speed loop and the setters */
    float i_align_a;
    float id_run_a;               /* Id after the handoff until set_id() */

    float i_bw_hz;
    float kp_i;                   /* 0 = IMC-zoh from R, L (pu of Vdc per A) */
    float ki_i;

    bool speed_loop;              /* false = torque only */
    float speed_bw_hz;            /* 0 = derive from fe_rated_hz */
    float speed_zeta;
    float kp_w;                   /* 0 = derive from i_max_a and speed_err_sat_frac */
    float ki_w;
    float speed_filt_frac;        /* w_ctrl LPF corner as a fraction of speed_bw_hz */
    float speed_err_sat_frac;     /* speed error that saturates iq, of fe_rated_hz */
    float wref_slew_hz_s;

    float w_min_hz;               /* speed-mode start/cut on |w_ref| */
    float iq_min_a;               /* torque-mode start/cut on |iq| */
    float rev_decel_hz_s;         /* speed-mode reversal ramp */
    float rev_decel_a_s;          /* torque-mode reversal ramp */
    uint32_t coast_ms;            /* minimum bridge-off time before a relaunch */

    struct {
        int cal_rounds;
        uint32_t arm_ms;
        uint32_t align_ms;
        uint32_t align_settle_ms;
        uint32_t align_timeout_ms;
        uint32_t step_ms;
        float accel_rads2;
        float plateau_hz;         /* 0 = derive from f_base */
        float plateau_fbase_frac;
        float plateau_min_hz;
        float plateau_max_hz;
        float vf_hz;              /* 0 = no V/f break-away */
        uint32_t vf_ramp_ms;
        float vf_margin_v;
        float follow_frac;
        uint32_t follow_hold_ms;
        uint32_t follow_timeout_ms;
        float ramp_id_err_a;
        uint32_t ramp_id_err_ms;
    } startup;

    struct {
        float band_hz;
        float hunt_hz;
        uint32_t hold_ms;
        uint32_t timeout_ms;
        uint32_t blend_ms;
        uint32_t settle_ms;
        float iq_start_a;
        float we_min_frac;        /* of the plateau */
    } handoff;

    struct {
        float floor_a;            /* speed-mode |iq| floor while catching */
        float brake_a;            /* speed-mode reverse-torque allowance once caught */
        uint32_t hold_ms;         /* speed-mode hold on |w_obs| before ramping */
        float hold_fbase_frac;    /* ceiling of that hold, of f_base */
        float close_frac;
        float close_min_hz;
        uint32_t close_timeout_ms;
        float slew_a_s;           /* torque-mode ramp to the user iq, Id ramp */
    } catch_up;

    struct {
        float obs_bw_hz;
        float obs_zeta;
        float track_bw_hz;        /* 0 = derive */
        float track_zeta;
        float blend_hz;           /* 0 = derive */
        float psi_lock_frac;
        float w_max_frac;         /* of fe_rated_hz */
        uint16_t lock_count;
    } observer;

    struct {
        float bemf_frac;
        uint32_t bemf_hold_ms;
        float bemf_fmin_hz;
        float overspeed_frac;     /* of fe_rated_hz */
        uint32_t overspeed_hold_ms;
        float collapse_a;
        uint32_t collapse_hold_ms;
        uint32_t lock_loss_ms;
        bool lock_loss_abort;
    } guard;

    esp_foc_phase_map_t map;
    bool map_valid;

    esp_foc_sensorless_event_cb_t on_event;
    void *ctx;
} esp_foc_sensorless_config_t;

typedef struct {
    esp_foc_sensorless_state_t state;
    int8_t dir;
    bool park_on_observer;
    bool observer_locked;
    float theta_e_rad;
    float we_rads;
    float w_ctrl_rads;
    float w_ref_rads;
    float fe_ol_hz;
    float id_a;
    float iq_a;
    float id_ref_a;
    float iq_ref_a;
    float vd_v;
    float vq_v;
    float vdc_v;
    float f_base_hz;
    float plateau_hz;
    float we_min_hz;
    uint32_t tez;
    esp_foc_sensorless_fail_t fail;
    esp_foc_sensorless_abort_t abort;
    esp_foc_fault_reason_t fault;
} esp_foc_sensorless_status_t;

/* Resolved tuning after init(), for logs and the setters' range rules. */
typedef struct {
    float kp_i;
    float ki_i;
    float speed_bw_hz;
    float kp_w;
    float ki_w;
    float speed_filt_hz;
    float track_bw_hz;
    float blend_hz;
    float slot_hz;
} esp_foc_sensorless_tuning_t;

void esp_foc_sensorless_default_config(esp_foc_sensorless_config_t *cfg);

#if CONFIG_ESP_FOC_ENABLE_MOTOR_ID
/* Copies the fields valid_mask marks valid. R prefers the loop value: that
 * is the series resistance the current PI and v - R·i act against. */
void esp_foc_sensorless_config_from_motor_id(esp_foc_sensorless_config_t *cfg,
                                             const esp_foc_motor_id_result_t *r);
#endif

void esp_foc_sensorless_config_from_phase_map(esp_foc_sensorless_config_t *cfg,
                                              const esp_foc_phase_discover_result_t *r);

/**
 * Validates the config, designs the loops, applies the phase map (bridge
 * must be disabled), installs the inverter callbacks and starts the
 * supervisor. Ends in IDLE with the bridge untouched.
 *
 * @return ESP_OK, ESP_ERR_INVALID_ARG (also an axis out of range),
 *         ESP_ERR_INVALID_STATE (axis already initialised), ESP_ERR_NO_MEM (task), or the design's error.
 */
esp_err_t esp_foc_sensorless_init(esp_foc_inverter_t *inv,
                                  const esp_foc_sensorless_config_t *cfg);

/*
 * Calls below act on one axis. An axis out of range is ESP_ERR_INVALID_ARG
 * (IDLE for get_state, no-op for the void calls).
 */

/* Dry cut, stops the supervisor, removes the callbacks. Task context. */
void esp_foc_sensorless_deinit(uint8_t axis);

/* IDLE -> ARMED. Does not enable the bridge. */
esp_err_t esp_foc_sensorless_run(uint8_t axis);

/* Dry cut from any state, ends in IDLE. Blocks until the bridge is off
 * unless called from the event callback. */
esp_err_t esp_foc_sensorless_stop(uint8_t axis);

/* Clears the stack latch and the inverter's, then FAULT -> ARMED.
 * ESP_ERR_INVALID_STATE if nothing is latched or the inverter refuses. */
esp_err_t esp_foc_sensorless_clear_fault(uint8_t axis);

/* Speed mode only, signed electrical Hz. */
esp_err_t esp_foc_sensorless_set_speed_ref_hz(uint8_t axis, float fe_hz);
esp_err_t esp_foc_sensorless_set_speed_slew(uint8_t axis, float hz_per_s);
/* Torque mode: the iq reference. Speed mode: added to the speed PI output. */
esp_err_t esp_foc_sensorless_set_iq(uint8_t axis, float a);
esp_err_t esp_foc_sensorless_set_id(uint8_t axis, float a);
/* Added to the current PI outputs, both modes, from CATCH on: the align and
 * the lock-in run without it. */
esp_err_t esp_foc_sensorless_set_vdq_ff(uint8_t axis, float vd_v, float vq_v);
esp_err_t esp_foc_sensorless_set_current_pi(uint8_t axis, float kp, float ki);
esp_err_t esp_foc_sensorless_set_speed_pi(uint8_t axis, float kp, float ki);
/* Redesigns the speed PI and its feedback filter. Must stay under the PLL
 * bandwidth, and the filter under a quarter of the slot rate. */
esp_err_t esp_foc_sensorless_set_speed_bw(uint8_t axis, float hz);

esp_foc_sensorless_state_t esp_foc_sensorless_get_state(uint8_t axis);
void esp_foc_sensorless_get_status(uint8_t axis, esp_foc_sensorless_status_t *st);
void esp_foc_sensorless_get_tuning(uint8_t axis, esp_foc_sensorless_tuning_t *t);

#ifdef __cplusplus
}
#endif
