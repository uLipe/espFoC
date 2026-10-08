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
#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Phase map discovery: which MCPWM output drives which logical phase, and the
 * sign of the current sense, measured at standstill before anything else
 * excites the machine. Every probe downstream (identification, current
 * tuning, the observer) is taken in the dq frame this map defines; a wrong
 * one once read 8.3 ohm for a 2.2 ohm winding.
 *
 * Stage A pulses Vd at theta_e = 0 in both polarities for each of the twelve
 * uniform-sign candidates and ranks them by id - 2|iq| - 2|iv - iw|. The
 * pulse is shorter than the mechanical time constant and the polarity pair
 * cancels what the shaft does, so the answer is the winding, not the rotor.
 * Ranking rather than gating: an absolute gate sat on top of the measured
 * population and refused whole sweeps on a healthy bench.
 *
 * Stage C checks that +Vq answers +Iq. A negated q feedback is positive
 * feedback for the q regulator and no permutation can express it, so a
 * failure there is reported, not ranked away.
 *
 * Verify walks the ranking and accepts the first map that puts +Id on +Vd
 * with |Iq| < Id and passes Kirchhoff. A refused attempt is retried after a
 * 120 deg nudge, because Ld != Lq makes stage A sensitive to where the rotor
 * parked.
 *
 * With a rotor sensor, the field is then dragged one electrical revolution
 * at i_well_a and held on theta_e = 0 until the shaft is still, and the
 * sensor offset is taken there. Cogging holds the rotor anywhere within
 * asin(cog / i_well) of the well, and a rotor that starts near the unstable
 * point stays there unless the field sweeps it in; at 0.25 A with ~0.11 A of
 * cogging that was +-26 deg elec and two weak-torque boots, at 0.60 A after a
 * sweep it is 11 deg. The hold runs in voltage mode, sized from the
 * admittance verify measured, so no current loop is needed. A short +Vq
 * pulse finally reports whether the sensor counts against the field.
 *
 * Blocking, task context only. While run() executes the block owns the
 * inverter's TEZ, DMA and fault callbacks; it removes them on every return
 * path. Every candidate costs ~70 ms of arm/settle plus three pulse_ms
 * windows; an attempt is 16 candidates when the top map verifies and 27 when
 * verify walks the whole ranking (~2.4 s). Worst case with the defaults is
 * about 14 s: three attempts, two nudges of nudge_ms + settle_ms, and the
 * sensor stage bounded by sweep_ms + timeout_ms + dir_ms.
 *
 * Needs a 1 kHz RTOS tick: pulses are timed in 1 ms sleeps.
 *
 * With a sensor, nothing else may fetch it while run() executes.
 */

typedef enum {
    ESP_FOC_PHASE_DISCOVER_EV_ATTEMPT = 0, /* idx = attempt, value = probe Vd [pu] */
    ESP_FOC_PHASE_DISCOVER_EV_CANDIDATE,   /* stage A, idx = 0..11 */
    ESP_FOC_PHASE_DISCOVER_EV_RANKED,      /* idx = admitted count, map = best */
    ESP_FOC_PHASE_DISCOVER_EV_HANDED,      /* stage C, idx 0 = best, 1 = its V<->W mirror */
    ESP_FOC_PHASE_DISCOVER_EV_VERIFY,      /* idx = rank position, value = Kirchhoff residual */
    ESP_FOC_PHASE_DISCOVER_EV_REFUSED,     /* attempt failed, err set */
    ESP_FOC_PHASE_DISCOVER_EV_NUDGE,       /* idx = 120 deg step, ms = hold */
    ESP_FOC_PHASE_DISCOVER_EV_MAP,         /* accepted map, applied to the inverter */
    ESP_FOC_PHASE_DISCOVER_EV_WELL,        /* sweep done, value = hold Vd [pu] */
    ESP_FOC_PHASE_DISCOVER_EV_STILL,       /* ms = time to rest, good = reached */
    ESP_FOC_PHASE_DISCOVER_EV_ZERO,        /* err = calibrate_offset result */
    ESP_FOC_PHASE_DISCOVER_EV_DIR,         /* value = dtheta_m [rad] over the +Vq pulse */
} esp_foc_phase_discover_ev_t;

/** One step of the sequence, for logging. Currents are the pulse-pair half difference [A]. */
typedef struct {
    esp_foc_phase_discover_ev_t ev;
    int8_t idx;
    uint8_t attempt;
    esp_foc_phase_map_t map;
    q16_t id, iq, iu, iv, iw;
    q16_t score;
    q16_t value;
    uint32_t ms;
    esp_err_t err;
    bool good;
    bool tripped;
    bool reversed;
} esp_foc_phase_discover_event_t;

typedef void (*esp_foc_phase_discover_event_cb_t)(void *ctx,
                                                  const esp_foc_phase_discover_event_t *e);

typedef struct {
    /*
     * Probe height in volts, not a fraction of the bus: dead time and device
     * drop eat a fixed ~0.7 V, so a bus fraction that works at 24 V sits
     * inside the dead zone at 12 V. Clamped to [0.05, 0.25] of Vdc.
     */
    float vd_v;
    float id_min_a;
    uint32_t pulse_ms;
    int cal_rounds;
    uint8_t tries;
    uint32_t nudge_ms;
    /* Mechanical, not electrical: every candidate re-zeroes the shunts and
     * must not average the rotor's spring-back into the offset. */
    uint32_t settle_ms;
    struct {
        float i_well_a;
        uint32_t sweep_ms;
        float still_tol_rad;    /* max |theta_m - window start| counted as rest */
        /* Longer than one swing of the rotor in the well (~3 Hz pendulum here). */
        uint32_t calm_ms;
        uint32_t timeout_ms;
        int zero_samples;
        float dir_v;
        uint32_t dir_ms;
        float dir_min_rad;
    } sensor;
    esp_foc_phase_discover_event_cb_t on_event;
    void *ctx;
} esp_foc_phase_discover_config_t;

typedef struct {
    esp_foc_phase_map_t map;
    uint8_t attempts;
    int8_t rank_idx;        /* stage A index of the accepted map */
    q16_t id_verify;        /* [A] */
    q16_t iq_verify;        /* [A] */
    q16_t admittance;       /* id_verify / probe Vd [A per pu of Vdc] */
    bool sensor_zeroed;
    /* Reported only: the block cannot flip a sensor through the portable API.
     * Offsets taken before the flip stay valid on sensors that subtract the
     * offset before applying the direction. */
    bool sensor_reversed;
} esp_foc_phase_discover_result_t;

/** Caller-owned, no heap. Everything the ISRs touch lives here. */
typedef struct {
    esp_foc_inverter_t *inv;
    esp_foc_rotor_sensor_t *rotor;
    esp_foc_phase_discover_config_t cfg;
    q16_t vd_pu;
    q16_t id_min;
    q16_t dir_vq_pu;
    q16_t dir_min;
    q16_t still_tol;
    q16_t i_well;
    q16_t sweep_step;       /* rad per PWM period */
    volatile q16_t vd;
    volatile q16_t vq;
    volatile q16_t theta;
    volatile q16_t dtheta;
    volatile bool drive;
    volatile uint32_t fault_count;
    volatile esp_foc_fault_reason_t fault_reason;
    bool inited;
    bool installed;
    bool bridge_on;
} esp_foc_phase_discover_t;

void esp_foc_phase_discover_default_config(esp_foc_phase_discover_config_t *cfg);

/**
 * Bind the block to an inverter and an optional rotor sensor. A sensor must
 * report ESP_FOC_ROTOR_CAP_MECH_ABS. Does not touch the inverter. cfg NULL
 * takes the Kconfig defaults.
 *
 * @return ESP_OK, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_STATE (pd is running),
 *         ESP_ERR_NOT_SUPPORTED (sensor without MECH_ABS, or tick below 1 kHz).
 */
esp_err_t esp_foc_phase_discover_init(esp_foc_phase_discover_t *pd,
                                      esp_foc_inverter_t *inv,
                                      esp_foc_rotor_sensor_t *rotor,
                                      const esp_foc_phase_discover_config_t *cfg);

/**
 * Blocking. On ESP_OK the map is applied to the inverter. On every return the
 * bridge is disabled and the inverter callbacks are NULL.
 *
 * @return ESP_OK, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_STATE (ISR context,
 *         not inited, or re-entered), ESP_FAIL (no candidate carried current,
 *         or the bridge tripped in the sensor stage),
 *         ESP_ERR_INVALID_RESPONSE (sense mirrored, no map verified, or the
 *         shaft did not answer the direction pulse), ESP_ERR_TIMEOUT (rotor
 *         never came to rest), or the sensor's error.
 */
esp_err_t esp_foc_phase_discover_run(esp_foc_phase_discover_t *pd,
                                     esp_foc_phase_discover_result_t *out);

/**
 * Idempotent. Removes the callbacks the block installed and, if the block
 * left the bridge enabled, parks the duties at mid and disables it. Never
 * touches an inverter the block did not arm, so it is a no-op after run()
 * and does not disturb callbacks the caller registered since. Safe without
 * a run, twice, or after a failed run.
 */
void esp_foc_phase_discover_cleanup(esp_foc_phase_discover_t *pd);

#ifdef __cplusplus
}
#endif
