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
#include "espFoC/motor_control/esp_foc_ident.h"
#include "esp_foc_motor_id_seq.h"
#include "espFoC/osal/esp_foc_osal.h"
#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Excitation and demodulation for the identification probes, over an inverter.
 *
 * This is the half of the sequencer's ops that has to live in the PWM ISR: it
 * emits the excitation sample and correlates the response in the same interrupt
 * that produced it. The dq pipeline ops (set_idq, set_fe_hz, fetch_dq,
 * apply_gains) stay with the caller, because those drive the caller's current
 * loop and there is no second one here.
 *
 * The probes deliberately read @c fetch_currents_raw. A sense LPF adds phase
 * that grows with frequency, which is indistinguishable from a wrong sense
 * delay and would make LAGCAL refuse the run.
 */
typedef enum {
    ESP_FOC_MOTOR_ID_PROBE_OFF = 0,
    ESP_FOC_MOTOR_ID_PROBE_AC,
    ESP_FOC_MOTOR_ID_PROBE_DC,
    ESP_FOC_MOTOR_ID_PROBE_TERMINAL,
    ESP_FOC_MOTOR_ID_PROBE_STEP,
} esp_foc_motor_id_probe_mode_t;

/** Why a window was discarded; a refused probe is otherwise indistinguishable
 *  from a machine that simply does not conduct. */
typedef enum {
    ESP_FOC_MOTOR_ID_ABORT_NONE = 0,
    ESP_FOC_MOTOR_ID_ABORT_FAULT,
    ESP_FOC_MOTOR_ID_ABORT_OVERCURRENT,
    ESP_FOC_MOTOR_ID_ABORT_TIMEOUT,
} esp_foc_motor_id_probe_abort_t;

typedef struct {
    esp_foc_inverter_t *inv;
    q16_t inv_vdc;      /**< 1/Vdc, so the ISR never divides */
    q16_t v_max_pu;     /**< modulation ceiling, per-unit of Vdc */
    q16_t i_abort;      /**< a window that reaches this is discarded */
    /**
     * Ceiling for the DC and terminal windows, which is deliberately looser.
     *
     * Holding a DC vector on a free shaft is not an electrical measurement alone:
     * the rotor swings in the well the vector creates, and with 13 pole pairs a
     * small mechanical swing is a fast electrical one, so the current carries real
     * BEMF. This bench rang to 2180 mA around a 500 mA injection and the run before
     * it did not, depending only on where the shaft happened to be resting. The
     * mean is still bounded by the supply's constant current, and a genuine short
     * is the bridge's own limit to catch.
     */
    q16_t i_abort_dc;
    uint32_t settle_samples;
    uint32_t avg_samples;
    uint32_t timeout_ms;

    volatile esp_foc_motor_id_probe_mode_t mode;
    q16_t sin_th;       /**< Park angle is fixed for a probe, so sincos is
                         *   resolved once in thread context */
    q16_t cos_th;
    q16_t v_pu;
    int terminal;
    esp_foc_ident_zprobe_t z;
    /** [0] carries the demodulated axis for AC and DC; all three are filled for
     *  a terminal window, which is what makes an asymmetry readable. */
    esp_foc_ident_dcprobe_t dc[3];
    int32_t i_ma[3];
    uint32_t n_win;
    /**
     * Step window, summed over many repetitions.
     *
     * One step cannot be timed on a bench whose sense floor is larger than the
     * first sample of the rise, and this one's is: 700 mA of peak against 170 mA.
     * The step is deterministic and locked to the carrier, so repeating it and
     * summing by position beats the floor down by the root of the count while
     * leaving the edge exactly where it was.
     */
    int32_t step_acc[ESP_FOC_IDENT_STEP_MAX];
    uint32_t step_pre;
    uint32_t step_period;   /**< samples per repetition, including the decay */
    uint32_t step_reps;
    /** Samples of a window that the over-current guard ignores; see the ISR. */
    uint32_t guard_after;
    volatile q16_t i_peak;
    q16_t i_avg;
    /** Last excitation and response the AC window drove, for tracing. */
    volatile q16_t i_dq;
    volatile bool aborted;
    esp_foc_motor_id_probe_abort_t abort_why;
    esp_foc_event_handle_t waiter;
} esp_foc_motor_id_probe_t;

/**
 * @p vdc and the abort ceiling come from the caller rather than the inverter so
 * a bench can probe below its trip level.
 */
esp_err_t esp_foc_motor_id_probe_init(esp_foc_motor_id_probe_t *p,
                                     esp_foc_inverter_t *inv,
                                     float vdc,
                                     float i_abort_a);

/**
 * Call from the PWM ISR. Returns false when no probe is armed, which is the
 * caller's cue to run its own control pipeline for this period.
 */
bool esp_foc_motor_id_probe_isr(esp_foc_motor_id_probe_t *p);

/**
 * Fills the three probe ops and takes over @c ops->ctx, since the vtable has a
 * single context. The caller's remaining ops therefore reach their own state
 * directly and ignore the ctx argument.
 */
void esp_foc_motor_id_probe_bind(esp_foc_motor_id_probe_t *p,
                                 esp_foc_motor_id_seq_ops_t *ops);

#ifdef __cplusplus
}
#endif
