/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

/**
 * @file esp_foc_injection.h
 * @brief Reference-signal injection for the current-loop q-axis.
 *
 * Lets a tuner or test harness temporarily override axis->target_i_q with a
 * deterministic waveform (step or chirp) so that step / frequency responses
 * can be captured via the scope without the host having to drive the
 * reference from outside.
 *
 * Kept Q16-only and allocation-free; the state lives inside the axis so no
 * external buffers are needed.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ESP_FOC_INJECTION_MODE_NONE  = 0,
    ESP_FOC_INJECTION_MODE_STEP  = 1,
    ESP_FOC_INJECTION_MODE_CHIRP = 2,
} esp_foc_injection_mode_t;

typedef struct {
    esp_foc_injection_mode_t mode;
    bool enabled;
    q16_t amplitude;            /* Q16 A (peak for chirp, magnitude for step) */
    q16_t freq_current_hz;      /* Q16 Hz (chirp only) */
    q16_t freq_end_hz;          /* Q16 Hz (chirp only) */
    q16_t freq_slope_hz_per_s;  /* Q16 Hz/s (chirp linear sweep rate) */
    q16_t phase_acc_rad;        /* Q16 radians, [0, 2*pi) */
    q16_t ts_seconds;           /* Q16 s, control-loop sample period */
    uint32_t samples_elapsed;
    uint32_t samples_total;
} esp_foc_injection_t;

/**
 * @brief Reset the injection state (disable).
 */
void esp_foc_injection_disable(esp_foc_injection_t *inj);

/**
 * @brief Configure a step injection on the q-axis reference.
 *
 * @param inj            injection state block
 * @param amplitude      Q16 A, held for duration_ms, then deactivated
 * @param duration_ms    how long to hold the step
 * @param ts_seconds     control loop period in Q16 seconds (for countdown)
 */
void esp_foc_injection_step_setup(esp_foc_injection_t *inj,
                                  q16_t amplitude,
                                  uint32_t duration_ms,
                                  q16_t ts_seconds);

/**
 * @brief Configure a linear-sweep chirp on the q-axis reference.
 *
 * @param inj            injection state block
 * @param amplitude      Q16 A (peak sinusoid amplitude)
 * @param freq_start_hz  initial frequency, Q16 Hz
 * @param freq_end_hz    final frequency, Q16 Hz
 * @param duration_ms    total sweep duration
 * @param ts_seconds     control loop period in Q16 seconds
 */
void esp_foc_injection_chirp_setup(esp_foc_injection_t *inj,
                                   q16_t amplitude,
                                   q16_t freq_start_hz,
                                   q16_t freq_end_hz,
                                   uint32_t duration_ms,
                                   q16_t ts_seconds);

/**
 * @brief Apply the injection to a base reference (one call per control step).
 *
 * If disabled, returns base_reference unchanged. Otherwise returns the base
 * reference added to the injection sample. Automatically deactivates when
 * the configured duration elapses.
 */
q16_t esp_foc_injection_apply_q16(esp_foc_injection_t *inj, q16_t base_reference);

#ifdef __cplusplus
}
#endif
