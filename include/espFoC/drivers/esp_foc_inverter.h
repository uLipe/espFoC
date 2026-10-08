/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct esp_foc_inverter_s esp_foc_inverter_t;

typedef void (*esp_foc_inverter_cb_t)(void *arg);

typedef enum {
    ESP_FOC_FAULT_NONE = 0,
    ESP_FOC_FAULT_ILIMIT,
    ESP_FOC_FAULT_GPIO,
    ESP_FOC_FAULT_SOFT_TRIP,
    /*
     * The sense frame stopped changing. This has to be its own fault and not a
     * diagnostic counter, because `ESP_FOC_FAULT_ILIMIT` compares the same
     * samples the control loop uses: when they freeze, the current regulators
     * wind to the modulation ceiling and over-current protection is looking at
     * a constant, so nothing stops the bridge. Measured 2026-08-28: Iq held
     * -0.367 A for 20 s with vd at 0.447 and no trip.
     */
    ESP_FOC_FAULT_SENSE_STALE,
} esp_foc_fault_reason_t;

typedef void (*esp_foc_fault_cb_t)(void *arg, esp_foc_fault_reason_t reason);

/**
 * Logical UVW ↔ hardware MCPWM ops / sense slots.
 * pwm_to_hw[L] = HW index (0..2) for logical phase L (U=0,V=1,W=2).
 * Must be a permutation of {0,1,2}. i_sign[L] = ±1 after gather.
 */
typedef struct {
    uint8_t pwm_to_hw[3];
    int8_t i_sign[3];
} esp_foc_phase_map_t;

static inline void esp_foc_phase_map_identity(esp_foc_phase_map_t *m)
{
    if (m == NULL) {
        return;
    }
    m->pwm_to_hw[0] = 0;
    m->pwm_to_hw[1] = 1;
    m->pwm_to_hw[2] = 2;
    m->i_sign[0] = 1;
    m->i_sign[1] = 1;
    m->i_sign[2] = 1;
}

static inline bool esp_foc_phase_map_valid(const esp_foc_phase_map_t *m)
{
    if (m == NULL) {
        return false;
    }
    bool seen[3] = {false, false, false};
    for (int i = 0; i < 3; i++) {
        uint8_t h = m->pwm_to_hw[i];
        if (h > 2u || seen[h]) {
            return false;
        }
        seen[h] = true;
        if (m->i_sign[i] != 1 && m->i_sign[i] != -1) {
            return false;
        }
    }
    return true;
}

/**
 * Portable inverter mechanism (PWM + current sense + fault).
 * Platform factories (e.g. MCPWM) return this interface.
 */
struct esp_foc_inverter_s {
    void (*set_pwm_callback)(esp_foc_inverter_t *self,
                             esp_foc_inverter_cb_t cb,
                             void *arg);
    void (*set_dma_callback)(esp_foc_inverter_t *self,
                             esp_foc_inverter_cb_t cb,
                             void *arg);
    void (*set_fault_callback)(esp_foc_inverter_t *self,
                               esp_foc_fault_cb_t cb,
                               void *arg);
    esp_err_t (*enable)(esp_foc_inverter_t *self);
    void (*disable)(esp_foc_inverter_t *self);
    void (*set_duties)(esp_foc_inverter_t *self,
                       q16_t duty_u, q16_t duty_v, q16_t duty_w);
    q16_t (*get_dc_link_voltage)(esp_foc_inverter_t *self);
    uint32_t (*get_pwm_rate_hz)(esp_foc_inverter_t *self);
    /** Canonical phase currents (LPF output when enabled). */
    void (*fetch_currents)(esp_foc_inverter_t *self,
                           q16_t *iu, q16_t *iv, q16_t *iw);
    /** Last raw (pre-LPF) sample — diagnostics / sense characterize. */
    void (*fetch_currents_raw)(esp_foc_inverter_t *self,
                               q16_t *iu, q16_t *iv, q16_t *iw);
    bool (*sample_ready)(esp_foc_inverter_t *self);
    void (*calibrate_currents)(esp_foc_inverter_t *self, int rounds);
    /**
     * Arm the frozen-sense watchdog. Off after init and after every enable().
     *
     * A constant reading only means the sense is dead if the current was supposed
     * to be moving, and only the caller knows that: mid duty, a DC hold and a
     * standstill align all answer one value indefinitely on a healthy converter.
     * Arm it for the closed-loop spin, where a frozen frame lets the regulators
     * wind into the modulation ceiling with over-current protection watching a
     * still picture, and leave it off for bring-up and identification.
     */
    void (*set_sense_watchdog)(esp_foc_inverter_t *self, bool enable);
    void (*soft_trip)(esp_foc_inverter_t *self);
    esp_err_t (*clear_fault)(esp_foc_inverter_t *self);
    bool (*is_faulted)(esp_foc_inverter_t *self);
    esp_foc_fault_reason_t (*get_fault_reason)(esp_foc_inverter_t *self);
    /** Only while disabled. Validates permutation + ±1 signs. */
    esp_err_t (*set_phase_map)(esp_foc_inverter_t *self, const esp_foc_phase_map_t *map);
    void (*get_phase_map)(esp_foc_inverter_t *self, esp_foc_phase_map_t *map);
};

#ifdef __cplusplus
}
#endif
