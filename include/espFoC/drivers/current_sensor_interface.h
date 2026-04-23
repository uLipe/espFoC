/*
 * MIT License
 */
#pragma once

#include "espFoC/utils/esp_foc_q16.h"

typedef struct {
    q16_t iu_axis_0;
    q16_t iv_axis_0;
    q16_t iw_axis_0;
    q16_t iu_axis_1;
    q16_t iv_axis_1;
    q16_t iw_axis_1;
} isensor_values_t;

typedef struct esp_foc_isensor_s esp_foc_isensor_t;
typedef void (*isensor_callback_t)(void *arg);

struct esp_foc_isensor_s {
    void (*fetch_isensors)(esp_foc_isensor_t *self, isensor_values_t *values);
    void (*sample_isensors)(esp_foc_isensor_t *self);
    void (*calibrate_isensors)(esp_foc_isensor_t *self, int calibration_rounds);
    void (*set_isensor_callback)(esp_foc_isensor_t *self, isensor_callback_t cb, void *param);

    /**
     * @brief Designs / re-designs the per-phase low-pass biquad inside
     *        the driver. Called from esp_foc_initialize_axis() and from
     *        the runtime tuner when the operator changes the cutoff.
     *
     * Drivers without a built-in filter (mocks, future hall-only)
     * provide a no-op stub.
     */
    void (*set_filter_cutoff)(esp_foc_isensor_t *self, float fc_hz, float fs_hz);

    /**
     * @brief After each conversion the driver applies offset, gain, and
     *        (optionally) Clarke to axis-0 shunt channels U / V, then
     *        writes i_alpha / i_beta, and (if the last two args are
     *        non-NULL) the same-tick U / V phase currents for scope/debug.
     *        The PWM ISR can read the published values without
     *        fetch_isensors().
     *
     * Pass NULL for any target to skip that sink. Drivers that do not run
     * an ISR-side conversion provide a no-op stub.
     */
    void (*set_publish_targets)(esp_foc_isensor_t *self,
                                q16_t *i_alpha_target,
                                q16_t *i_beta_target,
                                q16_t *i_u_target,
                                q16_t *i_v_target);
};
