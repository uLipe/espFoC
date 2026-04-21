/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

/**
 * @file esp_foc_calibration.h
 * @brief Per-axis calibration overlay backed by NVS.
 *
 * The build-time autogen header gives every axis a sensible
 * starting point. Once the operator has tuned the loop with the
 * runtime tuner (TunerStudio, tunerctl, or any other host that
 * speaks the protocol), the resulting gains can be persisted into
 * a dedicated NVS namespace so subsequent boots come up already
 * tuned — no host required.
 *
 * Boot priority in esp_foc_initialize_axis():
 *
 *   1. NVS overlay for this axis, IF its profile_hash matches the
 *      currently configured profile (`CONFIG_ESP_FOC_MOTOR_PROFILE`
 *      + `CONFIG_ESP_FOC_PROFILE_VERSION`). Mismatch is treated as
 *      "absent" — we never apply gains tuned for a different motor.
 *   2. Build-time autogen gains.
 *   3. If neither: zeroed gains, motor cannot be commanded until
 *      the runtime tuner programs them.
 *
 * Compiled in only when CONFIG_ESP_FOC_CALIBRATION_NVS is set.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <sdkconfig.h>
#include "espFoC/esp_foc.h"
#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Public payload — stored verbatim in NVS, exchanged through the API. */
typedef struct {
    q16_t kp;
    q16_t ki;
    q16_t integrator_limit;
    q16_t motor_r_ohm;
    q16_t motor_l_h;
    q16_t bandwidth_hz;
    /* Reserved for speed loop / observer params in a future revision. */
    uint8_t reserved[16];
} esp_foc_calibration_data_t;

/**
 * @brief Hash of the current build's motor profile + schema version.
 *
 * NVS overlays are tagged with this hash so a calibration tuned on
 * one motor cannot be silently applied on another.
 */
uint32_t esp_foc_calibration_profile_hash(void);

/**
 * @brief Persist a calibration payload for the given axis to NVS.
 *
 * Lazy-initialises the underlying NVS namespace on first call.
 * Returns ESP_FOC_OK on success or a negative status when NVS
 * cannot be opened / written.
 */
esp_foc_err_t esp_foc_calibration_save(uint8_t axis_id,
                                       const esp_foc_calibration_data_t *data);

/**
 * @brief Load the calibration payload for the given axis.
 *
 * Returns ESP_FOC_OK only when a matching blob is present (magic +
 * version + CRC + profile_hash all agree). Mismatch returns
 * ESP_FOC_ERR_AXIS_INVALID_STATE so callers can distinguish "no
 * calibration yet" from "I/O failure".
 */
esp_foc_err_t esp_foc_calibration_load(uint8_t axis_id,
                                       esp_foc_calibration_data_t *out);

/**
 * @brief Erase every persisted calibration in the espFoC namespace.
 */
esp_foc_err_t esp_foc_calibration_erase(void);

/**
 * @brief True when a calibration for the given axis exists and
 * matches the current profile_hash.
 */
bool esp_foc_calibration_present(uint8_t axis_id);

#ifdef __cplusplus
}
#endif
