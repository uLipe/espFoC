/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

/**
 * @file esp_foc_axis_tuning.h
 * @brief Runtime tuning API for axis current-loop PI gains.
 *
 * Lets a host (manual call, scope/tuner backend, or NVS load) reprogram the
 * current-loop gains without re-initializing the axis. The gain swap is
 * performed inside a critical section so the PID never observes a torn pair
 * of (Kp, Ki, integrator_limit). On retune the integrator is reset to keep
 * the post-swap response well-defined.
 *
 * The MPZ-based path (esp_foc_axis_retune_current_pi_q16) computes the
 * current sample period from the inverter's PWM rate and the configured
 * downsampling factor; both must be valid (axis must be initialized).
 */

#pragma once

#include "espFoC/esp_foc.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/esp_foc_design_mpz.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Compute MPZ gains from motor params and apply them atomically.
 *
 * @param axis           initialized axis (must have an inverter attached)
 * @param motor_r_ohm    phase resistance R [Q16, Ohm]
 * @param motor_l_h      phase inductance L [Q16, H]
 * @param bandwidth_hz   target closed-loop bandwidth [Q16, Hz]
 * @return ESP_FOC_OK or an error from the design or argument validation
 */
esp_foc_err_t esp_foc_axis_retune_current_pi_q16(
    esp_foc_axis_t *axis,
    q16_t motor_r_ohm,
    q16_t motor_l_h,
    q16_t bandwidth_hz);

/**
 * @brief Apply pre-computed gains atomically (manual override / NVS load).
 */
esp_foc_err_t esp_foc_axis_set_current_pi_gains_q16(
    esp_foc_axis_t *axis,
    q16_t kp,
    q16_t ki,
    q16_t integrator_limit);

/**
 * @brief Read back the currently active current-loop PI gains.
 *
 * Output pointers may be NULL to skip individual fields.
 */
void esp_foc_axis_get_current_pi_gains_q16(
    const esp_foc_axis_t *axis,
    q16_t *kp,
    q16_t *ki,
    q16_t *integrator_limit);

/**
 * @brief Arm a step injection on the q-axis current reference.
 *
 * Only active when CONFIG_ESP_FOC_INJECTION_ENABLE is set; otherwise
 * returns ESP_FOC_ERR_NOT_ALIGNED (compile-time disabled).
 */
esp_foc_err_t esp_foc_axis_inject_step_q16(
    esp_foc_axis_t *axis,
    q16_t amplitude,
    uint32_t duration_ms);

/**
 * @brief Arm a linear chirp injection on the q-axis current reference.
 */
esp_foc_err_t esp_foc_axis_inject_chirp_q16(
    esp_foc_axis_t *axis,
    q16_t amplitude,
    q16_t freq_start_hz,
    q16_t freq_end_hz,
    uint32_t duration_ms);

/**
 * @brief Disable any active injection.
 */
esp_foc_err_t esp_foc_axis_inject_stop(esp_foc_axis_t *axis);

#ifdef __cplusplus
}
#endif
