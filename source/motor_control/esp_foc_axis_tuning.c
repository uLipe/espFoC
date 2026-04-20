/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include <sdkconfig.h>
#include "espFoC/esp_foc_axis_tuning.h"
#include "espFoC/esp_foc_controls.h"
#include "espFoC/esp_foc_injection.h"

/* Apply (kp, ki, integrator_limit) to both torque controllers atomically.
 * Called with the axis lock held. */
static void apply_gains_locked(esp_foc_axis_t *axis,
                               q16_t kp, q16_t ki, q16_t integrator_limit)
{
    for (int i = 0; i < 2; ++i) {
        axis->torque_controller[i].kp = kp;
        axis->torque_controller[i].ki = ki;
        axis->torque_controller[i].integrator_limit = integrator_limit;
        /* Reset integrator/error history so post-swap response is well-defined. */
        axis->torque_controller[i].integrator = 0;
        axis->torque_controller[i].prev_error = 0;
    }
}

esp_foc_err_t esp_foc_axis_retune_current_pi_q16(
    esp_foc_axis_t *axis,
    q16_t motor_r_ohm,
    q16_t motor_l_h,
    q16_t bandwidth_hz)
{
    if (axis == NULL || axis->inverter_driver == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if (motor_r_ohm <= 0 || motor_l_h <= 0 || bandwidth_hz <= 0) {
        return ESP_FOC_ERR_INVALID_ARG;
    }

    int pwm_rate_hz = axis->inverter_driver->get_inverter_pwm_rate(axis->inverter_driver);
    if (pwm_rate_hz <= 0) {
        return ESP_FOC_ERR_INVALID_ARG;
    }

    /* loop_ts_us = 1e6 * DOWNSAMPLING / pwm_rate_hz */
    uint32_t loop_ts_us = (uint32_t)((1000000ULL * (uint64_t)ESP_FOC_LOW_SPEED_DOWNSAMPLING)
                                     / (uint64_t)pwm_rate_hz);
    if (loop_ts_us == 0) {
        return ESP_FOC_ERR_TIMESTEP_TOO_SMALL;
    }

    esp_foc_pi_design_input_t in = {
        .motor_r_ohm  = motor_r_ohm,
        .motor_l_h    = motor_l_h,
        .loop_ts_us   = loop_ts_us,
        .bandwidth_hz = bandwidth_hz,
        .v_max        = axis->max_voltage,
    };
    esp_foc_pi_design_output_t out;
    esp_foc_design_status_t st = esp_foc_design_pi_current_mpz_q16(&in, &out);
    if (st != ESP_FOC_DESIGN_OK || !out.valid) {
        return ESP_FOC_ERR_INVALID_ARG;
    }

    esp_foc_critical_enter();
    apply_gains_locked(axis, out.kp, out.ki, out.integrator_limit);
    esp_foc_critical_leave();

    return ESP_FOC_OK;
}

esp_foc_err_t esp_foc_axis_set_current_pi_gains_q16(
    esp_foc_axis_t *axis,
    q16_t kp,
    q16_t ki,
    q16_t integrator_limit)
{
    if (axis == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if (kp < 0 || ki < 0 || integrator_limit < 0) {
        return ESP_FOC_ERR_INVALID_ARG;
    }

    esp_foc_critical_enter();
    apply_gains_locked(axis, kp, ki, integrator_limit);
    esp_foc_critical_leave();
    return ESP_FOC_OK;
}

void esp_foc_axis_get_current_pi_gains_q16(
    const esp_foc_axis_t *axis,
    q16_t *kp,
    q16_t *ki,
    q16_t *integrator_limit)
{
    if (axis == NULL) {
        return;
    }
    /* Reading three q16_t (32-bit) fields without a lock is safe on 32-bit
     * targets: each field is naturally aligned and read atomically. The PID
     * never updates these from inside its own context, only the tuning API
     * does, so the worst case is observing a stale snapshot, never a torn
     * single value. */
    if (kp != NULL) {
        *kp = axis->torque_controller[0].kp;
    }
    if (ki != NULL) {
        *ki = axis->torque_controller[0].ki;
    }
    if (integrator_limit != NULL) {
        *integrator_limit = axis->torque_controller[0].integrator_limit;
    }
}

esp_foc_err_t esp_foc_axis_inject_step_q16(
    esp_foc_axis_t *axis,
    q16_t amplitude,
    uint32_t duration_ms)
{
#if !defined(CONFIG_ESP_FOC_INJECTION_ENABLE)
    (void)axis; (void)amplitude; (void)duration_ms;
    return ESP_FOC_ERR_AXIS_INVALID_STATE;
#else
    if (axis == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    q16_t ts = axis->torque_controller[0].dt;
    if (ts <= 0) {
        return ESP_FOC_ERR_TIMESTEP_TOO_SMALL;
    }
    esp_foc_critical_enter();
    esp_foc_injection_step_setup(&axis->injection, amplitude, duration_ms, ts);
    esp_foc_critical_leave();
    return ESP_FOC_OK;
#endif
}

esp_foc_err_t esp_foc_axis_inject_chirp_q16(
    esp_foc_axis_t *axis,
    q16_t amplitude,
    q16_t freq_start_hz,
    q16_t freq_end_hz,
    uint32_t duration_ms)
{
#if !defined(CONFIG_ESP_FOC_INJECTION_ENABLE)
    (void)axis; (void)amplitude; (void)freq_start_hz;
    (void)freq_end_hz; (void)duration_ms;
    return ESP_FOC_ERR_AXIS_INVALID_STATE;
#else
    if (axis == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if (freq_start_hz <= 0 || freq_end_hz <= 0) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    q16_t ts = axis->torque_controller[0].dt;
    if (ts <= 0) {
        return ESP_FOC_ERR_TIMESTEP_TOO_SMALL;
    }
    esp_foc_critical_enter();
    esp_foc_injection_chirp_setup(&axis->injection, amplitude,
                                  freq_start_hz, freq_end_hz,
                                  duration_ms, ts);
    esp_foc_critical_leave();
    return ESP_FOC_OK;
#endif
}

esp_foc_err_t esp_foc_axis_inject_stop(esp_foc_axis_t *axis)
{
    if (axis == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    esp_foc_critical_enter();
    esp_foc_injection_disable(&axis->injection);
    esp_foc_critical_leave();
    return ESP_FOC_OK;
}
