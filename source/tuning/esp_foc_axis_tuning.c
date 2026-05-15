/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include <sdkconfig.h>
#include <stdint.h>
#include "espFoC/tuning/esp_foc_axis_tuning.h"
#include "espFoC/esp_foc_controls.h"

static void apply_gains_locked(esp_foc_axis_t *axis,
                               q16_t kp, q16_t ki, q16_t integrator_limit)
{
    for (int i = 0; i < 2; ++i) {
        axis->torque_controller[i].kp = kp;
        axis->torque_controller[i].ki = ki;
        axis->torque_controller[i].integrator_limit = integrator_limit;
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

    q16_t loop_dt_q16 = axis->torque_controller[0].dt;
    if (loop_dt_q16 <= 0) {
        return ESP_FOC_ERR_TIMESTEP_TOO_SMALL;
    }
    uint32_t loop_ts_us = (uint32_t)(((int64_t)loop_dt_q16 * 1000000LL
                                      + (int64_t)Q16_ONE / 2)
                                     / (int64_t)Q16_ONE);
    if (loop_ts_us == 0) {
        return ESP_FOC_ERR_TIMESTEP_TOO_SMALL;
    }

    esp_foc_pi_design_input_t in = {
        .motor_r_ohm  = motor_r_ohm,
        .motor_l_h    = motor_l_h,
        .loop_ts_us   = loop_ts_us,
        .bandwidth_hz = bandwidth_hz,
        .v_max        = ESP_FOC_VPU_ONE_Q16,
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
