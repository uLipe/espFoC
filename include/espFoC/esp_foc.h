/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <sdkconfig.h>
#include "espFoC/utils/modulator.h"
#include "espFoC/drivers/inverter_interface.h"
#include "espFoC/drivers/current_sensor_interface.h"
#include "espFoC/drivers/rotor_sensor_interface.h"
#include "espFoC/osal/os_interface.h"
#include "espFoC/esp_foc_err.h"
#include "espFoC/esp_foc_axis.h"
#include "espFoC/gui_link/esp_foc_scope.h"
#include "espFoC/calibration/esp_foc_calibration.h"

esp_foc_err_t esp_foc_initialize_axis(esp_foc_axis_t *axis,
                                        esp_foc_inverter_t *inverter,
                                        esp_foc_rotor_sensor_t *rotor,
                                        esp_foc_isensor_t *isensor,
                                        esp_foc_motor_control_settings_t settings);
/** Recompute encoder_inv_cpr_q16 and encoder_counts_speed_to_omega_e_q16 after pp/cpr. */
void esp_foc_axis_refresh_encoder_q16_scales(esp_foc_axis_t *axis);
esp_foc_err_t esp_foc_align_axis(esp_foc_axis_t *axis);
esp_foc_err_t esp_foc_run(esp_foc_axis_t *axis);
esp_foc_err_t esp_foc_stop(esp_foc_axis_t *axis);
esp_foc_err_t esp_foc_set_regulation_callback(esp_foc_axis_t *axis,
                                              esp_foc_motor_regulation_callback_t callback);

esp_foc_err_t esp_foc_axis_set_current_loop_gains_q16(
    esp_foc_axis_t *axis,
    q16_t kp,
    q16_t ki,
    q16_t kd,
    q16_t kff,
    q16_t integrator_limit);

void esp_foc_axis_get_current_loop_gains_q16(
    const esp_foc_axis_t *axis,
    q16_t *kp,
    q16_t *ki,
    q16_t *kd,
    q16_t *kff,
    q16_t *integrator_limit);
