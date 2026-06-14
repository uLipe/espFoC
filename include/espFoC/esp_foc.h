/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <sdkconfig.h>
#include "espFoC/utils/modulator.h"
#include "espFoC/drivers/esp_foc_inverter.h"
#include "espFoC/drivers/esp_foc_encoder.h"
#include "espFoC/osal/os_interface.h"
#include "espFoC/esp_foc_err.h"
#include "espFoC/esp_foc_axis.h"
#include "espFoC/calibration/esp_foc_calibration.h"

esp_foc_err_t esp_foc_initialize_axis(esp_foc_axis_t *axis,
                                      esp_foc_inverter_t *inverter,
                                      esp_foc_encoder_t *encoder,
                                      esp_foc_motor_control_settings_t settings);

typedef struct {
    esp_foc_motor_control_settings_t motor;
    bool calibrate_isensor_at_init;
    q16_t bench_theta_e;
} esp_foc_axis_bench_config_t;

esp_foc_err_t esp_foc_initialize_axis_bench(esp_foc_axis_t *axis,
                                            esp_foc_inverter_t *inverter,
                                            const esp_foc_axis_bench_config_t *config);

esp_foc_err_t esp_foc_bench_arm(esp_foc_axis_t *axis);
esp_foc_err_t esp_foc_bench_disarm(esp_foc_axis_t *axis);
esp_foc_err_t esp_foc_bench_step(esp_foc_axis_t *axis);
void esp_foc_axis_refresh_encoder_q16_scales(esp_foc_axis_t *axis);
esp_foc_err_t esp_foc_align_axis(esp_foc_axis_t *axis);
esp_foc_err_t esp_foc_run(esp_foc_axis_t *axis);
esp_foc_err_t esp_foc_stop(esp_foc_axis_t *axis);
esp_foc_err_t esp_foc_set_regulation_callback(esp_foc_axis_t *axis,
                                              esp_foc_motor_regulation_callback_t callback);

void esp_foc_axis_apply_bypass_current_loop_gains(esp_foc_axis_t *axis);

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
