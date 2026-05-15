/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#pragma once

#include "espFoC/esp_foc_err.h"
#include "espFoC/esp_foc_axis.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/esp_foc_design_mpz.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_foc_err_t esp_foc_axis_retune_current_pi_q16(
    esp_foc_axis_t *axis,
    q16_t motor_r_ohm,
    q16_t motor_l_h,
    q16_t bandwidth_hz);

esp_foc_err_t esp_foc_axis_set_current_pi_gains_q16(
    esp_foc_axis_t *axis,
    q16_t kp,
    q16_t ki,
    q16_t integrator_limit);

void esp_foc_axis_get_current_pi_gains_q16(
    const esp_foc_axis_t *axis,
    q16_t *kp,
    q16_t *ki,
    q16_t *integrator_limit);

#ifdef __cplusplus
}
#endif
