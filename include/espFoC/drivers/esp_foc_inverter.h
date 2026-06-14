/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#pragma once

#include <stdint.h>
#include "espFoC/utils/esp_foc_q16.h"

typedef void (*esp_foc_inverter_callback_t)(void *argument);

typedef struct {
    q16_t iu_axis_0;
    q16_t iv_axis_0;
    q16_t iw_axis_0;
    q16_t iu_axis_1;
    q16_t iv_axis_1;
    q16_t iw_axis_1;
} esp_foc_inverter_isensor_values_t;

typedef struct esp_foc_inverter_s esp_foc_inverter_t;

struct esp_foc_inverter_s {
    void (*set_inverter_callback)(esp_foc_inverter_t *self,
                                  esp_foc_inverter_callback_t callback,
                                  void *argument);
    q16_t (*get_dc_link_voltage)(esp_foc_inverter_t *self);
    void (*set_duties)(esp_foc_inverter_t *self,
                       q16_t duty_a, q16_t duty_b, q16_t duty_c);
    uint32_t (*get_inverter_pwm_rate)(esp_foc_inverter_t *self);
    void (*enable)(esp_foc_inverter_t *self);
    void (*disable)(esp_foc_inverter_t *self);

    void (*fetch_isensors)(esp_foc_inverter_t *self,
                           esp_foc_inverter_isensor_values_t *values);
    void (*sample_isensors)(esp_foc_inverter_t *self);
    void (*calibrate_isensors)(esp_foc_inverter_t *self, int calibration_rounds);
    void (*set_filter_cutoff)(esp_foc_inverter_t *self, float fc_hz, float fs_hz);
    void (*set_publish_targets)(esp_foc_inverter_t *self,
                                q16_t *i_alpha_target,
                                q16_t *i_beta_target,
                                q16_t *i_u_target,
                                q16_t *i_v_target);
};
