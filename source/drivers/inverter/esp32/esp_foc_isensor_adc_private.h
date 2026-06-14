/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#pragma once

#include "espFoC/drivers/esp_foc_inverter.h"
#include "espFoC/esp_foc_err.h"
#include "hal/adc_types.h"

typedef struct {
    adc_channel_t channels[2];
    adc_unit_t unit;
    float amp_gain;
    float shunt_resistance;
} esp_foc_isensor_adc_config_t;

typedef struct esp_foc_isensor_s esp_foc_isensor_t;
typedef void (*isensor_callback_t)(void *arg);

typedef struct {
    q16_t iu_axis_0;
    q16_t iv_axis_0;
    q16_t iw_axis_0;
    q16_t iu_axis_1;
    q16_t iv_axis_1;
    q16_t iw_axis_1;
} isensor_values_t;

struct esp_foc_isensor_s {
    void (*fetch_isensors)(esp_foc_isensor_t *self, isensor_values_t *values);
    void (*sample_isensors)(esp_foc_isensor_t *self);
    void (*calibrate_isensors)(esp_foc_isensor_t *self, int calibration_rounds);
    void (*set_isensor_callback)(esp_foc_isensor_t *self, isensor_callback_t cb, void *param);
    void (*set_filter_cutoff)(esp_foc_isensor_t *self, float fc_hz, float fs_hz);
    void (*set_publish_targets)(esp_foc_isensor_t *self,
                                q16_t *i_alpha_target,
                                q16_t *i_beta_target,
                                q16_t *i_u_target,
                                q16_t *i_v_target);
};

esp_foc_isensor_t *isensor_adc_new(esp_foc_isensor_adc_config_t *config);
void isensor_adc_set_software_trigger(esp_foc_isensor_t *isensor);
void isensor_adc_set_etm_trigger(esp_foc_isensor_t *isensor);
