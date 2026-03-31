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
};
