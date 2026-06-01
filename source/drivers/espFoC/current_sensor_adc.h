/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#pragma once

#include "espFoC/esp_foc.h"
#include "hal/adc_types.h"
#include "espFoC/esp_foc_err.h"

typedef struct {
    adc_channel_t channels[2];
    adc_unit_t unit;
    float amp_gain;
    float shunt_resistance;
} esp_foc_isensor_adc_config_t;

typedef enum {
    ESP_FOC_ISENSOR_ADC_TRIG_SOFTWARE = 0,
    ESP_FOC_ISENSOR_ADC_TRIG_ETM,
} esp_foc_isensor_adc_trigger_t;

esp_foc_isensor_t *isensor_adc_new(esp_foc_isensor_adc_config_t *config);

esp_foc_err_t esp_foc_isensor_adc_set_trigger(esp_foc_isensor_t *isensor,
                                              esp_foc_isensor_adc_trigger_t mode);
