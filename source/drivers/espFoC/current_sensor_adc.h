/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#pragma once

#include "espFoC/esp_foc.h"
#include "hal/adc_types.h"
#include "espFoC/esp_foc_err.h"
#include "soc/soc_caps.h"

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

typedef enum {
    ESP_FOC_ISENSOR_ADC_MCPWM_EVT_TIMER_TEZ = 0,
    ESP_FOC_ISENSOR_ADC_MCPWM_EVT_TIMER_TEP,
} esp_foc_isensor_adc_mcpwm_event_t;

typedef struct {
    uint8_t mcpwm_timer;
    esp_foc_isensor_adc_mcpwm_event_t event;
} esp_foc_isensor_adc_etm_config_t;

esp_foc_isensor_t *isensor_adc_new(esp_foc_isensor_adc_config_t *config);

esp_foc_err_t esp_foc_isensor_adc_set_trigger(esp_foc_isensor_t *isensor,
                                              esp_foc_isensor_adc_trigger_t mode);

#if SOC_ETM_SUPPORTED
esp_foc_err_t esp_foc_isensor_adc_set_etm_source(esp_foc_isensor_t *isensor,
                                                 const esp_foc_isensor_adc_etm_config_t *cfg);
#endif
