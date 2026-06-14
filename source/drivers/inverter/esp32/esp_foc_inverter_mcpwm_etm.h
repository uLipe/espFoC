/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#pragma once

#include "sdkconfig.h"
#include "soc/soc_caps.h"
#include "espFoC/esp_foc_err.h"
#include "espFoC/drivers/esp_foc_inverter.h"

#if SOC_ETM_SUPPORTED

#include "driver/mcpwm_prelude.h"
#include "esp_etm.h"

typedef enum {
    INVERTER_MCPWM_ADC_TRIGGER_CENTER = 0,
    INVERTER_MCPWM_ADC_TRIGGER_PEAK,
} inverter_mcpwm_adc_trigger_t;

typedef struct {
    mcpwm_cmpr_handle_t cmpr_center;
    mcpwm_cmpr_handle_t cmpr_peak;
    esp_etm_event_handle_t event_center;
    esp_etm_event_handle_t event_peak;
    bool inited;
} inverter_mcpwm_etm_t;

esp_err_t inverter_mcpwm_etm_init(inverter_mcpwm_etm_t *etm,
                                  mcpwm_oper_handle_t oper,
                                  uint32_t period_ticks);

void inverter_mcpwm_etm_register(esp_foc_inverter_t *iface, inverter_mcpwm_etm_t *etm);

esp_foc_err_t inverter_mcpwm_connect_adc_etm(esp_foc_inverter_t *inverter,
                                             inverter_mcpwm_adc_trigger_t trigger);

#endif /* SOC_ETM_SUPPORTED */
