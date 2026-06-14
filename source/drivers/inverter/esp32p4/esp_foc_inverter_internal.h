/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "hal/dma_types.h"
#include "soc/soc_caps.h"
#include "esp_foc_isensor_adc_private.h"

#define ESP_FOC_ISENSOR_ADC_PATTERN_HZ      80000
#define ESP_FOC_ISENSOR_ADC_NUM_CHANNELS    2

typedef struct isensor_adc_dma_ctx isensor_adc_dma_ctx_t;

typedef void (*isensor_adc_dma_done_fn_t)(void *user);

struct isensor_adc_dma_ctx {
    isensor_adc_dma_done_fn_t on_done;
    void *on_done_arg;
    volatile intptr_t eof_desc_addr;
};

esp_err_t isensor_adc_dma_init(isensor_adc_dma_ctx_t *ctx,
                               isensor_adc_dma_done_fn_t on_done,
                               void *user);
esp_err_t isensor_adc_dma_deinit(isensor_adc_dma_ctx_t *ctx);
esp_err_t isensor_adc_dma_start(isensor_adc_dma_ctx_t *ctx, dma_descriptor_t *desc);
esp_err_t isensor_adc_dma_stop(isensor_adc_dma_ctx_t *ctx);
esp_err_t isensor_adc_dma_reset(isensor_adc_dma_ctx_t *ctx);

typedef enum {
    ESP_FOC_ISENSOR_ADC_STATE_IDLE = 0,
    ESP_FOC_ISENSOR_ADC_STATE_BUSY,
} esp_foc_isensor_adc_state_t;

#if SOC_ETM_SUPPORTED
#include "esp_etm.h"
esp_err_t isensor_adc_etm_connect(esp_etm_event_handle_t mcpwm_event);
esp_err_t isensor_adc_etm_enable(bool enable);
void isensor_adc_etm_disconnect(void);
#endif
