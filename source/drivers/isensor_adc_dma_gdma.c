/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * GDMA RX backend for ADC digi (ESP32-S3, C3, C6, …) using gdma_hal + gdma_ll only.
 */

#include <string.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "esp_attr.h"
#include "hal/gdma_hal.h"
#include "hal/gdma_hal_ahb.h"
#include "hal/gdma_ll.h"
#include "hal/gdma_types.h"
#include "soc/gdma_periph.h"
#include "soc/lldesc.h"
#include "isensor_adc_internal.h"

#define ISENSOR_ADC_GDMA_GROUP   0
#define ISENSOR_ADC_GDMA_PAIR    2
#define ISENSOR_ADC_GDMA_RX_EOF  GDMA_LL_EVENT_RX_SUC_EOF

static gdma_hal_context_t s_gdma_hal;
static isensor_adc_dma_ctx_t *s_ctx;
static intr_handle_t s_intr;
static bool s_inited;

static void IRAM_ATTR isensor_adc_gdma_isr(void *arg)
{
    (void)arg;
    gdma_hal_context_t *hal = &s_gdma_hal;
    const int ch = ISENSOR_ADC_GDMA_PAIR;

    uint32_t st = hal->read_intr_status(hal, ch, GDMA_CHANNEL_DIRECTION_RX, true);
    if ((st & ISENSOR_ADC_GDMA_RX_EOF) == 0) {
        return;
    }
    hal->clear_intr(hal, ch, GDMA_CHANNEL_DIRECTION_RX, ISENSOR_ADC_GDMA_RX_EOF);

    if (s_ctx != NULL) {
        s_ctx->eof_desc_addr = (intptr_t)hal->get_eof_desc_addr(hal, ch, GDMA_CHANNEL_DIRECTION_RX, true);
        if (s_ctx->on_done != NULL) {
            s_ctx->on_done(s_ctx->on_done_arg);
        }
    }
}

esp_err_t isensor_adc_dma_init(isensor_adc_dma_ctx_t *ctx,
                               isensor_adc_dma_done_fn_t on_done,
                               void *user)
{
    if (ctx == NULL || on_done == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_inited) {
        ctx->on_done = on_done;
        ctx->on_done_arg = user;
        s_ctx = ctx;
        return ESP_OK;
    }

    int __DECLARE_RCC_ATOMIC_ENV __attribute__((unused));

    gdma_ll_enable_bus_clock(ISENSOR_ADC_GDMA_GROUP, true);
    gdma_ll_reset_register(ISENSOR_ADC_GDMA_GROUP);

    gdma_hal_config_t hal_cfg = {
        .group_id = ISENSOR_ADC_GDMA_GROUP,
    };
    gdma_ahb_hal_init(&s_gdma_hal, &hal_cfg);

    const int ch = ISENSOR_ADC_GDMA_PAIR;
    s_gdma_hal.reset(&s_gdma_hal, ch, GDMA_CHANNEL_DIRECTION_RX);
    s_gdma_hal.connect_peri(&s_gdma_hal, ch, GDMA_CHANNEL_DIRECTION_RX,
                            GDMA_TRIG_PERIPH_ADC, 0);
    s_gdma_hal.set_strategy(&s_gdma_hal, ch, GDMA_CHANNEL_DIRECTION_RX,
                            true, false, false);
    s_gdma_hal.enable_burst(&s_gdma_hal, ch, GDMA_CHANNEL_DIRECTION_RX, false, false);
    s_gdma_hal.enable_intr(&s_gdma_hal, ch, GDMA_CHANNEL_DIRECTION_RX,
                           ISENSOR_ADC_GDMA_RX_EOF, true);

    int irq = gdma_periph_signals.groups[ISENSOR_ADC_GDMA_GROUP].pairs[ch].rx_irq_id;
    esp_err_t err = esp_intr_alloc(irq, ESP_INTR_FLAG_IRAM, isensor_adc_gdma_isr, NULL, &s_intr);
    if (err != ESP_OK) {
        return err;
    }

    ctx->on_done = on_done;
    ctx->on_done_arg = user;
    s_ctx = ctx;
    s_inited = true;
    return ESP_OK;
}

esp_err_t isensor_adc_dma_deinit(isensor_adc_dma_ctx_t *ctx)
{
    (void)ctx;
    if (!s_inited) {
        return ESP_OK;
    }
    isensor_adc_dma_stop(ctx);
    if (s_intr != NULL) {
        esp_intr_free(s_intr);
        s_intr = NULL;
    }
    s_ctx = NULL;
    s_inited = false;
    return ESP_OK;
}

esp_err_t isensor_adc_dma_start(isensor_adc_dma_ctx_t *ctx, dma_descriptor_t *desc)
{
    (void)ctx;
    if (desc == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_gdma_hal.start_with_desc(&s_gdma_hal, ISENSOR_ADC_GDMA_PAIR,
                               GDMA_CHANNEL_DIRECTION_RX, (intptr_t)desc);
    return ESP_OK;
}

esp_err_t isensor_adc_dma_stop(isensor_adc_dma_ctx_t *ctx)
{
    (void)ctx;
    if (!s_inited) {
        return ESP_OK;
    }
    s_gdma_hal.stop(&s_gdma_hal, ISENSOR_ADC_GDMA_PAIR, GDMA_CHANNEL_DIRECTION_RX);
    return ESP_OK;
}

esp_err_t isensor_adc_dma_reset(isensor_adc_dma_ctx_t *ctx)
{
    (void)ctx;
    if (!s_inited) {
        return ESP_OK;
    }
    s_gdma_hal.reset(&s_gdma_hal, ISENSOR_ADC_GDMA_PAIR, GDMA_CHANNEL_DIRECTION_RX);
    return ESP_OK;
}
