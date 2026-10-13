/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#include "esp_foc_driver_soc_gate.h"

#include "etm_link.h"

#include "adc_dma_sense.h"
#include "esp_check.h"
#include "esp_foc_hal_compat.h"
#include "hal/etm_ll.h"
#if ESP_FOC_ADC_SAR_BURST
#include "esp_private/periph_ctrl.h"
#include "hal/mcpwm_ll.h"
#endif
#include "soc/soc_etm_struct.h"
#include "soc/soc_etm_source.h"
#include "soc/soc_caps.h"

#if !SOC_GDMA_SUPPORT_ETM
#error "espFoC: ADC block stop needs GDMA ETM events (SOC_GDMA_SUPPORT_ETM)"
#endif

static const char *TAG = "foc_etm";

/* Reset once: a second inverter must not wipe the channels of the first. */
static bool s_etm_ready;

static void etm_bring_up(void)
{
    if (s_etm_ready) {
        return;
    }
#if ESP_FOC_ADC_SAR_BURST
    PERIPH_RCC_ATOMIC() {
        etm_ll_enable_bus_clock(0, true);
        etm_ll_reset_register(0);
        etm_ll_enable_function_clock(0, true);
    }
    etm_ll_set_clock_source(0, ETM_CLK_SRC_DEFAULT);
#else
    etm_ll_enable_bus_clock(0, true);
    etm_ll_reset_register(0);
#endif
    s_etm_ready = true;
}

esp_err_t esp_foc_etm_link_init(esp_foc_etm_link_t *link,
                                const esp_foc_etm_link_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(link != NULL && cfg != NULL, ESP_ERR_INVALID_ARG, TAG, "null");
    ESP_RETURN_ON_FALSE(cfg->mcpwm_timer >= 0 && cfg->mcpwm_timer < 3, ESP_ERR_INVALID_ARG, TAG, "timer");
    ESP_RETURN_ON_FALSE(cfg->etm_channel >= 0 &&
                            (uint32_t)cfg->etm_channel < ESP_FOC_ETM_CHANNELS,
                        ESP_ERR_INVALID_ARG, TAG, "chan");
    ESP_RETURN_ON_FALSE(cfg->stop_channel >= 0 &&
                            (uint32_t)cfg->stop_channel < ESP_FOC_ETM_CHANNELS &&
                            cfg->stop_channel != cfg->etm_channel,
                        ESP_ERR_INVALID_ARG, TAG, "stop chan");
    ESP_RETURN_ON_FALSE(cfg->dma_rx_channel >= 0 && cfg->dma_rx_channel <= 2,
                        ESP_ERR_INVALID_ARG, TAG, "dma ch");

    ESP_RETURN_ON_FALSE(cfg->adc_unit <= ESP_FOC_ADC_UNITS, ESP_ERR_INVALID_ARG, TAG, "adc unit");

    etm_bring_up();

#if ESP_FOC_ADC_SAR_BURST
    uint32_t event_id = MCPWM_LL_ETM_TIMER_EVENT_TABLE(cfg->mcpwm_group, cfg->mcpwm_timer,
                                                       MCPWM_TIMER_ETM_EVENT_TEZ);
    /*
     * Burst path: the sample timer opens a continuous burst on SAMPLEn and the
     * DMA completing the descriptor closes it on STOPn. Only IN_DONE gives one
     * descriptor per period; the DMA EOF needs a converter EOF count, and any
     * count breaks the framing (adc_dma_sense_burst.c).
     */
#if ESP_FOC_ADC_UNITS >= 2
    const bool sar2 = (cfg->adc_unit == 2u);
#else
    const bool sar2 = false;
#endif
    uint32_t task_id = sar2 ? (uint32_t)ADC_TASK_SAMPLE2 : (uint32_t)ADC_TASK_SAMPLE1;
    const uint32_t stop_event = (uint32_t)(LP_PDMA_AHB_EVT_IN_DONE_CH0 + cfg->dma_rx_channel);
    const uint32_t stop_task = sar2 ? (uint32_t)ADC_TASK_STOP2 : (uint32_t)ADC_TASK_STOP1;
#else
    uint32_t event_id = (uint32_t)(MCPWM_EVT_TIMER0_TEZ + cfg->mcpwm_timer);
    uint32_t task_id = (uint32_t)ADC_TASK_START0;
    const uint32_t stop_event = cfg->stop_per_conversion
                                    ? (uint32_t)ADC_EVT_CONV_CMPLT0
                                    : (uint32_t)(GDMA_EVT_IN_SUC_EOF_CH0 + cfg->dma_rx_channel);
    const uint32_t stop_task = (uint32_t)ADC_TASK_STOP0;
#endif
    uint32_t chan = (uint32_t)cfg->etm_channel;

    etm_ll_disable_channel(&SOC_ETM, chan);
    etm_ll_channel_set_event(&SOC_ETM, chan, event_id);
    etm_ll_channel_set_task(&SOC_ETM, chan, task_id);
    etm_ll_enable_channel(&SOC_ETM, chan);

    /*
     * The block has to end in hardware. Stopping it from the EOF ISR leaves one
     * conversion interval of margin, and that ISR shares level 3 with TEZ, so it
     * cannot run until the control callback returns: a TEZ path a microsecond
     * longer let extra conversions in, which fired extra EOFs (31.5 k/s against
     * 20 k/s TEZ) and moved the sample instant off the period centre.
     */
    uint32_t stop = (uint32_t)cfg->stop_channel;
    etm_ll_disable_channel(&SOC_ETM, stop);
    etm_ll_channel_set_event(&SOC_ETM, stop, stop_event);
    etm_ll_channel_set_task(&SOC_ETM, stop, stop_task);
    etm_ll_enable_channel(&SOC_ETM, stop);

    link->channel = cfg->etm_channel;
    link->stop_channel = cfg->stop_channel;
    link->linked = true;
    return ESP_OK;
}

void esp_foc_etm_link_deinit(esp_foc_etm_link_t *link)
{
    if (link == NULL || !link->linked) {
        return;
    }
    etm_ll_disable_channel(&SOC_ETM, (uint32_t)link->channel);
    etm_ll_disable_channel(&SOC_ETM, (uint32_t)link->stop_channel);
    link->linked = false;
}

void esp_foc_etm_link_enable(esp_foc_etm_link_t *link, bool on)
{
    if (link == NULL || !link->linked) {
        return;
    }
    if (on) {
        etm_ll_enable_channel(&SOC_ETM, (uint32_t)link->stop_channel);
        etm_ll_enable_channel(&SOC_ETM, (uint32_t)link->channel);
    } else {
        etm_ll_disable_channel(&SOC_ETM, (uint32_t)link->channel);
        etm_ll_disable_channel(&SOC_ETM, (uint32_t)link->stop_channel);
    }
}
