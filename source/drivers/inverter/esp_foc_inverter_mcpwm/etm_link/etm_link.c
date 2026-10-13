/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#include "esp_foc_driver_soc_gate.h"

#include "etm_link.h"

#include "esp_check.h"
#include "hal/etm_ll.h"
#include "soc/soc_etm_struct.h"
#include "soc/soc_etm_source.h"
#include "soc/soc_caps.h"

#if !SOC_GDMA_SUPPORT_ETM
#error "espFoC: ADC block stop needs GDMA ETM events (SOC_GDMA_SUPPORT_ETM)"
#endif

static const char *TAG = "foc_etm";

esp_err_t esp_foc_etm_link_init(esp_foc_etm_link_t *link,
                                const esp_foc_etm_link_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(link != NULL && cfg != NULL, ESP_ERR_INVALID_ARG, TAG, "null");
    ESP_RETURN_ON_FALSE(cfg->mcpwm_timer >= 0 && cfg->mcpwm_timer < 3, ESP_ERR_INVALID_ARG, TAG, "timer");
    ESP_RETURN_ON_FALSE(cfg->etm_channel >= 0 &&
                            (uint32_t)cfg->etm_channel < SOC_ETM_CHANNELS_PER_GROUP,
                        ESP_ERR_INVALID_ARG, TAG, "chan");
    ESP_RETURN_ON_FALSE(cfg->stop_channel >= 0 &&
                            (uint32_t)cfg->stop_channel < SOC_ETM_CHANNELS_PER_GROUP &&
                            cfg->stop_channel != cfg->etm_channel,
                        ESP_ERR_INVALID_ARG, TAG, "stop chan");
    ESP_RETURN_ON_FALSE(cfg->dma_rx_channel >= 0 && cfg->dma_rx_channel <= 2,
                        ESP_ERR_INVALID_ARG, TAG, "dma ch");

    etm_ll_enable_bus_clock(0, true);
    etm_ll_reset_register(0);

    uint32_t event_id = (uint32_t)(MCPWM_EVT_TIMER0_TEZ + cfg->mcpwm_timer);
    uint32_t task_id = (uint32_t)ADC_TASK_START0;
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
    etm_ll_channel_set_event(&SOC_ETM, stop,
                             cfg->stop_per_conversion
                                 ? (uint32_t)ADC_EVT_CONV_CMPLT0
                                 : (uint32_t)(GDMA_EVT_IN_SUC_EOF_CH0 + cfg->dma_rx_channel));
    etm_ll_channel_set_task(&SOC_ETM, stop, (uint32_t)ADC_TASK_STOP0);
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
