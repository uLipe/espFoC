/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    int mcpwm_timer; /* 0..2 */
    int etm_channel; /* 0 .. SOC_ETM_CHANNELS_PER_GROUP-1 */
    int stop_channel; /* ADC stop; must differ from etm_channel */
    int dma_rx_channel;
    bool stop_per_conversion; /* stop on each conversion done, else on GDMA RX EOF */
} esp_foc_etm_link_cfg_t;

typedef struct {
    bool linked;
    int channel;
    int stop_channel;
} esp_foc_etm_link_t;

esp_err_t esp_foc_etm_link_init(esp_foc_etm_link_t *link,
                                const esp_foc_etm_link_cfg_t *cfg);
void esp_foc_etm_link_deinit(esp_foc_etm_link_t *link);
void esp_foc_etm_link_enable(esp_foc_etm_link_t *link, bool on);
