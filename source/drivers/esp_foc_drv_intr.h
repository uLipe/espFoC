/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "esp_intr_alloc.h"

/*
 * esp_intr_alloc_intrstatus on the CPU chosen by CONFIG_ESP_FOC_CORE_ID.
 * The IDF ties an interrupt to the CPU that allocates it and has no CPU
 * argument, so from another CPU the allocation is run there over IPC.
 * Task context. status_reg / status_mask 0 means a plain esp_intr_alloc.
 */
esp_err_t esp_foc_drv_intr_alloc(int source, int flags, uint32_t status_reg,
                                 uint32_t status_mask, intr_handler_t handler,
                                 void *arg, intr_handle_t *out);
