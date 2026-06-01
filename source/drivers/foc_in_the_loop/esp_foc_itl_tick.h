/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#pragma once

#include <stdint.h>

#include "espFoC/esp_foc_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*esp_foc_itl_tick_cb_t)(void *arg);

esp_foc_err_t esp_foc_itl_tick_start(esp_foc_itl_tick_cb_t cb, void *arg, uint32_t rate_hz);
void esp_foc_itl_tick_stop(void);

#ifdef __cplusplus
}
#endif
