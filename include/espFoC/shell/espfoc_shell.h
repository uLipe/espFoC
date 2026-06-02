/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * Console REPL on ESP_CONSOLE UART — see doc/SHELL.md
 */

#pragma once

#include <stdint.h>
#include "espFoC/esp_foc_axis.h"
#include "espFoC/esp_foc_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_foc_err_t espfoc_shell_register_axis(uint8_t axis_id, esp_foc_axis_t *axis);
void espfoc_shell_start(void);

#ifdef __cplusplus
}
#endif
