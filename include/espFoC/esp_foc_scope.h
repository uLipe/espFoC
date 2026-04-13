/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#pragma once

#include <stdint.h>
#include "espFoC/utils/esp_foc_q16.h"

void esp_foc_scope_initalize(void);
void esp_foc_scope_data_push(void);
int esp_foc_scope_add_channel(const q16_t *data_to_wire, int channel_number);

void esp_foc_init_bus_callback(void);
void esp_foc_send_buffer_callback(const uint8_t *buffer, int size);
