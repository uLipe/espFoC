/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#pragma once

#include <stdint.h>
#include "espFoC/utils/esp_foc_q16.h"

#if defined(CONFIG_ESP_FOC_SCOPE)
/** Q16 µs: last FOC PWM-ISR hot-path duration (written from ISR). */
extern q16_t esp_foc_debug_scope_hot_path_dt_us_q16;
#endif

void esp_foc_scope_initalize(void);

/* Task-context push. Snapshots wired channels into the active write
 * buffer; wakes the scope daemon on rollover via OSAL notifications. */
void esp_foc_scope_data_push(void);

/* ISR-context push. Same wire format and ping-pong protocol but uses
 * the ISR-safe notification path so it is safe from the PWM ISR.
 * Stores raw Q16 values; float conversion is deferred to the daemon. */
void esp_foc_scope_data_push_from_isr(void);

int esp_foc_scope_add_channel(const q16_t *data_to_wire, int channel_number);

void esp_foc_init_bus_callback(void);
void esp_foc_send_buffer_callback(const uint8_t *buffer, int size);
