/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#pragma once

#include <stdint.h>
#include "espFoC/utils/esp_foc_q16.h"

void esp_foc_scope_initalize(void);

/* Task-context push. Snapshots wired channels into the active write
 * buffer; wakes the scope daemon via xTaskNotifyGive on rollover. */
void esp_foc_scope_data_push(void);

/* ISR-context push. Same wire format and ping-pong protocol but uses
 * the FromISR notification variant so it is safe to call from inside
 * an interrupt handler (e.g. the FOC hot path inside the PWM ISR).
 * Stores raw Q16 values; float conversion is deferred to the daemon. */
void esp_foc_scope_data_push_from_isr(void);

int esp_foc_scope_add_channel(const q16_t *data_to_wire, int channel_number);

void esp_foc_init_bus_callback(void);
void esp_foc_send_buffer_callback(const uint8_t *buffer, int size);
