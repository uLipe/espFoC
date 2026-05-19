/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#pragma once

#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

void esp_foc_scope_initalize(void);

/** When false, captured samples are not transmitted (saves bandwidth). */
void esp_foc_scope_set_stream_enabled(bool enabled);
bool esp_foc_scope_stream_enabled(void);

void esp_foc_scope_data_push(void);
void esp_foc_scope_data_push_from_isr(void);

int esp_foc_scope_add_channel(const q16_t *data_to_wire, int channel_number);

#ifdef __cplusplus
}
#endif
