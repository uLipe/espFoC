/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#pragma once

#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/esp_foc_axis.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_ESP_FOC_SCOPE)
extern q16_t esp_foc_debug_scope_hot_path_dt_us_q16;
#endif

void esp_foc_scope_initalize(void);

/** Axis used to gate TX: stream only when state is RUNNING. */
void esp_foc_scope_bind_axis(esp_foc_axis_t *axis);

void esp_foc_scope_data_push(void);
void esp_foc_scope_data_push_from_isr(void);

int esp_foc_scope_add_channel(const q16_t *data_to_wire, int channel_number);

#ifdef __cplusplus
}
#endif
