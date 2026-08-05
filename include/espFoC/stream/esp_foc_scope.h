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

/** Start scope daemon + stream bridge (idempotent). Called from axis init. */
void esp_foc_scope_initalize(void);

/** True when daemon task is running (tests / diagnostics). */
bool esp_foc_scope_is_initialized(void);

/** Wire standard 17ch axis_shell map; requires CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS=17. */
esp_foc_err_t esp_foc_scope_wire_axis(esp_foc_axis_t *axis);

/** Custom channel maps (test apps): bind axis for TX gate, then add_channel. */
void esp_foc_scope_bind_axis(esp_foc_axis_t *axis);

void esp_foc_scope_data_push(void);
void esp_foc_scope_data_push_from_isr(void);

int esp_foc_scope_add_channel(const q16_t *data_to_wire, int channel_number);

#ifdef __cplusplus
}
#endif
