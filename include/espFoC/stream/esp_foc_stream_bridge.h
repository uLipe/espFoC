/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * Scope stream TX bridge — see doc/PIVOT_SCOPE_SHELL.md
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*esp_foc_stream_send_fn)(const uint8_t *data, size_t len);

/** Install HW and register the low-level send hook (TX only). Idempotent. */
void esp_foc_stream_bridge_init(void);

/** Send a complete ESPF frame (built by esp_foc_scope). */
void esp_foc_stream_bridge_send_frame(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
