/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

/**
 * @file esp_foc_link_session.h
 * @brief Link session, heartbeat liveness, and scope stream gating (protocol v2).
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "espFoC/gui_link/esp_foc_link.h"
#include "espFoC/gui_link/esp_foc_tuner.h"
#include "espFoC/esp_foc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_FOC_LINK_PROTO_VER           2u
#define ESP_FOC_LINK_HEARTBEAT_PERIOD_MS 500u
#define ESP_FOC_LINK_HEARTBEAT_MISS_MAX  2u

#define ESP_FOC_HB_MSG_FW  0x01u
#define ESP_FOC_HB_MSG_ACK 0x02u

/** Feed raw bytes from the transport reader (all link channels). */
void esp_foc_link_process_byte(uint8_t byte);

void esp_foc_link_reactor_reset(void);

bool esp_foc_link_session_is_connected(void);
bool esp_foc_link_session_scope_streaming(void);

/**
 * @brief Drop session, stop scope stream, reset heartbeat state.
 * Called on DISCONNECT, host timeout, or voluntary teardown.
 */
void esp_foc_link_session_end(void);

/**
 * @brief Start the periodic heartbeat task (idempotent).
 * Called from the bridge init path.
 */
void esp_foc_link_session_start(void);

/** Unit tests: skip CONNECT/HB handshake. */
void esp_foc_link_session_force_connected(bool connected);

bool esp_foc_link_try_acquire_tx_low_prio(void);

bool esp_foc_link_session_tuner_may_dispatch(esp_foc_tuner_op_t op,
                                             esp_foc_tuner_id_t id,
                                             bool *silent_drop);

void esp_foc_link_session_on_connect(void);
void esp_foc_link_session_on_disconnect(void);
void esp_foc_link_session_on_scope_start(void);
void esp_foc_link_session_on_scope_stop(void);

#ifdef __cplusplus
}
#endif
