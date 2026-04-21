/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

/**
 * @file esp_foc_tuner.h
 * @brief Transport-agnostic backend for live PI tuning of axes.
 *
 * Mirrors the scope module pattern: the tuner exposes weak callbacks for
 * I/O so the user can attach UART, USB-CDC, BLE, TCP, etc. The tuner core
 * is a stateless request handler: a host opens a session and sends typed
 * read / write / exec requests to inspect or update gains and motor params.
 *
 * Wire protocol (little-endian):
 *
 *   request:  [op:u8][id:u16][axis:u8][len:u16][payload:len bytes]
 *   response: [status:u8][len:u16][payload:len bytes]
 *
 * status follows esp_foc_err_t (truncated to int8). Application code is
 * free to wrap this in framing (length-prefix, CRC, etc.) inside its
 * weak send/recv callbacks.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <sdkconfig.h>
#include "espFoC/esp_foc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_FOC_TUNER_MAX_AXES 4
#define ESP_FOC_TUNER_MAX_PAYLOAD 16

typedef enum {
    ESP_FOC_TUNER_OP_READ  = 0x01,
    ESP_FOC_TUNER_OP_WRITE = 0x02,
    ESP_FOC_TUNER_OP_EXEC  = 0x03,
} esp_foc_tuner_op_t;

typedef enum {
    /* Read-only parameters */
    ESP_FOC_TUNER_PARAM_KP_Q16          = 0x0010, /* read q16 */
    ESP_FOC_TUNER_PARAM_KI_Q16          = 0x0011, /* read q16 */
    ESP_FOC_TUNER_PARAM_INT_LIM_Q16     = 0x0012, /* read q16 */
    ESP_FOC_TUNER_PARAM_V_MAX_Q16       = 0x0013, /* read q16 */
    ESP_FOC_TUNER_PARAM_AXIS_STATE      = 0x0040, /* read u8 bitmap */
    ESP_FOC_TUNER_PARAM_AXIS_LAST_ERR   = 0x0041, /* read i8 (esp_foc_err_t) */

    /* Write: gain swap (atomic) */
    ESP_FOC_TUNER_WRITE_KP_Q16          = 0x0020, /* write q16 */
    ESP_FOC_TUNER_WRITE_KI_Q16          = 0x0021, /* write q16 */
    ESP_FOC_TUNER_WRITE_INT_LIM_Q16     = 0x0022, /* write q16 */

    /* Write: tuner-driven motion targets (only honored while override active) */
    ESP_FOC_TUNER_WRITE_TARGET_ID_Q16   = 0x0050, /* write q16 (current ref) */
    ESP_FOC_TUNER_WRITE_TARGET_IQ_Q16   = 0x0051, /* write q16 (current ref) */
    ESP_FOC_TUNER_WRITE_TARGET_UD_Q16   = 0x0052, /* write q16 (voltage ff) */
    ESP_FOC_TUNER_WRITE_TARGET_UQ_Q16   = 0x0053, /* write q16 (voltage ff) */

    /* Commands (exec) */
    ESP_FOC_TUNER_CMD_RECOMPUTE_GAINS   = 0x0080, /* [R_q16,L_q16,bw_q16] */
    ESP_FOC_TUNER_CMD_OVERRIDE_ON       = 0x00A0, /* no payload */
    ESP_FOC_TUNER_CMD_OVERRIDE_OFF      = 0x00A1, /* no payload */
} esp_foc_tuner_id_t;

/* Bitmap returned by ESP_FOC_TUNER_PARAM_AXIS_STATE. */
#define ESP_FOC_AXIS_STATE_INITIALIZED    (1u << 0)
#define ESP_FOC_AXIS_STATE_ALIGNED        (1u << 1)
#define ESP_FOC_AXIS_STATE_RUNNING        (1u << 2)
#define ESP_FOC_AXIS_STATE_TUNER_OVERRIDE (1u << 3)

/**
 * @brief Register an axis under a stable id so the host can address it.
 *
 * @param axis_id arbitrary id chosen by the user (0..ESP_FOC_TUNER_MAX_AXES-1)
 * @param axis    pointer to a fully initialized axis (NULL detaches)
 * @return ESP_FOC_OK or ESP_FOC_ERR_INVALID_ARG
 */
esp_foc_err_t esp_foc_tuner_attach_axis(uint8_t axis_id, esp_foc_axis_t *axis);

/**
 * @brief Handle a single request synchronously.
 *
 * Intended to be called from a transport layer (UART task, BLE callback,
 * etc.) once a complete request frame has been parsed.
 *
 * @param axis_id     axis previously registered with esp_foc_tuner_attach_axis
 * @param op          read / write / exec
 * @param id          parameter or command id
 * @param payload     opaque request payload (may be NULL when len==0)
 * @param payload_len length of payload in bytes
 * @param response    output buffer for the response payload
 * @param response_len in: capacity of response, out: bytes written
 * @return ESP_FOC_OK or specific error
 */
esp_foc_err_t esp_foc_tuner_handle_request(
    uint8_t axis_id,
    esp_foc_tuner_op_t op,
    esp_foc_tuner_id_t id,
    const void *payload,
    size_t payload_len,
    void *response,
    size_t *response_len);

/**
 * @brief Drive the tuner with raw bytes from a transport reader.
 *
 * Bridges (UART, USB-CDC, ...) implement esp_foc_tuner_recv_callback() to
 * pull bytes off the wire and feed them here one at a time. When a full
 * tuner-channel frame is reassembled, it is dispatched to handle_request()
 * and the encoded response is sent back via esp_foc_tuner_send_callback().
 *
 * The reactor maintains internal state across calls; do not interleave
 * bytes coming from different physical buses.
 *
 * Application-level frame layout (inside the link payload):
 *
 *   request : [op:u8][id:u16 LE][axis:u8][cmd_payload:N]
 *   response: [status:i8][seq:u8][resp_payload:N]
 *
 * The seq byte is echoed from the request frame's header so the host can
 * match responses to outstanding requests.
 */
void esp_foc_tuner_process_byte(uint8_t byte);

/**
 * @brief Reset the reactor's link decoder. Useful after a transport
 * disconnect / reconnect so a half-parsed frame does not leak across.
 */
void esp_foc_tuner_reactor_reset(void);

/* Optional weak hooks for the user-defined transport layer.
 * The default no-op implementations let the protocol be exercised directly
 * via esp_foc_tuner_handle_request() without any I/O. */
void esp_foc_tuner_init_bus_callback(void);
int  esp_foc_tuner_recv_callback(uint8_t *buf, size_t max);
void esp_foc_tuner_send_callback(const uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif
