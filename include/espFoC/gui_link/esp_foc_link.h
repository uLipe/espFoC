/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

/**
 * @file esp_foc_link.h
 * @brief Wire-level framing for the espFoC tuner / scope protocols.
 *
 * Provides a transport-agnostic codec used by every bridge (USB-CDC,
 * UART, TCP, ...). One frame on the wire looks like:
 *
 *   offset  bytes  field
 *   -----   -----  -----
 *   0       1      sync = 0xA5
 *   1       2      payload_len (LE, max ESP_FOC_LINK_MAX_PAYLOAD)
 *   3       1      channel  (see esp_foc_link_channel_t)
 *   4       1      seq      (8-bit rolling, free for the caller)
 *   5       N      payload  (N = payload_len)
 *   5+N     2      crc16    (CRC-16/CCITT over channel..payload, LE)
 *
 * The codec itself is allocation-free, no-floating-point and re-entrant
 * provided each call site uses its own state struct.
 *
 * Channels multiplex the same physical bus between the tuner request /
 * response stream, the scope sample stream, and an optional log stream.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_FOC_LINK_SYNC          0xA5u
#define ESP_FOC_LINK_HEADER_BYTES  5u   /* sync + len(2) + channel + seq */
#define ESP_FOC_LINK_TRAILER_BYTES 2u   /* crc16 */
#define ESP_FOC_LINK_MAX_PAYLOAD   256u
#define ESP_FOC_LINK_MAX_FRAME     \
    (ESP_FOC_LINK_HEADER_BYTES + ESP_FOC_LINK_MAX_PAYLOAD + ESP_FOC_LINK_TRAILER_BYTES)

typedef enum {
    ESP_FOC_LINK_CH_TUNER  = 0x01,  /* tuner request / response */
    ESP_FOC_LINK_CH_SCOPE  = 0x02,  /* scope: binary Q16, see SCOPE v1 in esp_foc_scope.c */
    ESP_FOC_LINK_CH_LOG    = 0x03,  /* free-form log lines */
} esp_foc_link_channel_t;

typedef enum {
    ESP_FOC_LINK_OK              = 0,
    ESP_FOC_LINK_ERR_INVALID_ARG = -1,
    ESP_FOC_LINK_ERR_TOO_BIG     = -2,
    ESP_FOC_LINK_ERR_NEED_MORE   = -3,
    ESP_FOC_LINK_ERR_BAD_SYNC    = -4,
    ESP_FOC_LINK_ERR_BAD_CRC     = -5,
} esp_foc_link_status_t;

/**
 * @brief Encode a single frame.
 *
 * @param channel       channel identifier (see esp_foc_link_channel_t)
 * @param seq           caller-managed sequence byte
 * @param payload       payload bytes (may be NULL when payload_len == 0)
 * @param payload_len   payload length in bytes (max ESP_FOC_LINK_MAX_PAYLOAD)
 * @param out           output buffer
 * @param out_cap       output buffer capacity
 * @param out_written   on success, total bytes written to @p out
 *
 * @return ESP_FOC_LINK_OK or a negative status.
 */
esp_foc_link_status_t esp_foc_link_encode(
    esp_foc_link_channel_t channel,
    uint8_t seq,
    const uint8_t *payload, size_t payload_len,
    uint8_t *out, size_t out_cap,
    size_t *out_written);

/* --- Streaming decoder ------------------------------------------------- */

typedef struct {
    /* Internal: never touched by the caller after init. */
    uint8_t state;
    uint8_t channel;
    uint8_t seq;
    uint16_t payload_len;
    uint16_t payload_received;
    uint16_t crc_recv;
    uint16_t crc_calc;
    uint8_t buf[ESP_FOC_LINK_MAX_PAYLOAD];
} esp_foc_link_decoder_t;

/* Initialise / reset a decoder before first use or after a fatal error. */
void esp_foc_link_decoder_reset(esp_foc_link_decoder_t *dec);

/**
 * @brief Push a single byte into the streaming decoder.
 *
 * Returns ESP_FOC_LINK_OK when a full frame has been parsed (the caller
 * then reads the channel/seq/payload from the corresponding accessors
 * and resets the decoder). Returns ESP_FOC_LINK_ERR_NEED_MORE when more
 * bytes are required, or one of the negative statuses on framing errors;
 * any negative status implicitly resets the decoder.
 */
esp_foc_link_status_t esp_foc_link_decoder_push(
    esp_foc_link_decoder_t *dec,
    uint8_t byte);

/* Accessors for the most recently completed frame. Valid only after
 * esp_foc_link_decoder_push() returned ESP_FOC_LINK_OK and before the
 * next call to push(). */
uint8_t        esp_foc_link_decoder_channel(const esp_foc_link_decoder_t *dec);
uint8_t        esp_foc_link_decoder_seq(const esp_foc_link_decoder_t *dec);
const uint8_t *esp_foc_link_decoder_payload(const esp_foc_link_decoder_t *dec);
size_t         esp_foc_link_decoder_payload_len(const esp_foc_link_decoder_t *dec);

/* Exposed for golden tests and host-side mirror. CRC-16/CCITT, poly
 * 0x1021, init 0xFFFF, no reflection, no xor-out. */
uint16_t esp_foc_link_crc16(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
