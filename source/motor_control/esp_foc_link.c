/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include <string.h>
#include "espFoC/esp_foc_link.h"

/* CRC-16/CCITT, polynomial 0x1021, init 0xFFFF, no reflection, no xor-out.
 * Implemented bit-by-bit so the static footprint stays at zero (no LUT).
 * Average frame is small enough that the throughput cost is negligible
 * at typical link rates. */
uint16_t esp_foc_link_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    if (data == NULL) {
        return crc;
    }
    for (size_t i = 0; i < len; ++i) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

esp_foc_link_status_t esp_foc_link_encode(
    esp_foc_link_channel_t channel,
    uint8_t seq,
    const uint8_t *payload, size_t payload_len,
    uint8_t *out, size_t out_cap,
    size_t *out_written)
{
    if (out == NULL || out_written == NULL) {
        return ESP_FOC_LINK_ERR_INVALID_ARG;
    }
    if (payload_len > ESP_FOC_LINK_MAX_PAYLOAD) {
        return ESP_FOC_LINK_ERR_TOO_BIG;
    }
    if (payload == NULL && payload_len > 0) {
        return ESP_FOC_LINK_ERR_INVALID_ARG;
    }

    size_t total = ESP_FOC_LINK_HEADER_BYTES + payload_len + ESP_FOC_LINK_TRAILER_BYTES;
    if (out_cap < total) {
        return ESP_FOC_LINK_ERR_TOO_BIG;
    }

    out[0] = ESP_FOC_LINK_SYNC;
    out[1] = (uint8_t)(payload_len & 0xFFu);
    out[2] = (uint8_t)((payload_len >> 8) & 0xFFu);
    out[3] = (uint8_t)channel;
    out[4] = seq;
    if (payload_len > 0) {
        memcpy(&out[ESP_FOC_LINK_HEADER_BYTES], payload, payload_len);
    }

    /* CRC covers channel, seq and payload (everything past sync+len). */
    uint16_t crc = esp_foc_link_crc16(&out[3], 2u + payload_len);
    out[ESP_FOC_LINK_HEADER_BYTES + payload_len + 0] = (uint8_t)(crc & 0xFFu);
    out[ESP_FOC_LINK_HEADER_BYTES + payload_len + 1] = (uint8_t)((crc >> 8) & 0xFFu);

    *out_written = total;
    return ESP_FOC_LINK_OK;
}

/* --- Streaming decoder ------------------------------------------------- */

enum {
    DEC_WAIT_SYNC = 0,
    DEC_LEN_LO,
    DEC_LEN_HI,
    DEC_CHANNEL,
    DEC_SEQ,
    DEC_PAYLOAD,
    DEC_CRC_LO,
    DEC_CRC_HI,
    DEC_DONE,
};

void esp_foc_link_decoder_reset(esp_foc_link_decoder_t *dec)
{
    if (dec == NULL) {
        return;
    }
    dec->state = DEC_WAIT_SYNC;
    dec->channel = 0;
    dec->seq = 0;
    dec->payload_len = 0;
    dec->payload_received = 0;
    dec->crc_recv = 0;
    dec->crc_calc = 0xFFFFu;
}

/* Update the running CRC with a single byte. */
static void crc16_update(uint16_t *crc, uint8_t byte)
{
    *crc ^= ((uint16_t)byte) << 8;
    for (int b = 0; b < 8; ++b) {
        if (*crc & 0x8000u) {
            *crc = (uint16_t)((*crc << 1) ^ 0x1021u);
        } else {
            *crc = (uint16_t)(*crc << 1);
        }
    }
}

esp_foc_link_status_t esp_foc_link_decoder_push(
    esp_foc_link_decoder_t *dec,
    uint8_t byte)
{
    if (dec == NULL) {
        return ESP_FOC_LINK_ERR_INVALID_ARG;
    }

    /* If a previous push completed a frame, reset before parsing new bytes. */
    if (dec->state == DEC_DONE) {
        esp_foc_link_decoder_reset(dec);
    }

    switch (dec->state) {
    case DEC_WAIT_SYNC:
        if (byte != ESP_FOC_LINK_SYNC) {
            return ESP_FOC_LINK_ERR_NEED_MORE;
        }
        dec->state = DEC_LEN_LO;
        return ESP_FOC_LINK_ERR_NEED_MORE;

    case DEC_LEN_LO:
        dec->payload_len = byte;
        dec->state = DEC_LEN_HI;
        return ESP_FOC_LINK_ERR_NEED_MORE;

    case DEC_LEN_HI:
        dec->payload_len |= ((uint16_t)byte) << 8;
        if (dec->payload_len > ESP_FOC_LINK_MAX_PAYLOAD) {
            esp_foc_link_decoder_reset(dec);
            return ESP_FOC_LINK_ERR_TOO_BIG;
        }
        dec->state = DEC_CHANNEL;
        return ESP_FOC_LINK_ERR_NEED_MORE;

    case DEC_CHANNEL:
        dec->channel = byte;
        crc16_update(&dec->crc_calc, byte);
        dec->state = DEC_SEQ;
        return ESP_FOC_LINK_ERR_NEED_MORE;

    case DEC_SEQ:
        dec->seq = byte;
        crc16_update(&dec->crc_calc, byte);
        if (dec->payload_len == 0) {
            dec->state = DEC_CRC_LO;
        } else {
            dec->state = DEC_PAYLOAD;
        }
        return ESP_FOC_LINK_ERR_NEED_MORE;

    case DEC_PAYLOAD:
        dec->buf[dec->payload_received++] = byte;
        crc16_update(&dec->crc_calc, byte);
        if (dec->payload_received >= dec->payload_len) {
            dec->state = DEC_CRC_LO;
        }
        return ESP_FOC_LINK_ERR_NEED_MORE;

    case DEC_CRC_LO:
        dec->crc_recv = byte;
        dec->state = DEC_CRC_HI;
        return ESP_FOC_LINK_ERR_NEED_MORE;

    case DEC_CRC_HI:
        dec->crc_recv |= ((uint16_t)byte) << 8;
        if (dec->crc_recv != dec->crc_calc) {
            esp_foc_link_decoder_reset(dec);
            return ESP_FOC_LINK_ERR_BAD_CRC;
        }
        dec->state = DEC_DONE;
        return ESP_FOC_LINK_OK;

    default:
        esp_foc_link_decoder_reset(dec);
        return ESP_FOC_LINK_ERR_BAD_SYNC;
    }
}

uint8_t esp_foc_link_decoder_channel(const esp_foc_link_decoder_t *dec)
{
    return dec ? dec->channel : 0;
}

uint8_t esp_foc_link_decoder_seq(const esp_foc_link_decoder_t *dec)
{
    return dec ? dec->seq : 0;
}

const uint8_t *esp_foc_link_decoder_payload(const esp_foc_link_decoder_t *dec)
{
    return dec ? dec->buf : NULL;
}

size_t esp_foc_link_decoder_payload_len(const esp_foc_link_decoder_t *dec)
{
    return dec ? dec->payload_len : 0;
}
