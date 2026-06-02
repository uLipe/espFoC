/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * ESPF scope stream frame v1 — see doc/PIVOT_SCOPE_SHELL.md
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_FOC_STREAM_MAGIC_HDR_0 0x45u /* 'E' */
#define ESP_FOC_STREAM_MAGIC_HDR_1 0x46u /* 'F' */
#define ESP_FOC_STREAM_MAGIC_HDR_2 0x50u /* 'P' */
#define ESP_FOC_STREAM_MAGIC_HDR_3 0x46u /* 'F' */

#define ESP_FOC_STREAM_MAGIC_FTR_0 0x46u /* 'F' */
#define ESP_FOC_STREAM_MAGIC_FTR_1 0x4Eu /* 'N' */
#define ESP_FOC_STREAM_MAGIC_FTR_2 0x44u /* 'D' */
#define ESP_FOC_STREAM_MAGIC_FTR_3 0x21u /* '!' */

#define ESP_FOC_STREAM_HEADER_BYTES  8u
#define ESP_FOC_STREAM_FOOTER_BYTES  4u
#define ESP_FOC_STREAM_FIXED_BYTES   (ESP_FOC_STREAM_HEADER_BYTES + ESP_FOC_STREAM_FOOTER_BYTES)

static inline size_t esp_foc_stream_frame_total_bytes(uint16_t n_channels)
{
    return ESP_FOC_STREAM_FIXED_BYTES + (size_t)n_channels * 4u;
}

#ifdef __cplusplus
}
#endif
