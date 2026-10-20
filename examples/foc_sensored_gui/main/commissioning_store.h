/*
 * SPDX-FileCopyrightText: 2026 Felipe Neves
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/*
 * Commissioning results kept in NVS, one blob per step. Each blob carries a
 * format version, its size, a hash of the board configuration it was
 * measured on and a CRC32 of the payload; a blob that fails any of them is
 * reported and never applied.
 *
 * Flash writes stall every task for milliseconds, the encoder fetch
 * included: call save/erase with the bridge off.
 */

/* Opens the store; board_hash identifies the board configuration. */
esp_err_t commissioning_store_init(uint32_t board_hash);

/*
 * @return ESP_OK, ESP_ERR_NOT_FOUND (never saved), ESP_ERR_INVALID_VERSION
 *         (other format or size), ESP_ERR_INVALID_STATE (other board
 *         configuration) or ESP_ERR_INVALID_CRC.
 */
esp_err_t commissioning_store_load(const char *key, void *data, size_t size);
esp_err_t commissioning_store_save(const char *key, const void *data, size_t size);
/* ESP_OK also when the key was not there. */
esp_err_t commissioning_store_erase(const char *key);

/* CRC32 over a block, to build the board hash from. */
uint32_t commissioning_store_crc(uint32_t crc, const void *data, size_t size);

/* Short reason for a load result, for the log. */
const char *commissioning_store_reason(esp_err_t err);
