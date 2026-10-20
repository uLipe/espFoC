/*
 * SPDX-FileCopyrightText: 2026 Felipe Neves
 * SPDX-License-Identifier: Apache-2.0
 */
#include "commissioning_store.h"

#include <stdlib.h>
#include <string.h>

#include "esp_rom_crc.h"
#include "nvs.h"
#include "nvs_flash.h"

#define STORE_NAMESPACE "espfoc_gui"
#define STORE_MAGIC 0x47434645u /* "EFCG" */
/* Bump when a stored struct changes meaning without changing size. */
#define STORE_VERSION 1u

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t size;
    uint32_t board_hash;
    uint32_t crc;
} store_header_t;

static nvs_handle_t store;
static uint32_t store_board_hash;

uint32_t commissioning_store_crc(uint32_t crc, const void *data, size_t size)
{
    return esp_rom_crc32_le(crc, (const uint8_t *)data, (uint32_t)size);
}

esp_err_t commissioning_store_init(uint32_t board_hash)
{
    esp_err_t err = nvs_flash_init();
    if ((err == ESP_ERR_NVS_NO_FREE_PAGES) || (err == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
        err = nvs_flash_erase();
        if (err == ESP_OK) {
            err = nvs_flash_init();
        }
    }
    if (err == ESP_OK) {
        err = nvs_open(STORE_NAMESPACE, NVS_READWRITE, &store);
    }
    store_board_hash = board_hash;
    return err;
}

esp_err_t commissioning_store_load(const char *key, void *data, size_t size)
{
    size_t stored = 0;
    esp_err_t err = nvs_get_blob(store, key, NULL, &stored);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_ERR_NOT_FOUND;
    }
    if (err != ESP_OK) {
        return err;
    }
    if (stored != sizeof(store_header_t) + size) {
        return ESP_ERR_INVALID_VERSION;
    }
    uint8_t *blob = malloc(stored);
    if (blob == NULL) {
        return ESP_ERR_NO_MEM;
    }
    err = nvs_get_blob(store, key, blob, &stored);
    if (err == ESP_OK) {
        store_header_t header;
        memcpy(&header, blob, sizeof(header));
        const uint8_t *payload = blob + sizeof(header);
        if ((header.magic != STORE_MAGIC) || (header.version != STORE_VERSION) ||
            (header.size != size)) {
            err = ESP_ERR_INVALID_VERSION;
        } else if (header.crc != commissioning_store_crc(0, payload, size)) {
            err = ESP_ERR_INVALID_CRC;
        } else if (header.board_hash != store_board_hash) {
            err = ESP_ERR_INVALID_STATE;
        } else {
            memcpy(data, payload, size);
        }
    }
    free(blob);
    return err;
}

esp_err_t commissioning_store_save(const char *key, const void *data, size_t size)
{
    const store_header_t header = {
        .magic = STORE_MAGIC,
        .version = STORE_VERSION,
        .size = (uint32_t)size,
        .board_hash = store_board_hash,
        .crc = commissioning_store_crc(0, data, size),
    };
    uint8_t *blob = malloc(sizeof(header) + size);
    if (blob == NULL) {
        return ESP_ERR_NO_MEM;
    }
    memcpy(blob, &header, sizeof(header));
    memcpy(blob + sizeof(header), data, size);
    /* The old copy goes first: the largest blob spans pages, and the default
     * NVS partition has no room for two of it at once. */
    esp_err_t err = commissioning_store_erase(key);
    if (err == ESP_OK) {
        err = nvs_set_blob(store, key, blob, sizeof(header) + size);
    }
    if (err == ESP_OK) {
        err = nvs_commit(store);
    }
    free(blob);
    return err;
}

esp_err_t commissioning_store_erase(const char *key)
{
    esp_err_t err = nvs_erase_key(store, key);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err == ESP_OK) {
        err = nvs_commit(store);
    }
    return err;
}

const char *commissioning_store_reason(esp_err_t err)
{
    switch (err) {
    case ESP_OK:
        return "valid";
    case ESP_ERR_NOT_FOUND:
        return "not saved";
    case ESP_ERR_INVALID_VERSION:
        return "other format";
    case ESP_ERR_INVALID_STATE:
        return "board configuration changed";
    case ESP_ERR_INVALID_CRC:
        return "corrupt";
    default:
        return esp_err_to_name(err);
    }
}
