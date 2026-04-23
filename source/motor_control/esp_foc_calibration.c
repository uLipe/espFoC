/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include <sdkconfig.h>
#include "espFoC/esp_foc_calibration.h"

void esp_foc_calibration_pack_align(esp_foc_calibration_data_t *d,
                                   uint8_t flags,
                                   uint16_t enc_zero_12b,
                                   q16_t natural_direction)
{
    if (d == NULL) {
        return;
    }
    d->reserved[0] = flags;
    d->reserved[1] = 0;
    d->reserved[2] = (uint8_t)(enc_zero_12b & 0xFFu);
    d->reserved[3] = (uint8_t)((enc_zero_12b >> 8) & 0xFFu);
    uint32_t nd = (uint32_t)(int32_t)natural_direction;
    d->reserved[4] = (uint8_t)(nd & 0xFFu);
    d->reserved[5] = (uint8_t)((nd >> 8) & 0xFFu);
    d->reserved[6] = (uint8_t)((nd >> 16) & 0xFFu);
    d->reserved[7] = (uint8_t)((nd >> 24) & 0xFFu);
}

void esp_foc_calibration_get_align(const esp_foc_calibration_data_t *d,
                                  uint8_t *flags,
                                  uint16_t *enc_zero_12b,
                                  q16_t *natural_direction)
{
    if (d == NULL) {
        return;
    }
    if (flags != NULL) {
        *flags = d->reserved[0];
    }
    if (enc_zero_12b != NULL) {
        *enc_zero_12b = (uint16_t)d->reserved[2] |
            ((uint16_t)d->reserved[3] << 8);
    }
    if (natural_direction != NULL) {
        uint32_t u = (uint32_t)d->reserved[4] |
            ((uint32_t)d->reserved[5] << 8) |
            ((uint32_t)d->reserved[6] << 16) |
            ((uint32_t)d->reserved[7] << 24);
        *natural_direction = (q16_t)u;
    }
}

#if defined(CONFIG_ESP_FOC_CALIBRATION_NVS)

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "ESP_FOC_CALIB";

#define ESPFOC_CAL_NS         "espfoc_cal"
#define ESPFOC_CAL_MAGIC      0xE5F0CC11u
#define ESPFOC_CAL_VERSION    1u
#define ESPFOC_CAL_MAX_AXES   4

/* Wire format on NVS. The payload (everything after `crc32`) is what
 * the public API exchanges as esp_foc_calibration_data_t. The header
 * stays tiny so the blob fits comfortably in a single NVS entry. */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t  version;
    uint8_t  reserved[3];
    uint32_t profile_hash;
    uint32_t crc32;
    uint16_t payload_len;
    uint16_t pad;
    esp_foc_calibration_data_t payload;
} espfoc_cal_blob_t;

/* CRC-32 (IEEE 802.3 polynomial). Bit-by-bit; calibration save/load
 * is a cold path, no LUT cost worth paying for. */
static uint32_t crc32_ieee(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            uint32_t mask = -(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

static uint32_t fnv1a_32(const char *s)
{
    uint32_t h = 0x811c9dc5u;
    while (*s) {
        h ^= (uint8_t)*s++;
        h *= 0x01000193u;
    }
    return h;
}

uint32_t esp_foc_calibration_profile_hash(void)
{
    /* Hash "<profile_name>:<version>" so bumping CONFIG_ESP_FOC_PROFILE_VERSION
     * invalidates persisted calibrations even when the profile name
     * stays the same — useful after editing the JSON in a way that
     * changes its meaning. */
    char buf[80];
    snprintf(buf, sizeof(buf), "%s:%d",
             CONFIG_ESP_FOC_MOTOR_PROFILE,
             CONFIG_ESP_FOC_PROFILE_VERSION);
    return fnv1a_32(buf);
}

static esp_foc_err_t ensure_nvs(void)
{
    /* Idempotent: nvs_flash_init returns ESP_ERR_NVS_NEW_VERSION_FOUND
     * or ESP_ERR_NVS_NO_FREE_PAGES on the first run after a partition
     * format change; both are recoverable by erasing and retrying. */
    static bool ready = false;
    if (ready) {
        return ESP_FOC_OK;
    }
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs erase, doing it");
        if (nvs_flash_erase() != ESP_OK) {
            return ESP_FOC_ERR_AXIS_INVALID_STATE;
        }
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %d", (int)err);
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }
    ready = true;
    return ESP_FOC_OK;
}

static void axis_key(uint8_t axis_id, char out[16])
{
    snprintf(out, 16, "axis_%u", axis_id);
}

esp_foc_err_t esp_foc_calibration_save(uint8_t axis_id,
                                       const esp_foc_calibration_data_t *data)
{
    if (data == NULL || axis_id >= ESPFOC_CAL_MAX_AXES) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    esp_foc_err_t e = ensure_nvs();
    if (e != ESP_FOC_OK) {
        return e;
    }
    espfoc_cal_blob_t blob = {0};
    blob.magic = ESPFOC_CAL_MAGIC;
    blob.version = ESPFOC_CAL_VERSION;
    blob.profile_hash = esp_foc_calibration_profile_hash();
    blob.payload_len = sizeof(blob.payload);
    blob.payload = *data;
    blob.crc32 = crc32_ieee((const uint8_t *)&blob.payload,
                            sizeof(blob.payload));

    nvs_handle_t h;
    if (nvs_open(ESPFOC_CAL_NS, NVS_READWRITE, &h) != ESP_OK) {
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }
    char key[16];
    axis_key(axis_id, key);
    esp_err_t err = nvs_set_blob(h, key, &blob, sizeof(blob));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs save axis %u failed: %d", axis_id, (int)err);
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }
    {
        uint8_t af;
        uint16_t enc0;
        q16_t nat_d;
        const float fcli = (data->current_filter_fc_hz != 0)
                            ? q16_to_float(data->current_filter_fc_hz) : 0.0f;
        esp_foc_calibration_get_align(data, &af, &enc0, &nat_d);
        ESP_LOGI(
            TAG,
            "axis %u: save OK  profile_hash=0x%08x  Kp=%.4f Ki=%.2f ILim=%.3f  I-lpf=%.1fHz",
            axis_id, (unsigned)blob.profile_hash,
            (double)q16_to_float(data->kp), (double)q16_to_float(data->ki),
            (double)q16_to_float(data->integrator_limit), (double)fcli);
        ESP_LOGI(
            TAG,
            "axis %u: save  motor: R=%.4f Ohm  L=%.4e H  bandwidth=%.1f Hz (audit / MPZ input)",
            axis_id, (double)q16_to_float(data->motor_r_ohm),
            (double)q16_to_float(data->motor_l_h),
            (double)q16_to_float(data->bandwidth_hz));
        {
            const char *nlab = "?";
            if (nat_d == Q16_ONE) {
                nlab = "CW";
            } else if (nat_d == Q16_MINUS_ONE) {
                nlab = "CCW";
            }
            const int offv = (af & ESP_FOC_CAL_ALIGN_FLAG_OFFSET) ? 1 : 0;
            const int dirv = (af & ESP_FOC_CAL_ALIGN_FLAG_DIR) ? 1 : 0;
            ESP_LOGI(
                TAG,
                "axis %u: save  align: off_flag=%d enc_raw=%u  dir_flag=%d nat_stored=%s",
                axis_id, offv,
                (unsigned)((af & ESP_FOC_CAL_ALIGN_FLAG_OFFSET) != 0u ? enc0 : 0u),
                dirv,
                (af & ESP_FOC_CAL_ALIGN_FLAG_DIR) != 0u ? nlab : "—");
        }
    }
    return ESP_FOC_OK;
}

esp_foc_err_t esp_foc_calibration_load(uint8_t axis_id,
                                       esp_foc_calibration_data_t *out)
{
    if (out == NULL || axis_id >= ESPFOC_CAL_MAX_AXES) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    esp_foc_err_t e = ensure_nvs();
    if (e != ESP_FOC_OK) {
        return e;
    }
    nvs_handle_t h;
    if (nvs_open(ESPFOC_CAL_NS, NVS_READONLY, &h) != ESP_OK) {
        /* Namespace doesn't exist yet — nothing to load. */
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }
    char key[16];
    axis_key(axis_id, key);
    espfoc_cal_blob_t blob = {0};
    size_t got = sizeof(blob);
    esp_err_t err = nvs_get_blob(h, key, &blob, &got);
    nvs_close(h);
    if (err != ESP_OK || got != sizeof(blob)) {
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }
    if (blob.magic != ESPFOC_CAL_MAGIC ||
        blob.version != ESPFOC_CAL_VERSION ||
        blob.payload_len != sizeof(blob.payload)) {
        ESP_LOGW(TAG, "axis %u stored blob header invalid, ignoring", axis_id);
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }
    if (crc32_ieee((const uint8_t *)&blob.payload,
                   sizeof(blob.payload)) != blob.crc32) {
        ESP_LOGW(TAG, "axis %u stored blob CRC mismatch, ignoring", axis_id);
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }
    if (blob.profile_hash != esp_foc_calibration_profile_hash()) {
        ESP_LOGW(TAG, "axis %u stored profile_hash 0x%08x != current 0x%08x — "
                      "calibration was made for a different motor, ignoring",
                 axis_id, (unsigned)blob.profile_hash,
                 (unsigned)esp_foc_calibration_profile_hash());
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }
    *out = blob.payload;
    return ESP_FOC_OK;
}

esp_foc_err_t esp_foc_calibration_erase(void)
{
    esp_foc_err_t e = ensure_nvs();
    if (e != ESP_FOC_OK) {
        return e;
    }
    nvs_handle_t h;
    if (nvs_open(ESPFOC_CAL_NS, NVS_READWRITE, &h) != ESP_OK) {
        return ESP_FOC_OK; /* nothing to erase */
    }
    esp_err_t err = nvs_erase_all(h);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err != ESP_OK) {
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }
    ESP_LOGI(TAG, "calibration namespace erased");
    return ESP_FOC_OK;
}

bool esp_foc_calibration_present(uint8_t axis_id)
{
    esp_foc_calibration_data_t scratch;
    return esp_foc_calibration_load(axis_id, &scratch) == ESP_FOC_OK;
}

#else /* CONFIG_ESP_FOC_CALIBRATION_NVS not set — stub everything. */

uint32_t esp_foc_calibration_profile_hash(void) { return 0u; }

esp_foc_err_t esp_foc_calibration_save(uint8_t axis_id,
                                       const esp_foc_calibration_data_t *data)
{
    (void)axis_id; (void)data;
    return ESP_FOC_ERR_AXIS_INVALID_STATE;
}

esp_foc_err_t esp_foc_calibration_load(uint8_t axis_id,
                                       esp_foc_calibration_data_t *out)
{
    (void)axis_id; (void)out;
    return ESP_FOC_ERR_AXIS_INVALID_STATE;
}

esp_foc_err_t esp_foc_calibration_erase(void)
{
    return ESP_FOC_ERR_AXIS_INVALID_STATE;
}

bool esp_foc_calibration_present(uint8_t axis_id)
{
    (void)axis_id;
    return false;
}

#endif
