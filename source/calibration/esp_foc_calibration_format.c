/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include "espFoC/calibration/esp_foc_calibration.h"

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

void esp_foc_calibration_pack_pole_pairs(esp_foc_calibration_data_t *d,
                                        int32_t motor_pole_pairs)
{
    if (d == NULL) {
        return;
    }
    if (motor_pole_pairs < 1) {
        motor_pole_pairs = 0;
    } else if (motor_pole_pairs > 64) {
        motor_pole_pairs = 64;
    }
    {
        uint32_t u = (uint32_t)motor_pole_pairs;
        d->reserved[8] = (uint8_t)(u & 0xFFu);
        d->reserved[9] = (uint8_t)((u >> 8) & 0xFFu);
        d->reserved[10] = (uint8_t)((u >> 16) & 0xFFu);
        d->reserved[11] = (uint8_t)((u >> 24) & 0xFFu);
    }
}

int32_t esp_foc_calibration_get_pole_pairs(const esp_foc_calibration_data_t *d)
{
    if (d == NULL) {
        return 0;
    }
    uint32_t u = (uint32_t)d->reserved[8] | ((uint32_t)d->reserved[9] << 8) |
                 ((uint32_t)d->reserved[10] << 16) |
                 ((uint32_t)d->reserved[11] << 24);
    return (int32_t)u;
}
