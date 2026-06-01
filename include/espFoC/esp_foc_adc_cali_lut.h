/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "hal/adc_types.h"

#define ESP_FOC_ISENSOR_ADC_LUT_SIZE  4096

bool esp_foc_adc_cali_lut_build(adc_unit_t unit,
                                adc_channel_t channel,
                                adc_atten_t atten,
                                int16_t *lut_out,
                                unsigned lut_len);

static inline int32_t esp_foc_adc_cali_lut_apply(const int16_t *lut, int32_t raw12)
{
    if (raw12 < 0) {
        raw12 = 0;
    }
    if (raw12 >= 4096) {
        raw12 = 4095;
    }
    return (int32_t)lut[raw12];
}
