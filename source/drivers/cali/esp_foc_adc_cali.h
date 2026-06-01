/*
 * SPDX-FileCopyrightText: 2019-2023 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * Ported for espFoC isensor ADC LUT init (no esp_adc runtime dependency).
 */
#pragma once

#include <stdint.h>
#include "hal/adc_types.h"

typedef struct {
    uint8_t term_num;
    const uint64_t (*coeff)[2];
    const int32_t (*sign);
} esp_foc_cali_chars_second_step_t;

void esp_foc_curve_fitting_get_second_step_coeff(adc_unit_t unit,
                                                 adc_atten_t atten,
                                                 esp_foc_cali_chars_second_step_t *ctx);

bool esp_foc_adc_cali_curve_raw_to_mv(adc_unit_t unit,
                                      adc_channel_t channel,
                                      adc_atten_t atten,
                                      int raw,
                                      int *mv_out);

bool esp_foc_adc_cali_line_raw_to_mv(adc_unit_t unit,
                                     adc_channel_t channel,
                                     adc_atten_t atten,
                                     int raw,
                                     int *mv_out);
