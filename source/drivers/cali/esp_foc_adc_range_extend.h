/*
 * SPDX-FileCopyrightText: Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdbool.h>
#include "hal/adc_types.h"

bool esp_foc_adc_range_extend_supported(void);
int esp_foc_adc_range_extend_mv(adc_atten_t atten, int mv);
