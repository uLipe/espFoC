/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * Line-fitting ADC calibration for ESP32-C2 (init-time LUT).
 */

#include <stdint.h>
#include <stdbool.h>
#include "sdkconfig.h"
#include "hal/adc_types.h"
#include "esp_err.h"
#include "esp_efuse_rtc_calib.h"
#include "esp_foc_adc_cali.h"

#if CONFIG_IDF_TARGET_ESP32C2 && SOC_ADC_CALIB_SCHEME_LINE_FITTING_SUPPORTED

static const int coeff_a_scaling = 65536;

typedef struct {
    adc_unit_t unit;
    adc_atten_t atten;
    bool ready;
    uint32_t coeff_a;
    uint32_t coeff_b;
} esp_foc_line_ctx_t;

static esp_foc_line_ctx_t s_line;

static bool line_ctx_prepare(adc_unit_t unit, adc_atten_t atten)
{
    if (s_line.ready && s_line.unit == unit && s_line.atten == atten) {
        return true;
    }

    uint32_t ver = esp_efuse_rtc_calib_get_ver();
    if (ver < ESP_EFUSE_ADC_CALIB_VER_MIN || ver > ESP_EFUSE_ADC_CALIB_VER_MAX) {
        return false;
    }

    uint32_t voltage_mv = 0;
    uint32_t digi_val = 0;
    if (esp_efuse_rtc_calib_get_cal_voltage(ver, unit, (int)atten, &digi_val, &voltage_mv) != ESP_OK
            || digi_val == 0) {
        return false;
    }

    s_line.unit = unit;
    s_line.atten = atten;
    s_line.coeff_a = (uint32_t)(coeff_a_scaling * voltage_mv / digi_val);
    s_line.coeff_b = 0;
    s_line.ready = true;
    return true;
}

bool esp_foc_adc_cali_line_raw_to_mv(adc_unit_t unit, adc_channel_t channel,
                                     adc_atten_t atten, int raw, int *mv_out)
{
    (void)channel;
    if (mv_out == NULL || !line_ctx_prepare(unit, atten)) {
        return false;
    }

    *mv_out = (int)(raw * s_line.coeff_a / coeff_a_scaling + s_line.coeff_b);
    return true;
}

#else

bool esp_foc_adc_cali_line_raw_to_mv(adc_unit_t unit, adc_channel_t channel,
                                     adc_atten_t atten, int raw, int *mv_out)
{
    (void)unit; (void)channel; (void)atten; (void)raw; (void)mv_out;
    return false;
}

#endif
