/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * Line-fitting ADC calibration for ESP32 / ESP32-S2 (init-time LUT only).
 */

#include <stdint.h>
#include "sdkconfig.h"
#include "hal/adc_types.h"
#include "hal/efuse_ll.h"
#include "esp_foc_adc_cali.h"

#if (CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2) && SOC_ADC_CALIB_SCHEME_LINE_FITTING_SUPPORTED

#define LIN_COEFF_A_SCALE   65536
#define LIN_COEFF_A_ROUND   (LIN_COEFF_A_SCALE / 2)

static bool line_coeffs_ready;
static uint32_t s_coeff_a;
static int32_t s_coeff_b;

static bool line_coeffs_init(adc_unit_t unit, adc_atten_t atten)
{
    if (line_coeffs_ready) {
        return true;
    }

    uint32_t coeff_a = 0;
    int32_t coeff_b = 0;
    if (efuse_ll_get_adc_calib_ver() == 0) {
        return false;
    }

    if (unit == ADC_UNIT_1) {
        coeff_a = efuse_ll_get_adc_calib_cali_val(atten);
        coeff_b = efuse_ll_get_adc_calib_offset(atten);
    } else {
        coeff_a = efuse_ll_get_adc2_calib_cali_val(atten);
        coeff_b = efuse_ll_get_adc2_calib_offset(atten);
    }

    if (coeff_a == 0) {
        return false;
    }

    s_coeff_a = coeff_a;
    s_coeff_b = coeff_b;
    line_coeffs_ready = true;
    return true;
}

bool esp_foc_adc_cali_line_raw_to_mv(adc_unit_t unit,
                                     adc_channel_t channel,
                                     adc_atten_t atten,
                                     int raw,
                                     int *mv_out)
{
    (void)channel;
    if (mv_out == NULL || !line_coeffs_init(unit, atten)) {
        return false;
    }

    *mv_out = (int)(((int64_t)raw * s_coeff_a + LIN_COEFF_A_ROUND) / LIN_COEFF_A_SCALE) + s_coeff_b;
    return true;
}

#else

bool esp_foc_adc_cali_line_raw_to_mv(adc_unit_t unit,
                                     adc_channel_t channel,
                                     adc_atten_t atten,
                                     int raw,
                                     int *mv_out)
{
    (void)unit;
    (void)channel;
    (void)atten;
    (void)raw;
    (void)mv_out;
    return false;
}

#endif
