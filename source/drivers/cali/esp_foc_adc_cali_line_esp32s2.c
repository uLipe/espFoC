/*
 * SPDX-FileCopyrightText: 2019-2023 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * Line-fitting ADC calibration for ESP32-S2 (init-time LUT).
 */

#include <stdint.h>
#include <stdbool.h>
#include "sdkconfig.h"
#include "hal/adc_types.h"
#include "esp_efuse_rtc_table.h"
#include "esp_foc_adc_cali.h"

#if CONFIG_IDF_TARGET_ESP32S2 && SOC_ADC_CALIB_SCHEME_LINE_FITTING_SUPPORTED

static const int coeff_a_scaling = 65536;
static const int coeff_b_scaling = 1024;

typedef struct {
    int adc_calib_high;
    int adc_calib_low;
} esp_foc_s2_calib_ver1_t;

typedef struct {
    int adc_calib_high;
    int adc_calib_high_voltage;
} esp_foc_s2_calib_ver2_t;

typedef struct {
    char version_num;
    adc_unit_t unit_id;
    adc_atten_t atten_level;
    union {
        esp_foc_s2_calib_ver1_t ver1;
        esp_foc_s2_calib_ver2_t ver2;
    } efuse_data;
} esp_foc_s2_calib_info_t;

typedef struct {
    adc_unit_t unit;
    adc_atten_t atten;
    bool ready;
    uint32_t coeff_a;
    uint32_t coeff_b;
} esp_foc_line_ctx_t;

static esp_foc_line_ctx_t s_line;

static void characterize_using_two_point(adc_atten_t atten, uint32_t high, uint32_t low,
                                         uint32_t *coeff_a, uint32_t *coeff_b)
{
    static const uint32_t v_high[] = {600, 800, 1000, 2000};
    static const uint32_t v_low = 250;
    *coeff_a = (uint32_t)(coeff_a_scaling * (v_high[atten] - v_low) / (high - low));
    *coeff_b = (uint32_t)(coeff_b_scaling * (v_low * high - v_high[atten] * low) / (high - low));
}

static bool prepare_calib_data(adc_unit_t unit_id, adc_atten_t atten, esp_foc_s2_calib_info_t *out)
{
    int version_num = esp_efuse_rtc_table_read_calib_version();
    if (version_num != 1 && version_num != 2) {
        return false;
    }

    out->version_num = (char)version_num;
    out->unit_id = unit_id;
    out->atten_level = atten;

    int tag;
    switch (version_num) {
    case 1:
        tag = esp_efuse_rtc_table_get_tag(version_num, unit_id, atten, RTCCALIB_V1_PARAM_VLOW);
        out->efuse_data.ver1.adc_calib_low = esp_efuse_rtc_table_get_parsed_efuse_value(tag, false);
        tag = esp_efuse_rtc_table_get_tag(version_num, unit_id, atten, RTCCALIB_V1_PARAM_VHIGH);
        out->efuse_data.ver1.adc_calib_high = esp_efuse_rtc_table_get_parsed_efuse_value(tag, false);
        break;
    case 2:
        tag = esp_efuse_rtc_table_get_tag(version_num, unit_id, atten, RTCCALIB_V2_PARAM_VHIGH);
        out->efuse_data.ver2.adc_calib_high = esp_efuse_rtc_table_get_parsed_efuse_value(tag, false);
        switch (atten) {
        case ADC_ATTEN_DB_0:       out->efuse_data.ver2.adc_calib_high_voltage = 600; break;
        case ADC_ATTEN_DB_2_5:     out->efuse_data.ver2.adc_calib_high_voltage = 800; break;
        case ADC_ATTEN_DB_6:       out->efuse_data.ver2.adc_calib_high_voltage = 1000; break;
        case ADC_ATTEN_DB_12:      out->efuse_data.ver2.adc_calib_high_voltage = 2000; break;
        default: return false;
        }
        break;
    default:
        return false;
    }
    return true;
}

static bool line_ctx_prepare(adc_unit_t unit, adc_atten_t atten)
{
    if (s_line.ready && s_line.unit == unit && s_line.atten == atten) {
        return true;
    }

    esp_foc_s2_calib_info_t info = {0};
    if (!prepare_calib_data(unit, atten, &info)) {
        return false;
    }

    switch (info.version_num) {
    case 1:
        characterize_using_two_point(atten,
                                     (uint32_t)info.efuse_data.ver1.adc_calib_high,
                                     (uint32_t)info.efuse_data.ver1.adc_calib_low,
                                     &s_line.coeff_a, &s_line.coeff_b);
        break;
    case 2:
        s_line.coeff_a = (uint32_t)(coeff_a_scaling * info.efuse_data.ver2.adc_calib_high_voltage
                                    / info.efuse_data.ver2.adc_calib_high);
        s_line.coeff_b = 0;
        break;
    default:
        return false;
    }

    s_line.unit = unit;
    s_line.atten = atten;
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

    *mv_out = (int)(raw * s_line.coeff_a / (coeff_a_scaling / coeff_b_scaling) + s_line.coeff_b) / coeff_b_scaling;
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
