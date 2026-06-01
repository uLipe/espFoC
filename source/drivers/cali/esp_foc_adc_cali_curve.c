/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * Curve-fitting ADC calibration (ported from ESP-IDF, init-time LUT only).
 */

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include "esp_err.h"
#include "sdkconfig.h"
#include "esp_private/adc_share_hw_ctrl.h"
#include "esp_foc_adc_cali.h"

#if SOC_ADC_CALIB_SCHEME_CURVE_FITTING_SUPPORTED
#include "esp_efuse_rtc_calib.h"

static const int coeff_a_scaling = 65536;

typedef struct {
    uint32_t voltage;
    uint32_t digi;
} esp_foc_adc_calib_data_ver1_t;

typedef struct {
    char version_num;
    adc_unit_t unit_id;
    adc_atten_t atten;
    esp_foc_adc_calib_data_ver1_t ver1;
} esp_foc_adc_calib_info_t;

typedef struct {
    uint32_t coeff_a;
    uint32_t coeff_b;
} esp_foc_cali_chars_first_step_t;

typedef struct {
    adc_unit_t unit_id;
    adc_channel_t chan;
    adc_atten_t atten;
    esp_foc_cali_chars_first_step_t first;
    esp_foc_cali_chars_second_step_t second;
} esp_foc_cali_curve_ctx_t;

static esp_foc_cali_curve_ctx_t s_curve_ctx;

static int32_t get_reading_error(uint64_t v_cali_1, const esp_foc_cali_chars_second_step_t *param)
{
    if (v_cali_1 == 0 || param->term_num == 0) {
        return 0;
    }

    uint8_t term_num = param->term_num;
    int32_t error = 0;
    uint64_t coeff = 0;
    uint64_t variable[3];
    uint64_t term[3];

    variable[0] = 1;
    coeff = param->coeff[0][0];
    term[0] = variable[0] * coeff / param->coeff[0][1];
    error = (int32_t)term[0] * param->sign[0];

    for (int i = 1; i < term_num; i++) {
        variable[i] = variable[i - 1] * v_cali_1;
        coeff = param->coeff[i][0];
        term[i] = variable[i] * coeff;
        term[i] = term[i] / param->coeff[i][1];
        error += (int32_t)term[i] * param->sign[i];
    }

    return error;
}

static void get_first_step_reference_point(int version_num,
                                           adc_unit_t unit_id,
                                           adc_atten_t atten,
                                           esp_foc_adc_calib_info_t *calib_info)
{
    calib_info->version_num = (char)version_num;
    calib_info->unit_id = unit_id;
    calib_info->atten = atten;

    uint32_t voltage = 0;
    uint32_t digi = 0;
    esp_err_t ret = esp_efuse_rtc_calib_get_cal_voltage(version_num, unit_id, (int)atten, &digi, &voltage);
    if (ret != ESP_OK) {
        calib_info->ver1.voltage = 0;
        calib_info->ver1.digi = 0;
        return;
    }
    calib_info->ver1.voltage = voltage;
    calib_info->ver1.digi = digi;
}

static void calc_first_step_coefficients(const esp_foc_adc_calib_info_t *parsed,
                                         esp_foc_cali_curve_ctx_t *ctx)
{
    if (parsed->ver1.digi == 0) {
        ctx->first.coeff_a = coeff_a_scaling;
        ctx->first.coeff_b = 0;
        return;
    }
    ctx->first.coeff_a = coeff_a_scaling * parsed->ver1.voltage / parsed->ver1.digi;
    ctx->first.coeff_b = 0;
}

static bool curve_ctx_prepare(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten)
{
    uint32_t ver = esp_efuse_rtc_calib_get_ver();
    if (ver < ESP_EFUSE_ADC_CALIB_VER_MIN || ver > ESP_EFUSE_ADC_CALIB_VER_MAX) {
        return false;
    }

    esp_foc_adc_calib_info_t info = {0};
    get_first_step_reference_point((int)ver, unit, atten, &info);
    if (info.ver1.digi == 0) {
        return false;
    }

    calc_first_step_coefficients(&info, &s_curve_ctx);
    esp_foc_curve_fitting_get_second_step_coeff(unit, atten, &s_curve_ctx.second);
    s_curve_ctx.unit_id = unit;
    s_curve_ctx.chan = channel;
    s_curve_ctx.atten = atten;
    return true;
}

bool esp_foc_adc_cali_curve_raw_to_mv(adc_unit_t unit,
                                      adc_channel_t channel,
                                      adc_atten_t atten,
                                      int raw,
                                      int *mv_out)
{
    if (mv_out == NULL) {
        return false;
    }
    if (s_curve_ctx.unit_id != unit || s_curve_ctx.chan != channel || s_curve_ctx.atten != atten) {
        if (!curve_ctx_prepare(unit, channel, atten)) {
            return false;
        }
    }

#if SOC_ADC_CALIB_CHAN_COMPENS_SUPPORTED
    int chan_comp = adc_get_hw_calibration_chan_compens(unit, channel, atten);
    raw -= chan_comp;
    int max_val = (1 << SOC_ADC_RTC_MAX_BITWIDTH) - 1;
    if (raw < 0) {
        raw = 0;
    }
    if (raw > max_val) {
        raw = max_val;
    }
#endif

    uint64_t v_cali_1 = (uint64_t)raw * s_curve_ctx.first.coeff_a / coeff_a_scaling
                        + s_curve_ctx.first.coeff_b;
    int32_t error = get_reading_error(v_cali_1, &s_curve_ctx.second);
    *mv_out = (int32_t)v_cali_1 - error;
    return true;
}

#else

bool esp_foc_adc_cali_curve_raw_to_mv(adc_unit_t unit,
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
