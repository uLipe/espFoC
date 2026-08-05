/*
 * SPDX-FileCopyrightText: Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * Curve-fitting coefficients ported from ESP-IDF esp_adc/esp32h2/curve_fitting_coefficients.c
 */

#include <stdint.h>
#include <assert.h>
#include "sdkconfig.h"
#include "esp_foc_adc_cali.h"

#if SOC_ADC_CALIB_SCHEME_CURVE_FITTING_SUPPORTED
#include "esp_efuse_rtc_calib.h"


#define COEFF_VERSION_NUM  1 // Currently H2 has one versions of curve calibration schemes
#define COEFF_GROUP_NUM    4
#define TERM_MAX           3

/**
 * @note Error Calculation
 * Coefficients for calculating the reading voltage error.
 * Four sets of coefficients for atten0 ~ atten3 respectively.
 *
 * For each item, first element is the Coefficient, second element is the Multiple. (Coefficient / Multiple) is the real coefficient.
 *
 * @note {0,0} stands for unused item
 * @note In case of the overflow, these coefficients are recorded as Absolute Value
 * @note For atten0 ~ 2, error = (K0 * X^0) + (K1 * X^1)
 * @note For atten3, error = (K0 * X^0) + (K1 * X^1) + (K2 * X^2)
 * @note Above formula is rewritten from the original documentation, please note that the coefficients are re-ordered.
 */
const static uint64_t adc1_error_coef_atten[COEFF_VERSION_NUM][COEFF_GROUP_NUM][TERM_MAX][2] = {
    /* Coefficients of calibration version 1 */
    {
        {{5081991760658888,     1e16}, {7858995318513,    1e19}, {0, 1}},                    //atten0
        {{8359230818901277,     1e16}, {9025419089179,    1e19}, {0, 1}},                    //atten1
        {{1165668771581976,     1e15}, {8294679249061,    1e19}, {0, 1}},                    //atten2
        {{3637329628677273,     1e16}, {19607259738935,   1e18}, {7871689227,   1e16}},      //atten3
    },
};

/**
 * Term sign
 */
const static int32_t adc1_error_sign[COEFF_VERSION_NUM][COEFF_GROUP_NUM][TERM_MAX] = {
    /* Coefficient sign of calibration version 1 */
    {
        {-1,  1,  1}, //atten0
        {-1,  1,  1}, //atten1
        {-1,  1,  1}, //atten2
        {-1, -1,  1}, //atten3
    },
};

void esp_foc_curve_fitting_get_second_step_coeff(adc_unit_t unit, adc_atten_t atten, esp_foc_cali_chars_second_step_t *ctx)
{
    uint32_t adc_calib_ver = esp_efuse_rtc_calib_get_ver();
    assert((adc_calib_ver >= ESP_EFUSE_ADC_CALIB_VER_MIN) &&
           (adc_calib_ver <= ESP_EFUSE_ADC_CALIB_VER_MAX));

    ctx->term_num = 3;
    ctx->coeff = adc1_error_coef_atten[VER2IDX(adc_calib_ver)][atten];
    ctx->sign = adc1_error_sign[VER2IDX(adc_calib_ver)][atten];
}

#else

void esp_foc_curve_fitting_get_second_step_coeff(adc_unit_t unit, adc_atten_t atten,
                                                 esp_foc_cali_chars_second_step_t *ctx)
{
    (void)unit; (void)atten;
    ctx->term_num = 0; ctx->coeff = NULL; ctx->sign = NULL;
}

#endif
