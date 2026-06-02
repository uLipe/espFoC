/*
 * SPDX-FileCopyrightText: Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * Curve-fitting coefficients ported from ESP-IDF esp_adc/esp32p4/curve_fitting_coefficients.c
 */

#include <stdint.h>
#include <assert.h>
#include "sdkconfig.h"
#include "esp_foc_adc_cali.h"

#if SOC_ADC_CALIB_SCHEME_CURVE_FITTING_SUPPORTED
#include "esp_efuse_rtc_calib.h"


#define COEFF_VERSION_NUM  1 // Currently P4 has one versions of curve calibration schemes
#define COEFF_GROUP_NUM    4
#define TERM_MAX           2

/**
 * @note Error Calculation
 * Coefficients for calculating the reading voltage error.
 * Four sets of coefficients for atten0 ~ atten3 respectively.
 *
 * For each item, first element is the Coefficient, second element is the Multiple. (Coefficient / Multiple) is the real coefficient.
 *
 * @note {0,0} stands for unused item
 * @note In case of the overflow, these coefficients are recorded as Absolute Value
 * @note For atten0 ~ 3, error = (K0 * X^0) + (K1 * X^1)
 * @note Above formula is rewritten from the original documentation, please note that the coefficients are re-ordered.
 */
const static uint64_t adc1_error_coef_atten[COEFF_VERSION_NUM][COEFF_GROUP_NUM][TERM_MAX][2] = {
    /* Coefficients of calibration version 1 */
    {
        {{7170501832480995,     1e16}, {10598497992115,   1e16}},   //atten0
        {{9960085535084866,     1e16}, {15840076608145,   1e16}},   //atten1
        {{14711053224678996,    1e16}, {1594266424857,    1e17}},   //atten2
        {{28811493455181565,    1e16}, {10082311568625,   1e16}},   //atten3
    },
};

const static uint64_t adc2_error_coef_atten[COEFF_VERSION_NUM][COEFF_GROUP_NUM][TERM_MAX][2] = {
    /* Coefficients of calibration version 1 */
    {
        {{4900967548489932,     1e16}, {5037402667913,    1e16}},   //atten0
        {{7296214814536025,     1e16}, {11021577596635,   1e16}},   //atten1
        {{10991620450220592,    1e16}, {11623930881896,   1e16}},   //atten2
        {{24421401024626730,    1e16}, {9458501263393,    1e16}},   //atten3
    },
};

/**
 * Term sign ADC1
 */
const static int32_t adc1_error_sign[COEFF_VERSION_NUM][COEFF_GROUP_NUM][TERM_MAX] = {
    /* Coefficient sign of calibration version 1 */
    {
        {-1,  1}, //atten0
        {-1,  1}, //atten1
        {-1,  1}, //atten2
        {-1,  1}, //atten3
    },
};

/**
 * Term sign ADC2
 */
const static int32_t adc2_error_sign[COEFF_VERSION_NUM][COEFF_GROUP_NUM][TERM_MAX] = {
    /* Coefficient sign of calibration version 1 */
    {
        {-1,  1}, //atten0
        {-1,  1}, //atten1
        {-1,  1}, //atten2
        {-1,  1}, //atten3
    },
};

void esp_foc_curve_fitting_get_second_step_coeff(adc_unit_t unit, adc_atten_t atten, esp_foc_cali_chars_second_step_t *ctx)
{
    uint32_t adc_calib_ver = esp_efuse_rtc_calib_get_ver();
    assert((adc_calib_ver >= ESP_EFUSE_ADC_CALIB_VER_MIN) &&
           (adc_calib_ver <= ESP_EFUSE_ADC_CALIB_VER_MAX));

    ctx->term_num = 2;

    ctx->coeff = unit == ADC_UNIT_1 ?
                 adc1_error_coef_atten[VER2IDX(adc_calib_ver)][atten] :
                 adc2_error_coef_atten[VER2IDX(adc_calib_ver)][atten];
    ctx->sign = unit == ADC_UNIT_1 ?
                adc1_error_sign[VER2IDX(adc_calib_ver)][atten] :
                adc2_error_sign[VER2IDX(adc_calib_ver)][atten];
}

#else

void esp_foc_curve_fitting_get_second_step_coeff(adc_unit_t unit, adc_atten_t atten,
                                                 esp_foc_cali_chars_second_step_t *ctx)
{
    (void)unit; (void)atten;
    ctx->term_num = 0; ctx->coeff = NULL; ctx->sign = NULL;
}

#endif
