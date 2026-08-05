/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * Line-fitting ADC calibration for ESP32 (init-time LUT, LUT high-range enabled).
 */

#include <stdint.h>
#include <stdbool.h>
#include "sdkconfig.h"
#include "hal/adc_types.h"
#include "hal/efuse_ll.h"
#include "esp_foc_adc_cali.h"

#if CONFIG_IDF_TARGET_ESP32 && SOC_ADC_CALIB_SCHEME_LINE_FITTING_SUPPORTED

#define LIN_COEFF_A_SCALE       65536
#define LIN_COEFF_A_ROUND       (LIN_COEFF_A_SCALE / 2)
#define LUT_VREF_LOW            1000
#define LUT_VREF_HIGH           1200
#define LUT_ADC_STEP_SIZE       64
#define LUT_POINTS              20
#define LUT_LOW_THRESH          2880
#define LUT_HIGH_THRESH         (LUT_LOW_THRESH + LUT_ADC_STEP_SIZE)
#define ADC_12_BIT_RES          4096

#define VREF_MASK               0x1F
#define VREF_STEP_SIZE          7
#define VREF_OFFSET             1100
#define TP_LOW1_OFFSET          278
#define TP_LOW2_OFFSET          421
#define TP_LOW_MASK             0x7F
#define TP_LOW_VOLTAGE          150
#define TP_HIGH1_OFFSET         3265
#define TP_HIGH2_OFFSET         3406
#define TP_HIGH_MASK            0x1FF
#define TP_HIGH_VOLTAGE         850
#define TP_STEP_SIZE            4
#define CHECK_BLK3_FLAG         1
#define VREF_FORMAT             0

static const uint32_t adc1_tp_atten_scale[4] = {65504, 86975, 120389, 224310};
static const uint32_t adc2_tp_atten_scale[4] = {65467, 86861, 120416, 224708};
static const uint32_t adc1_tp_atten_offset[4] = {0, 1, 27, 54};
static const uint32_t adc2_tp_atten_offset[4] = {0, 9, 26, 66};
static const uint32_t adc1_vref_atten_scale[4] = {57431, 76236, 105481, 196602};
static const uint32_t adc2_vref_atten_scale[4] = {57236, 76175, 105678, 197170};
static const uint32_t adc1_vref_atten_offset[4] = {75, 78, 107, 142};
static const uint32_t adc2_vref_atten_offset[4] = {63, 66, 89, 128};

static const uint32_t lut_adc1_low[LUT_POINTS] = {
    2240, 2297, 2352, 2405, 2457, 2512, 2564, 2616, 2664, 2709,
    2754, 2795, 2832, 2868, 2903, 2937, 2969, 3000, 3030, 3060
};
static const uint32_t lut_adc1_high[LUT_POINTS] = {
    2667, 2706, 2745, 2780, 2813, 2844, 2873, 2901, 2928, 2956,
    2982, 3006, 3032, 3059, 3084, 3110, 3135, 3160, 3184, 3209
};
static const uint32_t lut_adc2_low[LUT_POINTS] = {
    2238, 2293, 2347, 2399, 2451, 2507, 2561, 2613, 2662, 2710,
    2754, 2792, 2831, 2869, 2904, 2937, 2968, 2999, 3029, 3059
};
static const uint32_t lut_adc2_high[LUT_POINTS] = {
    2657, 2698, 2738, 2774, 2807, 2838, 2867, 2894, 2921, 2946,
    2971, 2996, 3020, 3043, 3067, 3092, 3116, 3139, 3162, 3185
};

typedef struct {
    adc_unit_t unit;
    adc_atten_t atten;
    bool ready;
    uint32_t coeff_a;
    uint32_t coeff_b;
    uint32_t vref;
    const uint32_t *low_curve;
    const uint32_t *high_curve;
} esp_foc_line_ctx_t;

static esp_foc_line_ctx_t s_line;

static inline int decode_bits(uint32_t bits, uint32_t mask, bool is_twos_compl)
{
    if (bits & (~(mask >> 1) & mask)) {
        if (is_twos_compl) {
            return -(((~bits) + 1) & (mask >> 1));
        }
        return -(bits & (mask >> 1));
    }
    return (int)(bits & (mask >> 1));
}

static bool check_efuse_tp(void)
{
    if (CHECK_BLK3_FLAG && (efuse_ll_get_blk3_part_reserve() == 0)) {
        return false;
    }
    return efuse_ll_get_adc1_tp_low() && efuse_ll_get_adc2_tp_low()
           && efuse_ll_get_adc1_tp_high() && efuse_ll_get_adc2_tp_high();
}

static void characterize_using_two_point(adc_unit_t unit, adc_atten_t atten,
                                       uint32_t high, uint32_t low,
                                       uint32_t *coeff_a, uint32_t *coeff_b)
{
    const uint32_t *scales = (unit == ADC_UNIT_1) ? adc1_tp_atten_scale : adc2_tp_atten_scale;
    const uint32_t *offsets = (unit == ADC_UNIT_1) ? adc1_tp_atten_offset : adc2_tp_atten_offset;
    uint32_t delta_x = high - low;
    if (delta_x == 0) {
        return;
    }
    uint32_t delta_v = TP_HIGH_VOLTAGE - TP_LOW_VOLTAGE;
    *coeff_a = (delta_v * scales[atten] + (delta_x / 2)) / delta_x;
    *coeff_b = TP_HIGH_VOLTAGE - ((delta_v * high + (delta_x / 2)) / delta_x) + offsets[atten];
}

static void characterize_using_vref(adc_unit_t unit, adc_atten_t atten,
                                    uint32_t vref, uint32_t *coeff_a, uint32_t *coeff_b)
{
    const uint32_t *scales = (unit == ADC_UNIT_1) ? adc1_vref_atten_scale : adc2_vref_atten_scale;
    const uint32_t *offsets = (unit == ADC_UNIT_1) ? adc1_vref_atten_offset : adc2_vref_atten_offset;
    *coeff_a = (vref * scales[atten]) / ADC_12_BIT_RES;
    *coeff_b = offsets[atten];
}

static uint32_t calc_voltage_linear(uint32_t raw, uint32_t coeff_a, uint32_t coeff_b)
{
    return (((coeff_a * raw) + LIN_COEFF_A_ROUND) / LIN_COEFF_A_SCALE) + coeff_b;
}

static inline uint32_t interpolate_two_points(uint32_t y1, uint32_t y2, uint32_t x_step, uint32_t x)
{
    return ((y1 * x_step) + (y2 * x) - (y1 * x) + (x_step / 2)) / x_step;
}

static uint32_t calc_voltage_lut(uint32_t adc, uint32_t vref,
                                 const uint32_t *low_curve, const uint32_t *high_curve)
{
    uint32_t i = (adc - LUT_LOW_THRESH) / LUT_ADC_STEP_SIZE;
    if (i + 1 >= LUT_POINTS) {
        i = LUT_POINTS - 2;
    }
    int x2dist = LUT_VREF_HIGH - (int)vref;
    int x1dist = (int)vref - LUT_VREF_LOW;
    int y2dist = (int)(((i + 1) * LUT_ADC_STEP_SIZE) + LUT_LOW_THRESH - adc);
    int y1dist = (int)(adc - ((i * LUT_ADC_STEP_SIZE) + LUT_LOW_THRESH));
    int q11 = (int)low_curve[i];
    int q12 = (int)low_curve[i + 1];
    int q21 = (int)high_curve[i];
    int q22 = (int)high_curve[i + 1];
    int voltage = (q11 * x2dist * y2dist) + (q21 * x1dist * y2dist)
                  + (q12 * x2dist * y1dist) + (q22 * x1dist * y1dist);
    voltage += ((LUT_VREF_HIGH - LUT_VREF_LOW) * LUT_ADC_STEP_SIZE) / 2;
    voltage /= ((LUT_VREF_HIGH - LUT_VREF_LOW) * LUT_ADC_STEP_SIZE);
    return (uint32_t)voltage;
}

static bool line_ctx_prepare(adc_unit_t unit, adc_atten_t atten)
{
    if (s_line.ready && s_line.unit == unit && s_line.atten == atten) {
        return true;
    }
    if (efuse_ll_get_adc_calib_ver() == 0) {
        return false;
    }

    uint32_t coeff_a = 0;
    uint32_t coeff_b = 0;
    uint32_t vref = 1100;

    if (check_efuse_tp()) {
        uint32_t high = (unit == ADC_UNIT_1) ?
                        (TP_HIGH1_OFFSET + decode_bits(efuse_ll_get_adc1_tp_high(), TP_HIGH_MASK, true) * TP_STEP_SIZE) :
                        (TP_HIGH2_OFFSET + decode_bits(efuse_ll_get_adc2_tp_high(), TP_HIGH_MASK, true) * TP_STEP_SIZE);
        uint32_t low = (unit == ADC_UNIT_1) ?
                       (TP_LOW1_OFFSET + decode_bits(efuse_ll_get_adc1_tp_low(), TP_LOW_MASK, true) * TP_STEP_SIZE) :
                       (TP_LOW2_OFFSET + decode_bits(efuse_ll_get_adc2_tp_low(), TP_LOW_MASK, true) * TP_STEP_SIZE);
        characterize_using_two_point(unit, atten, high, low, &coeff_a, &coeff_b);
        if (coeff_a == 0) {
            return false;
        }
        vref = VREF_OFFSET + decode_bits(efuse_ll_get_adc_vref(), VREF_MASK, VREF_FORMAT) * VREF_STEP_SIZE;
    } else if (efuse_ll_get_adc_vref() != 0) {
        vref = VREF_OFFSET + decode_bits(efuse_ll_get_adc_vref(), VREF_MASK, VREF_FORMAT) * VREF_STEP_SIZE;
        characterize_using_vref(unit, atten, vref, &coeff_a, &coeff_b);
    } else {
        characterize_using_vref(unit, atten, vref, &coeff_a, &coeff_b);
    }

    s_line.unit = unit;
    s_line.atten = atten;
    s_line.coeff_a = coeff_a;
    s_line.coeff_b = coeff_b;
    s_line.vref = vref;
    if (atten == ADC_ATTEN_DB_11 || atten == ADC_ATTEN_DB_12) {
        s_line.low_curve = (unit == ADC_UNIT_1) ? lut_adc1_low : lut_adc2_low;
        s_line.high_curve = (unit == ADC_UNIT_1) ? lut_adc1_high : lut_adc2_high;
    } else {
        s_line.low_curve = NULL;
        s_line.high_curve = NULL;
    }
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

    if (raw < 0) {
        raw = 0;
    }
    if (raw > ADC_12_BIT_RES - 1) {
        raw = ADC_12_BIT_RES - 1;
    }

    uint32_t voltage;
    if (s_line.low_curve != NULL && raw >= (int)LUT_LOW_THRESH) {
        uint32_t lut_v = calc_voltage_lut((uint32_t)raw, s_line.vref, s_line.low_curve, s_line.high_curve);
        if (raw <= (int)LUT_HIGH_THRESH) {
            uint32_t lin_v = calc_voltage_linear((uint32_t)raw, s_line.coeff_a, s_line.coeff_b);
            voltage = interpolate_two_points(lin_v, lut_v, LUT_ADC_STEP_SIZE, (uint32_t)(raw - LUT_LOW_THRESH));
        } else {
            voltage = lut_v;
        }
    } else {
        voltage = calc_voltage_linear((uint32_t)raw, s_line.coeff_a, s_line.coeff_b);
    }

    *mv_out = (int)voltage;
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
