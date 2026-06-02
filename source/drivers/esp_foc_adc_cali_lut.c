/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include <stdint.h>
#include "esp_log.h"
#include "sdkconfig.h"
#include "espFoC/esp_foc_adc_cali_lut.h"
#include "cali/esp_foc_adc_cali.h"
#include "cali/esp_foc_adc_range_extend.h"

static const char *TAG = "esp_foc_adc_cali_lut";

#define ESP_FOC_ADC_REF_MV  3300

static bool raw_to_mv(adc_unit_t unit,
                      adc_channel_t channel,
                      adc_atten_t atten,
                      int raw,
                      int *mv_out)
{
    bool ok = false;
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32C2
    ok = esp_foc_adc_cali_line_raw_to_mv(unit, channel, atten, raw, mv_out);
#else
    ok = esp_foc_adc_cali_curve_raw_to_mv(unit, channel, atten, raw, mv_out);
#endif
    if (ok && mv_out != NULL && esp_foc_adc_range_extend_supported()) {
        *mv_out = esp_foc_adc_range_extend_mv(atten, *mv_out);
    }
    return ok;
}

bool esp_foc_adc_cali_lut_build(adc_unit_t unit,
                                adc_channel_t channel,
                                adc_atten_t atten,
                                int16_t *lut_out,
                                unsigned lut_len)
{
    if (lut_out == NULL || lut_len < ESP_FOC_ISENSOR_ADC_LUT_SIZE) {
        return false;
    }

    bool any_cali = false;
    for (unsigned raw = 0; raw < ESP_FOC_ISENSOR_ADC_LUT_SIZE; raw++) {
        int mv = 0;
        if (raw_to_mv(unit, channel, atten, (int)raw, &mv)) {
            int linear = (mv * (int)ESP_FOC_ISENSOR_ADC_LUT_SIZE) / ESP_FOC_ADC_REF_MV;
            if (linear < 0) {
                linear = 0;
            }
            if (linear >= (int)ESP_FOC_ISENSOR_ADC_LUT_SIZE) {
                linear = (int)ESP_FOC_ISENSOR_ADC_LUT_SIZE - 1;
            }
            lut_out[raw] = (int16_t)linear;
            any_cali = true;
        } else {
            lut_out[raw] = (int16_t)raw;
        }
    }

    if (!any_cali) {
        ESP_LOGW(TAG, "ADC cali LUT identity (unit=%d ch=%d atten=%d)", (int)unit, (int)channel, (int)atten);
    }
    return any_cali;
}
