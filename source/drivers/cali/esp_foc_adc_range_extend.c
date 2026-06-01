/*
 * SPDX-FileCopyrightText: Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * ADC range extension for ESP32-S2/S3 (esp-iot-solution patch logic, LUT build path).
 * Dual-read HW path is not available in digi-DMA mode; polynomial extrapolation applies
 * when the first-pass calibrated voltage exceeds the chip threshold.
 */

#include <stdint.h>
#include "sdkconfig.h"
#include "hal/adc_types.h"
#include "esp_foc_adc_cali.h"

#define ESP_FOC_ADC_RANGE_EXTEND_REF_MV  3300

#if CONFIG_IDF_TARGET_ESP32S2

#define ESP_FOC_ADC_RANGE_THRESHOLD_MV  2600

static int s2_range_extend_mv(int mv)
{
    const float a = -0.0000050800531f;
    const float b = 0.02334678273232382f;
    const float c = -26.699083271336267f;
    float v = (float)mv;
    float e = a * v * v + b * v + c;
    v = v * (1.0f + e / 100.0f);
    if (v < 0.0f) {
        v = 0.0f;
    }
    if (v > ESP_FOC_ADC_RANGE_EXTEND_REF_MV) {
        v = (float)ESP_FOC_ADC_RANGE_EXTEND_REF_MV;
    }
    return (int)(v + 0.5f);
}

#elif CONFIG_IDF_TARGET_ESP32S3

#define ESP_FOC_ADC_RANGE_THRESHOLD_MV  2900

static int s3_range_extend_mv(int mv)
{
    int v = mv + 1000;
    if (v > 2700) {
        const float a = -0.0000016625088686597596f;
        const float b = 0.0012152697844402401f;
        const float c = 7.660092154791914f;
        float vf = (float)v;
        float e = a * vf * vf + b * vf + c;
        vf = vf * (1.0f + e / 100.0f);
        v = (int)(vf + 0.5f);
    }
    if (v < 0) {
        v = 0;
    }
    if (v > ESP_FOC_ADC_RANGE_EXTEND_REF_MV) {
        v = ESP_FOC_ADC_RANGE_EXTEND_REF_MV;
    }
    return v;
}

#endif

bool esp_foc_adc_range_extend_supported(void)
{
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
    return true;
#else
    return false;
#endif
}

int esp_foc_adc_range_extend_mv(adc_atten_t atten, int mv)
{
#if CONFIG_IDF_TARGET_ESP32S2
    if (atten == ADC_ATTEN_DB_12 && mv > ESP_FOC_ADC_RANGE_THRESHOLD_MV) {
        return s2_range_extend_mv(mv);
    }
#elif CONFIG_IDF_TARGET_ESP32S3
    if (atten == ADC_ATTEN_DB_12 && mv > ESP_FOC_ADC_RANGE_THRESHOLD_MV) {
        return s3_range_extend_mv(mv);
    }
#endif
    return mv;
}
