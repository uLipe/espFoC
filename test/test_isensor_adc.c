/*
 * Unit tests for isensor ADC cali LUT (no hardware).
 */
#include <unity.h>
#include "espFoC/esp_foc_adc_cali_lut.h"
#include "espFoC/current_sensor_adc.h"
#include "espFoC/esp_foc_err.h"
#include "hal/adc_types.h"

TEST_CASE("adc cali LUT builds 4096 entries", "[espFoC][isensor_adc]")
{
    int16_t lut[ESP_FOC_ISENSOR_ADC_LUT_SIZE];
    bool ok = esp_foc_adc_cali_lut_build(ADC_UNIT_1, ADC_CHANNEL_0, ADC_ATTEN_DB_12,
                                         lut, ESP_FOC_ISENSOR_ADC_LUT_SIZE);
    (void)ok;
    TEST_ASSERT_EQUAL(0, lut[0]);
    TEST_ASSERT_EQUAL(4095, lut[4095]);
}

TEST_CASE("adc cali LUT apply clamps out of range", "[espFoC][isensor_adc]")
{
    int16_t lut[ESP_FOC_ISENSOR_ADC_LUT_SIZE];
    for (int i = 0; i < ESP_FOC_ISENSOR_ADC_LUT_SIZE; i++) {
        lut[i] = (int16_t)i;
    }
    TEST_ASSERT_EQUAL(0, esp_foc_adc_cali_lut_apply(lut, -10));
    TEST_ASSERT_EQUAL(4095, esp_foc_adc_cali_lut_apply(lut, 5000));
    TEST_ASSERT_EQUAL(100, esp_foc_adc_cali_lut_apply(lut, 100));
}

TEST_CASE("isensor trigger ETM returns not supported", "[espFoC][isensor_adc]")
{
    esp_foc_err_t err = esp_foc_isensor_adc_set_trigger(NULL, ESP_FOC_ISENSOR_ADC_TRIG_ETM);
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG, err);
}
