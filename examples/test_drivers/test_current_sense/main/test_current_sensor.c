/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#include "esp_log.h"
#include "esp_err.h"

#include "espFoC/current_sensor_adc.h"
#include "espFoC/esp_foc.h"
#include "espFoC/utils/esp_foc_q16.h"

static const char *TAG = "esp-foc-example";

static esp_foc_isensor_t  *shunts;

static void initialize_foc_drivers(void)
{
    esp_foc_isensor_adc_config_t shunt_cfg = {
        .axis_channels = {ADC_CHANNEL_1, ADC_CHANNEL_5},
        .units = {ADC_UNIT_1, ADC_UNIT_1},
        .amp_gain = 50.0f,
        .shunt_resistance = 0.01f,
        .number_of_channels = 2,
    };

    shunts = isensor_adc_new(&shunt_cfg);
    if(shunts == NULL) {
        ESP_LOGE(TAG, "failed to create the shunt sensor driver, aborting!");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
        abort();
    }
}

void app_main(void)
{
    isensor_values_t values;
    initialize_foc_drivers();

    shunts->calibrate_isensors(shunts, 50);

    while(1) {
        shunts->sample_isensors(shunts);
        esp_foc_sleep_ms(100);
        shunts->fetch_isensors(shunts, &values);
        ESP_LOGI(TAG, "phase currents:  %f, %f, %f",
            (double)q16_to_float(values.iu_axis_0),
            (double)q16_to_float(values.iv_axis_0),
            (double)q16_to_float(values.iw_axis_0));
    }
}
