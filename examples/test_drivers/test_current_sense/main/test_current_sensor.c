/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#include "esp_log.h"
#include "esp_err.h"

#include "espFoC/esp_foc.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/drivers/esp_foc_inverter_mcpwm.h"

static const char *TAG = "esp-foc-example";

static esp_foc_inverter_t *inverter;

static void initialize_foc_drivers(void)
{
    inverter = esp_foc_inverter_mcpwm_6pwm_new(
        4, 5, 6, 7, 8, 9,
        -1,
        24.0f,
        0,
        ADC_CHANNEL_1,
        ADC_CHANNEL_5,
        50.0f,
        0.01f);
    if (inverter == NULL) {
        ESP_LOGE(TAG, "failed to create the shunt sensor driver, aborting!");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
        abort();
    }
}

void app_main(void)
{
    esp_foc_inverter_isensor_values_t values;
    initialize_foc_drivers();

    inverter->calibrate_isensors(inverter, 50);

    while (1) {
        inverter->sample_isensors(inverter);
        esp_foc_sleep_ms(100);
        inverter->fetch_isensors(inverter, &values);
        ESP_LOGI(TAG, "phase currents:  %f, %f, %f",
                 (double)q16_to_float(values.iu_axis_0),
                 (double)q16_to_float(values.iv_axis_0),
                 (double)q16_to_float(values.iw_axis_0));
    }
}
