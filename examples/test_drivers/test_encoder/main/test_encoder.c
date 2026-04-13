/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#include "esp_log.h"
#include "esp_err.h"

#include "espFoC/rotor_sensor_as5600.h"
#include "espFoC/rotor_sensor_as5048.h"
#include "espFoC/esp_foc.h"
#include "espFoC/utils/esp_foc_q16.h"

static const char *TAG = "esp-foc-example";

static esp_foc_rotor_sensor_t *sensor;
static esp_foc_rotor_sensor_t *sensor2;


static void initialize_foc_drivers(void)
{
    sensor = rotor_sensor_as5600_new(
        CONFIG_FOC_ENCODER_SDA_PIN,
        CONFIG_FOC_ENCODER_SCL_PIN,
        0
    );

    if(sensor == NULL) {
        ESP_LOGE(TAG, "failed to create as5600 encoder driver");
    }

    sensor2 = rotor_sensor_as5048_new(
        CONFIG_FOC_ENCODER_MOSI_PIN,
        CONFIG_FOC_ENCODER_MISO_PIN,
        CONFIG_FOC_ENCODER_SCK_PIN,
        CONFIG_FOC_ENCODER_CS_PIN,
        0,
        0
    );

    if(sensor2 == NULL) {
        ESP_LOGE(TAG, "failed to create as5048 encoder driver");
    }
}

void app_main(void)
{
    initialize_foc_drivers();

    while(1) {
        esp_foc_sleep_ms(100);
        q16_t r1 = (sensor != NULL) ? sensor->read_counts(sensor) : 0;
        q16_t r2 = (sensor2 != NULL) ? sensor2->read_counts(sensor2) : 0;
        ESP_LOGI(TAG, "Turn the motor axis by Hand! Encoder reading raw:  %f", (double)q16_to_float(r1));
        ESP_LOGI(TAG, "Turn the motor axis by Hand! Encoder SPI reading raw:  %f", (double)q16_to_float(r2));
    }
}
