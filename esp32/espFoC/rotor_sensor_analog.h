#pragma once 

#include "espFoC/esp_foc.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_err.h"

esp_err_t rotor_sensor_analog_init();
esp_foc_rotor_sensor_t *rotor_sensor_analog_new(int adc_channel, 
                                                int min_sensor_count,
                                                int max_sensor_count);