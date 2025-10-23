#pragma once 

#include "espFoC/esp_foc.h"

esp_foc_rotor_sensor_t *rotor_sensor_pcnt_new(int pin_a,
                                            int pin_b,
                                            int port,
                                            int16_t pulses_per_revolution);