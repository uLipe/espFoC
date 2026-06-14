#pragma once 

#include "espFoC/drivers/esp_foc_encoder.h"

esp_foc_encoder_t *esp_foc_encoder_pcnt_new(int pin_a,
                                            int pin_b,
                                            int port,
                                            int16_t pulses_per_revolution);