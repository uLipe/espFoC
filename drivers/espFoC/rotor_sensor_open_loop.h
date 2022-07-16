#pragma once 

#include "espFoC/esp_foc.h"

esp_foc_rotor_sensor_t *rotor_sensor_open_loop_new(float motor_kv, float *uq_wire, esp_foc_seconds sample_rate);