/*
 * MIT License
 */
#pragma once

#include <stdint.h>
#include "espFoC/utils/esp_foc_q16.h"

typedef struct esp_foc_rotor_sensor_s esp_foc_rotor_sensor_t;

struct esp_foc_rotor_sensor_s {
    void (*set_to_zero)(esp_foc_rotor_sensor_t *self);
    uint32_t (*get_counts_per_revolution)(esp_foc_rotor_sensor_t *self);
    q16_t (*read_counts)(esp_foc_rotor_sensor_t *self);
    int64_t (*read_accumulated_counts_i64)(esp_foc_rotor_sensor_t *self);
    void (*set_simulation_count)(esp_foc_rotor_sensor_t *self, q16_t increment);
};
