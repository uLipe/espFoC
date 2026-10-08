/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "espFoC/drivers/esp_foc_rotor_sensor.h"
#include "espFoC/motor_control/esp_foc_observer.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    unsigned pole_pairs;
    esp_foc_observer_t *observer; /* non-owning; caller owns lifetime */
} esp_foc_rotor_sensorless_config_t;

esp_foc_rotor_sensor_t *esp_foc_rotor_sensorless_acquire(unsigned index);
void esp_foc_rotor_sensorless_release(esp_foc_rotor_sensor_t *s);

esp_err_t esp_foc_rotor_sensorless_init(esp_foc_rotor_sensor_t *s,
                                        const esp_foc_rotor_sensorless_config_t *cfg);
esp_err_t esp_foc_rotor_sensorless_deinit(esp_foc_rotor_sensor_t *s);

/** Latch θm/ωm from the observer (call after observer_update in TEZ or task). */
void esp_foc_rotor_sensorless_latch(esp_foc_rotor_sensor_t *s);

#ifdef __cplusplus
}
#endif
