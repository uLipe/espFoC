/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "espFoC/drivers/esp_foc_rotor_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int sda;
    int scl;
    int i2c_port;       /* HP I2C controller index, usually 0 */
    uint32_t i2c_hz;    /* bus frequency, e.g. 400000 */
    float dt_seconds;   /* low-speed sample period; converted to q16 at init */
    bool invert;
    /* 0 → get_electrical_position() stays 0 and ELEC_ABS is off. */
    unsigned pole_pairs;
    /*
     * Rate at which step() is called [Hz]. 0 → step() only ages the sample.
     * Non-zero enables PREDICT: dead-reckon θ_m at this rate from the last
     * I2C-derived ω_m, so the hot path is not a staircase of 2 kHz latches.
     */
    uint32_t pwm_hz;
} esp_foc_rotor_as5600_config_t;

esp_foc_rotor_sensor_t *esp_foc_rotor_as5600_acquire(unsigned index);
void esp_foc_rotor_as5600_release(esp_foc_rotor_sensor_t *s);

esp_err_t esp_foc_rotor_as5600_init(esp_foc_rotor_sensor_t *s,
                                    const esp_foc_rotor_as5600_config_t *cfg);
esp_err_t esp_foc_rotor_as5600_deinit(esp_foc_rotor_sensor_t *s);

/** True while an I2C angle transfer is in flight. */
bool esp_foc_rotor_as5600_busy(const esp_foc_rotor_sensor_t *s);
/** Abort stuck I2C (task context). Disarms the ANGLE pointer. */
void esp_foc_rotor_as5600_bus_recover(esp_foc_rotor_sensor_t *s);
/** Cumulative transfer failures (NACK / timeout / short RX). */
uint32_t esp_foc_rotor_as5600_xfer_fail_count(const esp_foc_rotor_sensor_t *s);
/** Successful completions (blocking fetch or async done with ESP_OK). */
uint32_t esp_foc_rotor_as5600_xfer_ok_count(const esp_foc_rotor_sensor_t *s);
void esp_foc_rotor_as5600_set_invert(esp_foc_rotor_sensor_t *s, bool invert);
void esp_foc_rotor_as5600_set_pole_pairs(esp_foc_rotor_sensor_t *s,
                                         unsigned pole_pairs);

#ifdef __cplusplus
}
#endif
