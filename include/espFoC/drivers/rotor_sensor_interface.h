/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
 
#pragma once

#include <sdkconfig.h>

#ifdef CONFIG_ESP_FOC_USE_FIXED_POINT
#include "espFoC/utils/esp_foc_iq31.h"
#include <stdint.h>
#endif

typedef struct esp_foc_rotor_sensor_s esp_foc_rotor_sensor_t;

struct esp_foc_rotor_sensor_s{
    void  (*set_to_zero) (esp_foc_rotor_sensor_t *self);
    float (*get_counts_per_revolution) (esp_foc_rotor_sensor_t *self);
    float (*read_counts) (esp_foc_rotor_sensor_t *self);
    float (*read_accumulated_counts) (esp_foc_rotor_sensor_t *self);
    void  (*set_simulation_count)(esp_foc_rotor_sensor_t *self, float increment);
#ifdef CONFIG_ESP_FOC_USE_FIXED_POINT
    /**
     * Position within one revolution, normalized [0, 1) in Q1.31
     * (same as read_counts / get_counts_per_revolution in float).
     */
    iq31_t (*read_counts_iq31) (esp_foc_rotor_sensor_t *self);
    /**
     * Accumulated count in integer encoder units (ticks from internal reference).
     * Reduces precision loss over multi-turn; semantics documented per driver.
     */
    int64_t (*read_accumulated_counts_i64) (esp_foc_rotor_sensor_t *self);
    /** Simulation increment in normalized counts [0,1) per call, Q1.31. */
    void  (*set_simulation_count_iq31)(esp_foc_rotor_sensor_t *self, iq31_t increment_normalized);
#endif
};