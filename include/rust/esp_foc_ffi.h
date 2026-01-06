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

#include <stdint.h>
#include <stdbool.h>

#include "espFoC/esp_foc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque handles for Rust */
typedef void* espfoc_axis_handle_t;
typedef void* espfoc_inverter_handle_t;
typedef void* espfoc_rotor_handle_t;
typedef void* espfoc_isensor_handle_t;

/* Minimal ADC current sensor config for FFI.
 * We use int32_t for adc_channel_t and adc_unit_t to keep this header independent
 * from ESP-IDF driver headers on the Rust side.
 */
typedef struct {
    int32_t axis_channels[4];
    int32_t units[4];
    float amp_gain;
    float shunt_resistance;
    int32_t number_of_channels;
} espfoc_isensor_adc_config_t;

/* Axis allocation (so Rust does not depend on esp_foc_axis_t layout) */
espfoc_axis_handle_t espfoc_axis_alloc(void);
void espfoc_axis_free(espfoc_axis_handle_t axis);

/* Driver constructors */
espfoc_inverter_handle_t espfoc_inverter_3pwm_mcpwm_new(int gpio_u, int gpio_v, int gpio_w,
                                                       int gpio_enable,
                                                       float dc_link_voltage,
                                                       int port);

espfoc_inverter_handle_t espfoc_inverter_6pwm_mcpwm_new(int gpio_u_high, int gpio_u_low,
                                                       int gpio_v_high, int gpio_v_low,
                                                       int gpio_w_high, int gpio_w_low,
                                                       int gpio_enable,
                                                       float dc_link_voltage,
                                                       int port);

/* Rotor sensors (optional; open-loop can pass NULL) */
espfoc_rotor_handle_t espfoc_rotor_sensor_as5600_new(int i2c_port, uint8_t i2c_address);

/* Current sensor (optional) */
espfoc_isensor_handle_t espfoc_isensor_adc_new(const espfoc_isensor_adc_config_t *config);

/* Axis lifecycle */
esp_foc_err_t espfoc_initialize_axis(espfoc_axis_handle_t axis,
                                    espfoc_inverter_handle_t inverter,
                                    espfoc_rotor_handle_t rotor_sensor,   /* may be NULL for open-loop */
                                    espfoc_isensor_handle_t isensor,       /* may be NULL */
                                    const esp_foc_motor_control_settings_t *settings);

esp_foc_err_t espfoc_align_axis(espfoc_axis_handle_t axis);
esp_foc_err_t espfoc_run_axis(espfoc_axis_handle_t axis);

/* Setpoints */
esp_foc_err_t espfoc_set_target_voltage(espfoc_axis_handle_t axis, float uq, float ud);
esp_foc_err_t espfoc_set_target_speed(espfoc_axis_handle_t axis, float speed);
esp_foc_err_t espfoc_set_target_position(espfoc_axis_handle_t axis, float position);
esp_foc_err_t espfoc_set_target_current(espfoc_axis_handle_t axis, float iq, float id);

/* Telemetry */
esp_foc_err_t espfoc_get_control_data(espfoc_axis_handle_t axis, esp_foc_control_data_t *out);

#ifdef __cplusplus
}
#endif
