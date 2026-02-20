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


#include "rust/esp_foc_ffi.h"

#ifdef CONFIG_ESP_FOC_RUST

#include <stdlib.h>
#include <string.h>

#include "espFoC/esp_foc_axis.h"

/* Drivers (internal headers are already in include path via CMakeLists) */
#include "espFoC/inverter_3pwm_mcpwm.h"
#include "espFoC/inverter_6pwm_mcpwm.h"
#include "espFoC/rotor_sensor_as5600.h"
#include "espFoC/current_sensor_adc.h"

espfoc_axis_handle_t espfoc_axis_alloc(void)
{
    esp_foc_axis_t *axis = (esp_foc_axis_t *)calloc(1, sizeof(esp_foc_axis_t));
    return (espfoc_axis_handle_t)axis;
}

void espfoc_axis_free(espfoc_axis_handle_t axis)
{
    if (axis) {
        free((void *)axis);
    }
}

espfoc_inverter_handle_t espfoc_inverter_3pwm_mcpwm_new(int gpio_u, int gpio_v, int gpio_w,
                                                       int gpio_enable,
                                                       float dc_link_voltage,
                                                       int port)
{
    return (espfoc_inverter_handle_t)inverter_3pwm_mpcwm_new(gpio_u, gpio_v, gpio_w, gpio_enable, dc_link_voltage, port);
}

espfoc_inverter_handle_t espfoc_inverter_6pwm_mcpwm_new(int gpio_u_high, int gpio_u_low,
                                                       int gpio_v_high, int gpio_v_low,
                                                       int gpio_w_high, int gpio_w_low,
                                                       int gpio_enable,
                                                       float dc_link_voltage,
                                                       int port)
{
    return (espfoc_inverter_handle_t)inverter_6pwm_mpcwm_new(gpio_u_high, gpio_u_low,
                                                            gpio_v_high, gpio_v_low,
                                                            gpio_w_high, gpio_w_low,
                                                            gpio_enable,
                                                            dc_link_voltage, port);
}

espfoc_rotor_handle_t espfoc_rotor_sensor_as5600_new(int pin_sda, int pin_scl,
                                                    int port);
{
    return (espfoc_rotor_handle_t)rotor_sensor_as5600_new(pin_sda, pin_scl, port);
}

espfoc_isensor_handle_t espfoc_isensor_adc_new(const espfoc_isensor_adc_config_t *config)
{
    if (!config) {
        return NULL;
    }

    esp_foc_isensor_adc_config_t cfg = {0};
    cfg.amp_gain = config->amp_gain;
    cfg.shunt_resistance = config->shunt_resistance;
    cfg.number_of_channels = config->number_of_channels;

    for (int i = 0; i < 4; i++) {
        cfg.axis_channels[i] = (adc_channel_t)config->axis_channels[i];
        cfg.units[i] = (adc_unit_t)config->units[i];
    }

    return (espfoc_isensor_handle_t)isensor_adc_new(&cfg);
}

static inline esp_foc_axis_t *axis_from_handle(espfoc_axis_handle_t h)
{
    return (esp_foc_axis_t *)h;
}

esp_foc_err_t espfoc_initialize_axis(espfoc_axis_handle_t axis,
                                    espfoc_inverter_handle_t inverter,
                                    espfoc_rotor_handle_t rotor_sensor,
                                    espfoc_isensor_handle_t isensor,
                                    const esp_foc_motor_control_settings_t *settings)
{
    if (!axis || !inverter || !settings) {
        return ESP_FOC_ERR_INVALID_ARG;
    }

    return esp_foc_initialize_axis(axis_from_handle(axis),
                                  (esp_foc_inverter_t *)inverter,
                                  (esp_foc_rotor_sensor_t *)rotor_sensor,
                                  (esp_foc_isensor_t *)isensor,
                                  *settings);
}

esp_foc_err_t espfoc_align_axis(espfoc_axis_handle_t axis)
{
    if (!axis) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    return esp_foc_align_axis(axis_from_handle(axis));
}

esp_foc_err_t espfoc_run_axis(espfoc_axis_handle_t axis)
{
    if (!axis) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    return esp_foc_run(axis_from_handle(axis));
}

esp_foc_err_t espfoc_set_target_voltage(espfoc_axis_handle_t axis, float uq, float ud)
{
    if (!axis) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    return esp_foc_set_target_voltage(axis_from_handle(axis),
                                      (esp_foc_voltage){ .raw = uq },
                                      (esp_foc_voltage){ .raw = ud });
}

esp_foc_err_t espfoc_set_target_speed(espfoc_axis_handle_t axis, float speed)
{
    if (!axis) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    return esp_foc_set_target_speed(axis_from_handle(axis),
                                    (esp_foc_angular_velocity){ .raw = speed });
}

esp_foc_err_t espfoc_set_target_position(espfoc_axis_handle_t axis, float position)
{
    if (!axis) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    return esp_foc_set_target_position(axis_from_handle(axis),
                                       (esp_foc_radians){ .raw = position });
}

esp_foc_err_t espfoc_set_target_current(espfoc_axis_handle_t axis, float iq, float id)
{
    if (!axis) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    return esp_foc_set_target_current(axis_from_handle(axis),
                                      (esp_foc_current){ .raw = iq },
                                      (esp_foc_current){ .raw = id });
}

esp_foc_err_t espfoc_get_control_data(espfoc_axis_handle_t axis, esp_foc_control_data_t *out)
{
    if (!axis || !out) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    return esp_foc_get_control_data(axis_from_handle(axis), out);
}
#endif