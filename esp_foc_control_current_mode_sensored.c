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

#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "esp_attr.h"
#include "esp_log.h"
#ifdef CONFIG_ESP_FOC_DEBUG_CORE_TIMING
#include "driver/gpio.h"
#endif
#include "espFoC/esp_foc.h"
#include "espFoC/esp_foc_simu_observer.h"
#include "espFoC/esp_foc_pll_observer.h"

#define ESP_FOC_DEBUG_PIN                  22
static const char * tag = "ESP_FOC_CONTROL";


IRAM_ATTR void do_current_mode_sensored_high_speed_loop(void *arg)
{
    esp_foc_axis_t *axis = (esp_foc_axis_t *)arg;

#ifdef CONFIG_ESP_FOC_DEBUG_CORE_TIMING
    gpio_set_level(ESP_FOC_DEBUG_PIN, true);
#endif

    isensor_values_t val;
    axis->isensor_driver->fetch_isensors(axis->isensor_driver, &val);
    axis->i_u = val.iu_axis_0;
    axis->i_v = val.iv_axis_0;
    axis->i_w = val.iw_axis_0;

    float ext_delta = (axis->dt * (ESP_FOC_LOW_SPEED_DOWNSAMPLING - axis->downsampling_low_speed));
    axis->extrapolated_rotor_position = axis->rotor_position + axis->current_speed * ext_delta;
    axis->rotor_elec_angle = esp_foc_mechanical_to_elec_angle(axis->extrapolated_rotor_position, axis->motor_pole_pairs);
    axis->rotor_elec_angle = esp_foc_normalize_angle(axis->rotor_elec_angle);
    float e_sin = esp_foc_sine(axis->rotor_elec_angle);
    float e_cos = esp_foc_cosine(axis->rotor_elec_angle);

    esp_foc_get_dq_currents(e_sin,
        e_cos,
        axis->i_u,
        axis->i_v,
        axis->i_w,
        &axis->i_alpha.raw,
        &axis->i_beta.raw,
        &axis->i_q.raw,
        &axis->i_d.raw);

    esp_foc_current_control_loop(axis);

    esp_foc_modulate_dq_voltage(e_sin,
                    e_cos,
                    axis->u_d.raw,
                    axis->u_q.raw,
                    &axis->u_alpha.raw,
                    &axis->u_beta.raw,
                    &axis->u_u.raw,
                    &axis->u_v.raw,
                    &axis->u_w.raw,
                    axis->biased_dc_link_voltage,
                    axis->dc_link_to_normalized);


    axis->inverter_driver->set_voltages(axis->inverter_driver,
                                        axis->u_u.raw,
                                        axis->u_v.raw,
                                        axis->u_w.raw);

    if(axis->downsampling_low_speed) {
        axis->downsampling_low_speed--;
        if(!axis->downsampling_low_speed) {
            axis->downsampling_low_speed = ESP_FOC_LOW_SPEED_DOWNSAMPLING;
            esp_foc_send_notification(axis->low_speed_ev);
        }
    }

#ifdef CONFIG_ESP_FOC_DEBUG_CORE_TIMING
    gpio_set_level(ESP_FOC_DEBUG_PIN, false);
#endif

    axis->isensor_driver->sample_isensors(axis->isensor_driver);
}

IRAM_ATTR void do_current_mode_sensored_low_speed_loop(void *arg)
{
    esp_foc_axis_t *axis = (esp_foc_axis_t *)arg;
    float low_speed_inv_dt = (axis->inv_dt / 10.0f);
    float raw_speed;

#ifdef CONFIG_ESP_FOC_SCOPE
    esp_foc_control_data_t control_data;
#endif

    axis->low_speed_ev = esp_foc_get_event_handle();
    axis->downsampling_low_speed = ESP_FOC_LOW_SPEED_DOWNSAMPLING;

    ESP_LOGI(tag,"Starting the current mode sensored low speed loop");

    axis->rotor_shaft_ticks = axis->rotor_sensor_driver->read_counts(axis->rotor_sensor_driver);
    axis->rotor_position =
        axis->rotor_shaft_ticks * axis->shaft_ticks_to_radians_ratio * axis->natural_direction;
    axis->rotor_position_prev = axis->rotor_position;
    axis->current_speed = 0.0f;

    ESP_LOGI(tag,"Starting current mode sensored high speed loop");

    axis->inverter_driver->set_inverter_callback(axis->inverter_driver,
        axis->high_speed_loop_cb,
        axis);

    while(1) {
        esp_foc_wait_notifier();

        axis->rotor_position = axis->rotor_sensor_driver->read_counts(axis->rotor_sensor_driver) *
                            axis->shaft_ticks_to_radians_ratio * axis->natural_direction;

        raw_speed = axis->rotor_position - axis->rotor_position_prev;
        if (raw_speed >  M_PI)  raw_speed -= 2*M_PI;
        if (raw_speed < -M_PI)  raw_speed += 2*M_PI;
        raw_speed *= low_speed_inv_dt;

        esp_foc_critical_enter();
        axis->current_speed = esp_foc_low_pass_filter_update( &axis->velocity_filter, raw_speed);
        axis->rotor_position_prev = axis->rotor_position;
        esp_foc_critical_leave();

        esp_foc_position_control_loop(axis);
        esp_foc_velocity_control_loop(axis);

#ifdef CONFIG_ESP_FOC_SCOPE
        esp_foc_get_control_data(axis, &control_data);
        esp_foc_scope_data_push(&control_data);
#endif
    }
}
