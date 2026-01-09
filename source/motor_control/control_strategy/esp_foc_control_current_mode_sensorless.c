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
#include "esp_log.h"
#include "espFoC/esp_foc.h"
#include "espFoC/observer/esp_foc_simu_observer.h"
#include "espFoC/observer/esp_foc_pll_observer.h"

static const char * tag = "ESP_FOC_CONTROL";

static void inverter_isr(void *data)
{
    esp_foc_axis_t *axis = (esp_foc_axis_t *)data;

    if(axis->downsampling_low_speed) {
        axis->downsampling_low_speed--;
        if(!axis->downsampling_low_speed) {
            axis->downsampling_low_speed = ESP_FOC_LOW_SPEED_DOWNSAMPLING;
            esp_foc_send_notification(axis->low_speed_ev);
        }
    }
}

void do_current_mode_sensorless_high_speed_loop(void *arg)
{
    /* Samples already buffered nothing to do here */
}

void do_current_mode_sensorless_low_speed_loop(void *arg)
{
    esp_foc_axis_t *axis = (esp_foc_axis_t *)arg;
    isensor_values_t ival;

#ifdef CONFIG_ESP_FOC_SCOPE
    esp_foc_control_data_t control_data;
#endif

    axis->low_speed_ev = esp_foc_get_event_handle();
    axis->downsampling_low_speed = ESP_FOC_LOW_SPEED_DOWNSAMPLING;

    ESP_LOGI(tag,"Starting the current mode sensored low speed loop");

    axis->rotor_position = axis->observer->get_angle(axis->observer);
    axis->current_speed = axis->observer->get_speed(axis->observer);

    ESP_LOGI(tag,"Starting current mode sensored high speed loop");

    axis->inverter_driver->set_inverter_callback(axis->inverter_driver,
        inverter_isr,
        axis);

    axis->isensor_driver->set_isensor_callback(axis->isensor_driver,
        axis->high_speed_loop_cb,
        axis);

    while(1) {
        esp_foc_wait_notifier();

#ifdef CONFIG_ESP_FOC_DEBUG_CORE_TIMING
        esp_foc_debug_pin_set();
#endif

#ifndef CONFIG_ESP_FOC_OPEN_LOOP_ONLY
        axis->isensor_driver->fetch_isensors(axis->isensor_driver, &ival);
        axis->i_u = ival.iu_axis_0;
        axis->i_v = ival.iv_axis_0;
        axis->i_w = ival.iw_axis_0;

        /* After picking the previous sample start the new conversion immediately
         * (it will be used on the next loop step )
         */
        axis->isensor_driver->sample_isensors(axis->isensor_driver);
#endif

        axis->rotor_position = axis->observer->get_angle(axis->observer);
        axis->rotor_elec_angle = axis->rotor_position;
        axis->current_speed = axis->observer->get_speed(axis->observer);
        esp_foc_position_control_loop(axis);
        esp_foc_velocity_control_loop(axis);

        /* Current control stuff */
        float e_sin = esp_foc_sine(axis->rotor_elec_angle);
        float e_cos = esp_foc_cosine(axis->rotor_elec_angle);

#ifdef CONFIG_ESP_FOC_OPEN_LOOP_ONLY
        axis->u_d.raw = axis->target_u_d.raw;
        axis->u_q.raw = axis->target_u_q.raw;

#else

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
#endif

        esp_foc_modulate_dq_voltage(e_sin,
                        e_cos,
                        axis->u_d.raw,
                        axis->u_q.raw,
                        &axis->u_alpha.raw,
                        &axis->u_beta.raw,
                        &axis->u_u.raw,
                        &axis->u_v.raw,
                        &axis->u_w.raw,
                        axis->max_voltage,
                        axis->biased_dc_link_voltage,
                        axis->dc_link_to_normalized);

        axis->inverter_driver->set_voltages(axis->inverter_driver,
                                            axis->u_u.raw,
                                            axis->u_v.raw,
                                            axis->u_w.raw);

        /* Observer update */
        esp_foc_observer_inputs_t in = {
            .i_dq[0] = axis->i_d.raw,
            .i_dq[1] = axis->i_q.raw,
            .u_dq[0] = axis->u_d.raw,
            .u_dq[1] = axis->u_q.raw,
            .u_alpha_beta[0] = axis->u_alpha.raw,
            .u_alpha_beta[1] = axis->u_beta.raw,
            .i_alpha_beta[0] = axis->i_alpha.raw,
            .i_alpha_beta[1] = axis->i_beta.raw,
        };

        axis->open_loop_observer->update(axis->open_loop_observer, &in);

#ifndef CONFIG_ESP_FOC_OPEN_LOOP_ONLY
        if(axis->isensor_driver != NULL) {
            int not_conv = axis->current_observer->update(axis->current_observer, &in);
            if(!not_conv) {
                axis->observer = axis->current_observer;
            }
        }
#endif

#ifdef CONFIG_ESP_FOC_SCOPE
        esp_foc_get_control_data(axis, &control_data);
        esp_foc_scope_data_push(&control_data);
#endif

#ifdef CONFIG_ESP_FOC_DEBUG_CORE_TIMING
        esp_foc_debug_pin_clear();
#endif
    }
}