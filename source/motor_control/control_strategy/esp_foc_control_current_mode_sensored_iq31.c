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

#include <math.h>
#include <stdbool.h>
#include "esp_log.h"
#include "espFoC/esp_foc_iq31_api.h"

#ifdef CONFIG_ESP_FOC_USE_FIXED_POINT

static const char * tag = "ESP_FOC_CONTROL_IQ31";

static void inverter_isr_iq31(void *data)
{
    esp_foc_axis_iq31_t *axis = (esp_foc_axis_iq31_t *)data;

    if(axis->downsampling_low_speed) {
        axis->downsampling_low_speed--;
        if(!axis->downsampling_low_speed) {
            axis->downsampling_low_speed = ESP_FOC_LOW_SPEED_DOWNSAMPLING;
            esp_foc_send_notification(axis->low_speed_ev);
        }
    }
}

void do_current_mode_sensored_high_speed_loop_iq31(void *arg)
{
    /* timestamp the current readings */
    uint64_t * current_timestamp = (uint64_t *)arg;
    *current_timestamp = esp_foc_now_useconds();
}

void do_current_mode_sensored_low_speed_loop_iq31(void *arg)
{
    esp_foc_axis_iq31_t *axis = (esp_foc_axis_iq31_t *)arg;
    isensor_values_iq31_t ival_q31;
    iq31_t iu_q31;
    iq31_t iv_q31;
    iq31_t iw_q31;
    iq31_t current_speed_q31;
    float low_speed_inv_dt = 1.0f / (iq31_to_float(axis->dt) * ESP_FOC_LOW_SPEED_DOWNSAMPLING);
    float raw_speed;
    float elec_delta;
    float theta_timestamp = 0.0f;
    uint64_t current_timestamp = 0;
    bool no_isensor = (axis->isensor_driver == NULL) ? true : false;

    axis->low_speed_ev = esp_foc_get_event_handle();
    axis->downsampling_low_speed = ESP_FOC_LOW_SPEED_DOWNSAMPLING;

    ESP_LOGI(tag,"Starting the current mode sensored low speed loop (IQ31)");

    iq31_t ticks_q31 = axis->rotor_sensor_driver->read_counts_iq31(axis->rotor_sensor_driver);
    axis->rotor_shaft_ticks = ticks_q31;
    axis->rotor_position = iq31_mul(ticks_q31, axis->natural_direction);
    axis->rotor_position_prev = axis->rotor_position;
    axis->current_speed = 0;

    ESP_LOGI(tag,"Starting current mode sensored high speed loop (IQ31)");

    axis->inverter_driver->set_inverter_callback(axis->inverter_driver,
        inverter_isr_iq31,
        axis);

    if(!no_isensor) {
        axis->isensor_driver->set_isensor_callback(axis->isensor_driver,
            axis->high_speed_loop_cb,
            &current_timestamp);
    }

    while(1) {
        esp_foc_wait_notifier();

#ifdef CONFIG_ESP_FOC_DEBUG_CORE_TIMING
       esp_foc_debug_pin_set();
#endif
        if(!no_isensor) {
            /* The ADC readings are already buffered while the angle sensor is being read */
            axis->isensor_driver->fetch_isensors_iq31(axis->isensor_driver, &ival_q31);
            iu_q31 = ival_q31.iu_axis_0;
            iv_q31 = ival_q31.iv_axis_0;
            iw_q31 = ival_q31.iw_axis_0;
            axis->i_u = iu_q31;
            axis->i_v = iv_q31;
            axis->i_w = iw_q31;

            /* Start new sample immediately; next loop uses it */
            axis->isensor_driver->sample_isensors(axis->isensor_driver);
        }

        ticks_q31 = axis->rotor_sensor_driver->read_counts_iq31(axis->rotor_sensor_driver);
        axis->rotor_shaft_ticks = ticks_q31;
        axis->rotor_position = iq31_mul(ticks_q31, axis->natural_direction);

        theta_timestamp = fabs(esp_foc_now_seconds() - ((float)(current_timestamp) * 0.000001f));

        elec_delta = iq31_to_float(iq31_mul(axis->current_speed, axis->motor_pole_pairs));
        axis->rotor_elec_angle = iq31_mul(axis->rotor_position, axis->motor_pole_pairs);
        axis->rotor_elec_angle = iq31_normalize_angle(
            iq31_sub(axis->rotor_elec_angle, iq31_from_float(elec_delta * theta_timestamp)));

        iq31_t elec_angle_norm_q31 = iq31_normalize_angle(axis->rotor_elec_angle);
        iq31_t e_sin_q31 = iq31_sin(elec_angle_norm_q31);
        iq31_t e_cos_q31 = iq31_cos(elec_angle_norm_q31);

        raw_speed = iq31_to_float(iq31_sub(axis->rotor_position, axis->rotor_position_prev)) * low_speed_inv_dt;
        current_speed_q31 = esp_foc_low_pass_filter_update_iq31(&axis->velocity_filter, iq31_from_float(raw_speed));
        axis->current_speed = current_speed_q31;
        axis->rotor_position_prev = axis->rotor_position;

        if(!no_isensor) {
            iq31_t i_alpha_q31, i_beta_q31, i_q_q31, i_d_q31;
            esp_foc_get_dq_currents_iq31(e_sin_q31,
                e_cos_q31,
                axis->i_u,
                axis->i_v,
                axis->i_w,
                &i_alpha_q31,
                &i_beta_q31,
                &i_q_q31,
                &i_d_q31);

            axis->i_alpha.raw = i_alpha_q31;
            axis->i_beta.raw = i_beta_q31;
            axis->i_q.raw = i_q_q31;
            axis->i_d.raw = i_d_q31;

            if (!axis->skip_torque_control) {
                iq31_t i_q_filt_q31 = esp_foc_low_pass_filter_update_iq31(&axis->current_filters[0], i_q_q31);
                iq31_t i_d_filt_q31 = esp_foc_low_pass_filter_update_iq31(&axis->current_filters[1], i_d_q31);
                iq31_t u_q_q31 = esp_foc_pid_update_iq31(&axis->torque_controller[0],
                                                         axis->target_i_q.raw,
                                                         i_q_filt_q31);
                iq31_t u_d_q31 = esp_foc_pid_update_iq31(&axis->torque_controller[1],
                                                         axis->target_i_d.raw,
                                                         i_d_filt_q31);
                u_q_q31 = iq31_add(u_q_q31, axis->target_u_q.raw);
                u_d_q31 = iq31_add(u_d_q31, axis->target_u_d.raw);
                axis->u_q.raw = u_q_q31;
                axis->u_d.raw = u_d_q31;
            } else {
                axis->u_q.raw = axis->target_u_q.raw;
                axis->u_d.raw = axis->target_u_d.raw;
            }
        }
        if (no_isensor) {
            axis->u_q.raw = axis->target_u_q.raw;
            axis->u_d.raw = axis->target_u_d.raw;
        }

        iq31_t u_alpha_q31, u_beta_q31, u_u_q31, u_v_q31, u_w_q31;
        esp_foc_modulate_dq_voltage_iq31(e_sin_q31,
                        e_cos_q31,
                        axis->u_d.raw,
                        axis->u_q.raw,
                        &u_alpha_q31,
                        &u_beta_q31,
                        &u_u_q31,
                        &u_v_q31,
                        &u_w_q31,
                        iq31_from_float(axis->max_voltage),
                        iq31_from_float(axis->biased_dc_link_voltage),
                        iq31_from_float(axis->dc_link_to_normalized));

        axis->u_alpha.raw = u_alpha_q31;
        axis->u_beta.raw = u_beta_q31;
        axis->u_u.raw = u_u_q31;
        axis->u_v.raw = u_v_q31;
        axis->u_w.raw = u_w_q31;
        axis->inverter_driver->set_voltages_iq31(axis->inverter_driver, u_u_q31, u_v_q31, u_w_q31);

        esp_foc_send_notification(axis->regulator_ev);

#ifdef CONFIG_ESP_FOC_SCOPE
        esp_foc_scope_data_push();
#endif

#ifdef CONFIG_ESP_FOC_DEBUG_CORE_TIMING
       esp_foc_debug_pin_clear();
#endif
    }
}

#endif /* CONFIG_ESP_FOC_USE_FIXED_POINT */
