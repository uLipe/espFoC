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

#include <stdbool.h>
#include "esp_log.h"
#include "espFoC/esp_foc.h"
#include "espFoC/utils/foc_math_q16.h"

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

void do_current_mode_sensored_high_speed_loop(void *arg)
{
    /* timestamp the current readings */
    uint64_t * current_timestamp = (uint64_t *)arg;
    *current_timestamp = esp_foc_now_useconds();
}

void do_current_mode_sensored_low_speed_loop(void *arg)
{
    esp_foc_axis_t *axis = (esp_foc_axis_t *)arg;
    isensor_values_t ival_q16;
    q16_t iu_q16;
    q16_t iv_q16;
    q16_t iw_q16;
    q16_t current_speed_q16;
    uint64_t current_timestamp = 0;
    bool no_isensor = (axis->isensor_driver == NULL) ? true : false;

    axis->low_speed_ev = esp_foc_get_event_handle();
    axis->downsampling_low_speed = ESP_FOC_LOW_SPEED_DOWNSAMPLING;

    ESP_LOGI(tag,"Starting the current mode sensored low speed loop");

    q16_t ticks_q16 = axis->rotor_sensor_driver->read_counts(axis->rotor_sensor_driver);
    axis->rotor_shaft_ticks = ticks_q16;
    axis->rotor_position = q16_mul(ticks_q16, axis->natural_direction);
    axis->rotor_position_prev = axis->rotor_position;
    axis->current_speed = 0;

    ESP_LOGI(tag,"Starting current mode sensored high speed loop");

    axis->inverter_driver->set_inverter_callback(axis->inverter_driver,
        inverter_isr,
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
            axis->isensor_driver->fetch_isensors(axis->isensor_driver, &ival_q16);
            iu_q16 = ival_q16.iu_axis_0;
            iv_q16 = ival_q16.iv_axis_0;
            iw_q16 = ival_q16.iw_axis_0;
            axis->i_u = iu_q16;
            axis->i_v = iv_q16;
            axis->i_w = iw_q16;

            /* Start new sample immediately; next loop uses it */
            axis->isensor_driver->sample_isensors(axis->isensor_driver);
        }

        ticks_q16 = axis->rotor_sensor_driver->read_counts(axis->rotor_sensor_driver);
        axis->rotor_shaft_ticks = ticks_q16;
        axis->rotor_position = q16_mul(ticks_q16, axis->natural_direction);

        {
            int pp = axis->motor_pole_pairs;
            int64_t et = (int64_t)axis->rotor_position * (int64_t)pp;
            q16_t elec_frac = (q16_t)(et % (int64_t)Q16_ONE);
            if (elec_frac < 0) {
                elec_frac = (q16_t)((int64_t)elec_frac + (int64_t)Q16_ONE);
            }
            q16_t elec_from_enc = q16_mul(elec_frac, Q16_TWO_PI);
            q16_t omega_e = q16_mul(axis->current_speed, q16_from_int(pp));
            uint64_t now_us = esp_foc_now_useconds();
            uint64_t us_abs = (now_us >= current_timestamp)
                                  ? (now_us - current_timestamp)
                                  : (current_timestamp - now_us);
            q16_t theta_ts = q16_from_elapsed_us_u64(us_abs);
            axis->rotor_elec_angle = q16_normalize_angle_rad(
                q16_sub(elec_from_enc, q16_mul(omega_e, theta_ts)));
        }

        q16_t elec_angle_norm_q16 = q16_normalize_angle_rad(axis->rotor_elec_angle);
        q16_t e_sin_q16 = q16_sin(elec_angle_norm_q16);
        q16_t e_cos_q16 = q16_cos(elec_angle_norm_q16);

        /* ω [rad/s] in Q16: Δθ [rad] * (1/T_low) [1/s]; inv_dt matches core init (1/(dt_hs*N)). */
        {
            q16_t dtheta = q16_sub(axis->rotor_position, axis->rotor_position_prev);
            q16_t raw_speed_q16 = q16_mul(dtheta, axis->torque_controller[0].inv_dt);
            current_speed_q16 = esp_foc_low_pass_filter_update(&axis->velocity_filter, raw_speed_q16);
        }
        axis->current_speed = current_speed_q16;
        axis->rotor_position_prev = axis->rotor_position;

        if(!no_isensor) {
            q16_t i_alpha_q16, i_beta_q16, i_q_q16, i_d_q16;
            esp_foc_get_dq_currents(e_sin_q16,
                e_cos_q16,
                axis->i_u,
                axis->i_v,
                axis->i_w,
                &i_alpha_q16,
                &i_beta_q16,
                &i_q_q16,
                &i_d_q16);

            axis->i_alpha.raw = i_alpha_q16;
            axis->i_beta.raw = i_beta_q16;
            axis->i_q.raw = i_q_q16;
            axis->i_d.raw = i_d_q16;

            if (!axis->skip_torque_control) {
                q16_t i_q_filt_q16 = esp_foc_low_pass_filter_update(&axis->current_filters[0], i_q_q16);
                q16_t i_d_filt_q16 = esp_foc_low_pass_filter_update(&axis->current_filters[1], i_d_q16);
                q16_t u_q_q16 = esp_foc_pid_update(&axis->torque_controller[0],
                                                         axis->target_i_q.raw,
                                                         i_q_filt_q16);
                q16_t u_d_q16 = esp_foc_pid_update(&axis->torque_controller[1],
                                                         axis->target_i_d.raw,
                                                         i_d_filt_q16);
                u_q_q16 = q16_add(u_q_q16, axis->target_u_q.raw);
                u_d_q16 = q16_add(u_d_q16, axis->target_u_d.raw);
                axis->u_q.raw = u_q_q16;
                axis->u_d.raw = u_d_q16;
            } else {
                axis->u_q.raw = axis->target_u_q.raw;
                axis->u_d.raw = axis->target_u_d.raw;
            }
        }
        if (no_isensor) {
            axis->u_q.raw = axis->target_u_q.raw;
            axis->u_d.raw = axis->target_u_d.raw;
        }

        q16_t u_alpha_q16, u_beta_q16, u_u_q16, u_v_q16, u_w_q16;
        esp_foc_modulate_dq_voltage(e_sin_q16,
                        e_cos_q16,
                        axis->u_d.raw,
                        axis->u_q.raw,
                        &u_alpha_q16,
                        &u_beta_q16,
                        &u_u_q16,
                        &u_v_q16,
                        &u_w_q16,
                        axis->max_voltage,
                        axis->biased_dc_link_voltage,
                        axis->dc_link_to_normalized);

        axis->u_alpha.raw = u_alpha_q16;
        axis->u_beta.raw = u_beta_q16;
        axis->u_u.raw = u_u_q16;
        axis->u_v.raw = u_v_q16;
        axis->u_w.raw = u_w_q16;
        axis->inverter_driver->set_voltages(axis->inverter_driver, u_u_q16, u_v_q16, u_w_q16);

        esp_foc_send_notification(axis->regulator_ev);

#ifdef CONFIG_ESP_FOC_SCOPE
        esp_foc_scope_data_push();
#endif

#ifdef CONFIG_ESP_FOC_DEBUG_CORE_TIMING
       esp_foc_debug_pin_clear();
#endif
    }
}

