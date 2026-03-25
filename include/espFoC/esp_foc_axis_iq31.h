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

#include "espFoC/esp_foc.h"
#include "espFoC/esp_foc_units_iq31.h"
#include "espFoC/utils/pid_controller_iq31.h"
#include "espFoC/utils/ema_low_pass_filter_iq31.h"

typedef struct esp_foc_axis_iq31_s esp_foc_axis_iq31_t;

typedef void (*esp_foc_high_speed_loop_callback_iq31_t)(void *arg);
typedef void (*esp_foc_low_speed_loop_callback_iq31_t)(void *arg);
typedef void (*esp_foc_outer_loop_callback_iq31_t)(void *arg);
typedef void (*esp_foc_motor_regulation_callback_iq31_t)(
    esp_foc_axis_iq31_t *axis,
    esp_foc_d_current_iq31_t *id_ref,
    esp_foc_q_current_iq31_t *iq_ref,
    esp_foc_d_voltage_iq31_t *ud_forward,
    esp_foc_q_voltage_iq31_t *uq_forward);

typedef struct {
    esp_foc_motor_direction_t natural_direction;
    int motor_pole_pairs;
    iq31_t motor_resistance;
    iq31_t motor_inductance;
    iq31_t motor_inertia;
    int motor_unit;
} esp_foc_motor_control_settings_iq31_t;

struct esp_foc_axis_iq31_s {
    iq31_t i_u;
    iq31_t i_v;
    iq31_t i_w;

    esp_foc_q_current_iq31_t target_i_q;
    esp_foc_d_current_iq31_t target_i_d;

    esp_foc_q_voltage_iq31_t target_u_q;
    esp_foc_d_voltage_iq31_t target_u_d;

    esp_foc_q_current_iq31_t i_q;
    esp_foc_d_current_iq31_t i_d;

    esp_foc_q_voltage_iq31_t u_q;
    esp_foc_d_voltage_iq31_t u_d;

    esp_foc_alpha_current_iq31_t i_alpha;
    esp_foc_beta_current_iq31_t i_beta;
    esp_foc_alpha_voltage_iq31_t u_alpha;
    esp_foc_beta_voltage_iq31_t u_beta;
    esp_foc_u_voltage_iq31_t u_u;
    esp_foc_v_voltage_iq31_t u_v;
    esp_foc_w_voltage_iq31_t u_w;

    iq31_t dt;
    iq31_t inv_dt;

    iq31_t current_speed;
    iq31_t shaft_ticks_to_radians_ratio;
    iq31_t extrapolated_rotor_position;
    iq31_t rotor_shaft_ticks;
    iq31_t rotor_position;
    iq31_t rotor_position_prev;
    iq31_t rotor_elec_angle;

    int downsampling_low_speed;
    int skip_torque_control;

    iq31_t dc_link_voltage;
    iq31_t biased_dc_link_voltage;
    iq31_t dc_link_to_normalized;
    iq31_t max_voltage;
    iq31_t motor_pole_pairs;
    iq31_t natural_direction;

    iq31_t current_offsets[3];

    esp_foc_err_t rotor_aligned;
    esp_foc_pid_iq31_t torque_controller[2];
    esp_foc_lp_filter_iq31_t velocity_filter;
    esp_foc_lp_filter_iq31_t current_filters[2];

    esp_foc_inverter_t *inverter_driver;
    esp_foc_rotor_sensor_t *rotor_sensor_driver;
    esp_foc_isensor_t *isensor_driver;

    esp_foc_event_handle_t low_speed_ev;
    esp_foc_event_handle_t regulator_ev;

    esp_foc_high_speed_loop_callback_iq31_t high_speed_loop_cb;
    esp_foc_low_speed_loop_callback_iq31_t low_speed_loop_cb;
    esp_foc_outer_loop_callback_iq31_t outer_loop_cb;
    esp_foc_motor_regulation_callback_iq31_t regulator_cb;
};

