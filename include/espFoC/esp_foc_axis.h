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

typedef struct {
    esp_foc_q_current target_i_q;
    esp_foc_d_current target_i_d;

    esp_foc_q_current i_q;
    esp_foc_d_current i_d;

    esp_foc_q_voltage u_q;
    esp_foc_d_voltage u_d;

    esp_foc_q_voltage target_u_q;
    esp_foc_d_voltage target_u_d;

    esp_foc_alpha_voltage u_alpha;
    esp_foc_beta_voltage u_beta;

    esp_foc_u_voltage u_u;
    esp_foc_v_voltage u_v;
    esp_foc_w_voltage u_w;

    esp_foc_alpha_current i_alpha;
    esp_foc_beta_current i_beta;

    esp_foc_u_current i_u;
    esp_foc_v_current i_v;
    esp_foc_w_current i_w;

    int control_loop_downsampling;
    int control_loop_reload_val;

    float target_speed;
    float current_speed;
    float shaft_ticks_to_radians_ratio;
    float dt;
    float last_timestamp;
    int downsampling_speed_reload_value;
    int downsampling_speed;

    float target_position;
    float rotor_position;
    float accumulated_rotor_position;
    float rotor_position_prev;
    float rotor_shaft_ticks;
    float rotor_elec_angle;
    int downsampling_position_reload_value;
    int downsampling_position;

    float dc_link_voltage;
    float biased_dc_link_voltage;
    float motor_pole_pairs;
    float natural_direction;
    
    float estimators_sample_rate;
    int downsampling_estimators;
    int downsampling_estimators_reload_val;

    esp_foc_err_t rotor_aligned;
    esp_foc_pid_controller_t velocity_controller;
    esp_foc_pid_controller_t torque_controller[2];
    esp_foc_pid_controller_t position_controller;
    esp_foc_lp_filter_t velocity_filter;
    esp_foc_lp_filter_t current_filters[2];

    esp_foc_inverter_t * inverter_driver;
    esp_foc_rotor_sensor_t *rotor_sensor_driver;
    esp_foc_isensor_t *isensor_driver;
    esp_foc_event_handle_t ev_handle;
    esp_foc_event_handle_t control_handle;

} esp_foc_axis_t;
