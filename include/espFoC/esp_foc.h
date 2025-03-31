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

#include <stdbool.h>
#include <math.h>
#include <sdkconfig.h>
#include "espFoC/ema_low_pass_filter.h"
#include "espFoC/foc_math.h"
#include "espFoC/space_vector_modulator.h"
#include "espFoC/modulator.h"
#include "espFoC/pid_controller.h"
#include "espFoC/inverter_interface.h"
#include "espFoC/current_sensor_interface.h"
#include "espFoC/rotor_sensor_interface.h"
#include "espFoC/os_interface.h"
#include "espFoC/esp_foc_units.h"
#include "espFoC/esp_foc_observer_interface.h"

typedef enum {
    ESP_FOC_OK = 0,
    ESP_FOC_ERR_NOT_ALIGNED = -1,
    ESP_FOC_ERR_INVALID_ARG = -2,
    ESP_FOC_ERR_AXIS_INVALID_STATE = -3,
    ESP_FOC_ERR_ALIGNMENT_IN_PROGRESS = -4,
    ESP_FOC_ERR_TIMESTEP_TOO_SMALL = -5,
    ESP_FOC_ERR_UNKNOWN = -128
} esp_foc_err_t;

#include "espFoC/esp_foc_axis.h"

typedef enum {
    ESP_FOC_MOTOR_NATURAL_DIRECTION_CW,
    ESP_FOC_MOTOR_NATURAL_DIRECTION_CCW,
} esp_foc_motor_direction_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float integrator_limit;
    float max_output_value;
} esp_foc_control_settings_t;

typedef struct {
    esp_foc_control_settings_t torque_control_settings[2];
    esp_foc_control_settings_t velocity_control_settings;
    esp_foc_control_settings_t position_control_settings;
    esp_foc_motor_direction_t natural_direction;
    bool enable_position_control;
    bool enable_velocity_control;
    bool enable_torque_control;
    int motor_pole_pairs;
    float motor_resistance;
    float motor_inductance;
    float motor_inertia;
    float flux_linkage;
    float inertia;
    float friction;
    int motor_unit;
} esp_foc_motor_control_settings_t;

typedef struct {
    esp_foc_seconds timestamp;

    esp_foc_seconds dt;
    esp_foc_u_voltage u;
    esp_foc_v_voltage v;
    esp_foc_w_voltage w;

    esp_foc_q_voltage out_q;
    esp_foc_d_voltage out_d;

    esp_foc_u_current i_u;
    esp_foc_v_current i_v;
    esp_foc_w_current i_w;

    esp_foc_q_current i_q;
    esp_foc_d_current i_d;

    esp_foc_alpha_voltage u_alpha;
    esp_foc_beta_voltage u_beta;

    esp_foc_alpha_current i_alpha;
    esp_foc_beta_current i_beta;

    esp_foc_radians rotor_position;
    esp_foc_radians position;
    esp_foc_radians_per_second speed;

    esp_foc_radians target_position;
    esp_foc_radians_per_second target_speed;
    esp_foc_radians_per_second observer_angle;

} esp_foc_control_data_t;

#include "espFoC/esp_foc_scope.h"

esp_foc_err_t esp_foc_initialize_axis(esp_foc_axis_t *axis,
                                    esp_foc_inverter_t *inverter,
                                    esp_foc_rotor_sensor_t *rotor,
                                    esp_foc_isensor_t *isensor,
                                    esp_foc_motor_control_settings_t settings);
esp_foc_err_t esp_foc_align_axis(esp_foc_axis_t *axis);
esp_foc_err_t esp_foc_run(esp_foc_axis_t *axis);

esp_foc_err_t esp_foc_set_target_voltage(esp_foc_axis_t *axis, esp_foc_q_voltage uq, esp_foc_d_voltage ud);
esp_foc_err_t esp_foc_set_target_current(esp_foc_axis_t *axis, esp_foc_q_current iq, esp_foc_d_current id);
esp_foc_err_t esp_foc_set_target_speed(esp_foc_axis_t *axis, esp_foc_radians_per_second speed);
esp_foc_err_t esp_foc_set_target_position(esp_foc_axis_t *axis, esp_foc_radians position);

esp_foc_err_t esp_foc_get_control_data(esp_foc_axis_t *axis, esp_foc_control_data_t *control_data);