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
#include "espFoC/utils/ema_low_pass_filter.h"
#include "espFoC/utils/foc_math.h"
#include "espFoC/utils/space_vector_modulator.h"
#include "espFoC/utils/modulator.h"
#include "espFoC/utils/pid_controller.h"
#include "espFoC/drivers/inverter_interface.h"
#include "espFoC/drivers/current_sensor_interface.h"
#include "espFoC/drivers/rotor_sensor_interface.h"
#include "espFoC/osal/os_interface.h"
#include "espFoC/observer/esp_foc_observer_interface.h"
#include "espFoC/esp_foc_units.h"

typedef enum {
    ESP_FOC_OK = 0,
    ESP_FOC_ERR_NOT_ALIGNED = -1,
    ESP_FOC_ERR_INVALID_ARG = -2,
    ESP_FOC_ERR_AXIS_INVALID_STATE = -3,
    ESP_FOC_ERR_ALIGNMENT_IN_PROGRESS = -4,
    ESP_FOC_ERR_TIMESTEP_TOO_SMALL = -5,
    ESP_FOC_ERR_ROTOR_STARTUP = -6,
    ESP_FOC_ERR_ROTOR_STARTUP_PI = -7,
    ESP_FOC_ERR_UNKNOWN = -128
} esp_foc_err_t;

typedef struct esp_foc_axis_s esp_foc_axis_t;

typedef void (*esp_foc_motor_regulation_callback_t) (esp_foc_axis_t *axis, esp_foc_d_current *id_ref, esp_foc_q_current *iq_ref,
                                                    esp_foc_d_voltage *ud_forward, esp_foc_q_voltage *uq_forward);


#include "espFoC/esp_foc_axis.h"
#include "espFoC/esp_foc_controls.h"

typedef enum {
    ESP_FOC_MOTOR_NATURAL_DIRECTION_CW,
    ESP_FOC_MOTOR_NATURAL_DIRECTION_CCW,
} esp_foc_motor_direction_t;

typedef struct {
    esp_foc_motor_direction_t natural_direction;
    int motor_pole_pairs;
    float motor_resistance;
    float motor_inductance;
    float motor_inertia;
    int motor_unit;
} esp_foc_motor_control_settings_t;

#include "espFoC/esp_foc_scope.h"

esp_foc_err_t esp_foc_initialize_axis(esp_foc_axis_t *axis, esp_foc_inverter_t *inverter, esp_foc_rotor_sensor_t *rotor,
                                    esp_foc_isensor_t *isensor, esp_foc_motor_control_settings_t settings);
esp_foc_err_t esp_foc_align_axis(esp_foc_axis_t *axis);
esp_foc_err_t esp_foc_run(esp_foc_axis_t *axis);
esp_foc_err_t esp_foc_set_regulation_callback(esp_foc_axis_t *axis, esp_foc_motor_regulation_callback_t callback);

#include "espFoC/esp_foc_servo_speed.h"
#include "espFoC/esp_foc_servo_position.h"
