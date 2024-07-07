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

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <errno.h>
#include "esp_foc_motor_interface.h"
#include "esp_foc_svm.h"
#include "esp_foc_pid.h"

#define ESP_FOC_INNER_CONTROL_US_PERIOD     (250)

struct esp_foc_motor_control {
    struct esp_foc_motor_interface *motor_hw;
    struct esp_foc_pid *pid_position;
    struct esp_foc_pid *pid_velocity;

    int position_control_cntr;
    int speed_control_cntr;

    float target_speed_dps;
    float target_position_degrees;
    float current_speed_dps;
    float current_position_degrees;

    bool is_enabled;
};

extern const int ESP_FOC_SPEED_CONTROL_RATIO;
extern const int ESP_FOC_POSITION_CONTROL_RATIO;

int esp_foc_init_controller(struct esp_foc_motor_control *ctl,
                            const struct esp_foc_motor_interface *hw);

int esp_foc_controller_set_speed(struct esp_foc_motor_control *ctl,
                                float target_speed);

int esp_foc_controller_set_position(struct esp_foc_motor_control *ctl,
                                float target_position);

int esp_foc_add_position_control(struct esp_foc_motor_control *ctl,
                                const struct esp_foc_pid *pos_pid);

int esp_foc_add_speed_control(struct esp_foc_motor_control *ctl,
                                const struct esp_foc_pid *speed_pid);

int esp_foc_enable_axis(struct esp_foc_motor_control *ctl);

int esp_foc_disable_axis(struct esp_foc_motor_control *ctl);

int esp_foc_controller_run(struct esp_foc_motor_control *ctl);
