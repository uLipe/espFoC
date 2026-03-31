/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <math.h>
#include <sdkconfig.h>
#include "espFoC/utils/foc_math.h"
#include "espFoC/utils/modulator.h"
#include "espFoC/drivers/inverter_interface.h"
#include "espFoC/drivers/current_sensor_interface.h"
#include "espFoC/drivers/rotor_sensor_interface.h"
#include "espFoC/osal/os_interface.h"
#include "espFoC/observer/esp_foc_observer_interface.h"

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

typedef enum {
    ESP_FOC_MOTOR_NATURAL_DIRECTION_CW,
    ESP_FOC_MOTOR_NATURAL_DIRECTION_CCW,
} esp_foc_motor_direction_t;

#include "espFoC/esp_foc_axis.h"
#include "espFoC/esp_foc_controls.h"
#include "espFoC/esp_foc_scope.h"

esp_foc_err_t esp_foc_initialize_axis(esp_foc_axis_t *axis,
                                        esp_foc_inverter_t *inverter,
                                        esp_foc_rotor_sensor_t *rotor,
                                        esp_foc_isensor_t *isensor,
                                        esp_foc_motor_control_settings_t settings);
esp_foc_err_t esp_foc_align_axis(esp_foc_axis_t *axis);
esp_foc_err_t esp_foc_run(esp_foc_axis_t *axis);
esp_foc_err_t esp_foc_set_regulation_callback(esp_foc_axis_t *axis,
                                              esp_foc_motor_regulation_callback_t callback);
