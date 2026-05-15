/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#pragma once

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

typedef enum {
    ESP_FOC_AXIS_STATE_IDLE = 0,
    ESP_FOC_AXIS_STATE_ALIGNING = 1,
    ESP_FOC_AXIS_STATE_ALIGNED = 2,
    ESP_FOC_AXIS_STATE_RUNNING = 3,
} esp_foc_axis_state_t;
