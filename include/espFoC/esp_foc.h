#pragma once

#include <math.h>
#include "espFoC/ema_low_pass_filter.h"
#include "espFoC/foc_math.h"
#include "espFoC/modulator.h"
#include "espFoC/pid_controller.h"
#include "espFoC/inverter_interface.h"
#include "espFoC/current_sensor_interface.h"
#include "espFoC/rotor_sensor_interface.h"
#include "espFoC/os_interface.h"

typedef enum {
    ESP_FOC_OK = 0,
    ESP_FOC_ERR_NOT_ALIGNED = -1,
    ESP_FOC_ERR_INVALID_ARG = -2,
    ESP_FOC_ERR_AXIS_INVALID_STATE = -3,
    ESP_FOC_ERR_ALIGNMENT_IN_PROGRESS = -4,
    ESP_FOC_ERR_TIMESTEP_TOO_SMALL = -5,
    ESP_FOC_ERR_UNKNOWN = -128
} esp_foc_err_t;

typedef struct {
    float raw;
} esp_foc_q_voltage;

typedef struct {
    float raw;
} esp_foc_d_voltage;

typedef struct {
    float v_uvw[3];
    float radians;
    float shaft_ticks;
    float v_qd[2];
} esp_foc_telemetry_t;

typedef struct {
    float v_qd[2];
    float v_ab[2];
    float v_uvw[3];

    float i_uvw[3];
    float i_ab[2];
    float i_qd[2];

    float target_speed;

    float rotor_position;
    float rotor_position_prev;
    float rotor_shaft_ticks;
    float shaft_ticks_to_radians_ratio;

    float dt;
    float velocity_dt;
    float last_timestamp;
    float dc_link_voltage;
    float biased_dc_link_voltage;

    float motor_pole_pairs;
    int inner_control_runs;

    esp_foc_err_t rotor_aligned;

    esp_foc_pid_controller_t velocity_controller;
    esp_foc_pid_controller_t torque_controller;
    esp_foc_pid_controller_t position_control;
    esp_foc_lp_filter_t velocity_filter;
    esp_foc_lp_filter_t current_filters[2];

    esp_foc_inverter_t * inverter_driver;
    esp_foc_rotor_sensor_t *rotor_sensor_driver;

} esp_foc_axis_t;

esp_foc_err_t esp_foc_initialize_axis(esp_foc_axis_t *axis,
                                    esp_foc_inverter_t *inverter,
                                    esp_foc_rotor_sensor_t *rotor,
                                    float motor_pole_pairs);

esp_foc_err_t esp_foc_align_axis(esp_foc_axis_t *axis);

esp_foc_err_t esp_foc_set_target_voltage(esp_foc_axis_t *axis,
                                        esp_foc_q_voltage *vq,
                                        esp_foc_d_voltage *vd);

esp_foc_err_t esp_foc_set_target_velocity(esp_foc_axis_t *axis, float radians);

esp_foc_err_t esp_foc_run(esp_foc_axis_t *axis);