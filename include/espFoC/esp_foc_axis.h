/*
 * MIT License
 */
#pragma once

#include <stdbool.h>
#include <sdkconfig.h>
#include "espFoC/esp_foc_err.h"
#include "espFoC/esp_foc_units_q16.h"
#include "espFoC/utils/pid_controller.h"
#include "espFoC/esp_foc_estimator_q16.h"
#include "espFoC/drivers/esp_foc_inverter.h"
#include "espFoC/drivers/esp_foc_encoder.h"
#include "espFoC/osal/os_interface.h"

typedef struct esp_foc_axis_s esp_foc_axis_t;

typedef void (*esp_foc_motor_regulation_callback_t)(
    esp_foc_axis_t *axis,
    esp_foc_d_current_q16_t *id_ref,
    esp_foc_q_current_q16_t *iq_ref);

typedef struct {
    uint8_t axis_id;
} esp_foc_axis_cal_cache_t;

typedef struct {
    esp_foc_motor_direction_t natural_direction;
    int motor_pole_pairs;
    int motor_unit;
} esp_foc_motor_control_settings_t;

struct esp_foc_axis_s {
    q16_t i_u;
    q16_t i_v;
    q16_t i_w;

    esp_foc_q_current_q16_t target_i_q;
    esp_foc_d_current_q16_t target_i_d;

    esp_foc_q_current_q16_t i_q;
    esp_foc_d_current_q16_t i_d;

    esp_foc_q_voltage_q16_t u_q;
    esp_foc_d_voltage_q16_t u_d;

    esp_foc_alpha_current_q16_t i_alpha;
    esp_foc_beta_current_q16_t i_beta;
    esp_foc_alpha_voltage_q16_t u_alpha;
    esp_foc_beta_voltage_q16_t u_beta;
    esp_foc_u_voltage_q16_t u_u;
    esp_foc_v_voltage_q16_t u_v;
    esp_foc_w_voltage_q16_t u_w;

    q16_t dt;
    q16_t inv_dt;

    volatile q16_t current_speed;
    q16_t rotor_shaft_ticks;
    q16_t rotor_position;
    q16_t encoder_inv_cpr_q16;
    volatile q16_t rotor_elec_angle;
    esp_foc_estimator_q16_t rotor_estimator;

    int downsampling_low_speed;

    q16_t vdc_q16;
    q16_t mod_index_limit_q16;
    int motor_pole_pairs;
    esp_foc_axis_cal_cache_t cal;
    q16_t natural_direction;

    esp_foc_axis_state_t state;
    esp_foc_axis_mode_t mode;
    q16_t bench_theta_e;
    volatile bool runner_shutdown;
    void *runner_low_speed_hdl;
    void *runner_outer_hdl;

    esp_foc_pid_controller_t torque_controller[2];
    q16_t current_filter_fc_hz_q16;
    q16_t current_filter_fs_hz_q16;

    volatile q16_t latest_i_alpha;
    volatile q16_t latest_i_beta;

    esp_foc_inverter_t *inverter_driver;
    esp_foc_encoder_t *encoder_driver;

    esp_foc_event_handle_t low_speed_ev;
    esp_foc_event_handle_t regulator_ev;

    esp_foc_motor_regulation_callback_t regulator_cb;
};
