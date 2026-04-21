/*
 * MIT License
 */
#pragma once

#include <stdbool.h>
#include <sdkconfig.h>
#include "espFoC/esp_foc_units_q16.h"
#include "espFoC/utils/pid_controller.h"
#include "espFoC/utils/ema_low_pass_filter.h"
#include "espFoC/esp_foc_injection.h"
#include "espFoC/drivers/inverter_interface.h"
#include "espFoC/drivers/current_sensor_interface.h"
#include "espFoC/drivers/rotor_sensor_interface.h"
#include "espFoC/osal/os_interface.h"

/* Magic number used to validate axis pointers handed to the runtime tuner.
 * Lives only when CONFIG_ESP_FOC_TUNER_ENABLE is set so nano builds pay
 * zero overhead. Set in esp_foc_initialize_axis(). */
#define ESP_FOC_AXIS_MAGIC ((uint32_t)0xF0CA1515)

typedef struct esp_foc_axis_s esp_foc_axis_t;

typedef void (*esp_foc_high_speed_loop_callback_t)(void *arg);
typedef void (*esp_foc_low_speed_loop_callback_t)(void *arg);
typedef void (*esp_foc_outer_loop_callback_t)(void *arg);
typedef void (*esp_foc_motor_regulation_callback_t)(
    esp_foc_axis_t *axis,
    esp_foc_d_current_q16_t *id_ref,
    esp_foc_q_current_q16_t *iq_ref,
    esp_foc_d_voltage_q16_t *ud_forward,
    esp_foc_q_voltage_q16_t *uq_forward);

typedef struct {
    esp_foc_motor_direction_t natural_direction;  /* hint; alignment can override */
    int motor_pole_pairs;
    int motor_unit;
} esp_foc_motor_control_settings_t;

#if defined(CONFIG_ESP_FOC_TUNER_ENABLE)
typedef struct {
    bool active;
    q16_t target_id;
    q16_t target_iq;
    q16_t target_ud;
    q16_t target_uq;
} esp_foc_tuner_override_t;
#endif

struct esp_foc_axis_s {
#if defined(CONFIG_ESP_FOC_TUNER_ENABLE)
    /* Validates this struct came from esp_foc_initialize_axis() before any
     * tuner-driven mutation. Defends against stale or wild axis pointers. */
    uint32_t magic;
    esp_foc_tuner_override_t tuner_override;
#endif

    q16_t i_u;
    q16_t i_v;
    q16_t i_w;

    esp_foc_q_current_q16_t target_i_q;
    esp_foc_d_current_q16_t target_i_d;

    esp_foc_q_voltage_q16_t target_u_q;
    esp_foc_d_voltage_q16_t target_u_d;

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

    q16_t current_speed;
    q16_t rotor_shaft_ticks;
    q16_t rotor_position;
    q16_t rotor_position_prev;
    q16_t rotor_elec_angle;

    int downsampling_low_speed;
    int skip_torque_control;

    q16_t dc_link_voltage;
    q16_t dc_link_to_normalized;
    q16_t max_voltage;
    int motor_pole_pairs;
    q16_t natural_direction;

    esp_foc_err_t rotor_aligned;
    esp_foc_pid_controller_t torque_controller[2];
    esp_foc_lp_filter_t velocity_filter;
    esp_foc_lp_filter_t current_filters[2];

    esp_foc_injection_t injection;

    esp_foc_inverter_t *inverter_driver;
    esp_foc_rotor_sensor_t *rotor_sensor_driver;
    esp_foc_isensor_t *isensor_driver;

    esp_foc_event_handle_t low_speed_ev;
    esp_foc_event_handle_t regulator_ev;

    esp_foc_high_speed_loop_callback_t high_speed_loop_cb;
    esp_foc_low_speed_loop_callback_t low_speed_loop_cb;
    esp_foc_outer_loop_callback_t outer_loop_cb;
    esp_foc_motor_regulation_callback_t regulator_cb;
};
