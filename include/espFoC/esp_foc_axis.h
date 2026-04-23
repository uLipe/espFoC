/*
 * MIT License
 */
#pragma once

#include <stdbool.h>
#include <sdkconfig.h>
#include "espFoC/esp_foc_units_q16.h"
#include "espFoC/utils/pid_controller.h"
#include "espFoC/utils/biquad_q16.h"
#include "espFoC/utils/angle_predictor_q16.h"
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
    /* NVS key; copied from settings.motor_unit in esp_foc_initialize_axis. */
    uint8_t nvs_axis_id;
    /* Last known motor params from NVS (tuner readback; q16, Ohm / H / Hz). */
    q16_t nvs_motor_r_ohm;
    q16_t nvs_motor_l_h;
    q16_t nvs_bandwidth_hz;
    q16_t natural_direction;

    esp_foc_err_t rotor_aligned;
    esp_foc_pid_controller_t torque_controller[2];
    /* Velocity smoothing biquad. Cutoff dialled at init from
     * CONFIG_ESP_FOC_VELOCITY_FILTER_CUTOFF_HZ. The current i_q / i_d
     * filtering used to live here as well; it now sits inside the
     * isensor driver (per-phase, on raw ADC counts), which is the
     * only placement that survives the move into the PWM ISR. */
    esp_foc_biquad_q16_t velocity_filter;
    /* Current-sense low-pass cutoff (Hz, q16) currently programmed in
     * the isensor driver. The driver owns the biquad coefficients;
     * this field is just so the tuner can read the value back without
     * a driver-side accessor. Sample rate fed to the biquad designer
     * is current_filter_fs_hz_q16, captured at init. */
    q16_t current_filter_fc_hz_q16;
    q16_t current_filter_fs_hz_q16;

#if defined(CONFIG_ESP_FOC_ISR_HOT_PATH)
    /* Plan #2: full FOC pipeline runs inside the PWM ISR. The slow
     * encoder is read from the outer task, fed into the alpha-beta
     * predictor; the ISR queries predict() to get a fresh extrapolated
     * electrical angle every PWM cycle. */
    esp_foc_angle_predictor_q16_t angle_predictor;
    /* ADC ISR atomic-writes Clarke output here; PWM ISR reads. Both
     * are 32-bit aligned q16, single-instruction stores on Xtensa /
     * RISC-V — no torn-read concern. The volatile keeps the compiler
     * from caching the read across loop iterations. */
    volatile q16_t latest_i_alpha;
    volatile q16_t latest_i_beta;
#endif

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
