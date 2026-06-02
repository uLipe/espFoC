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
#include "espFoC/drivers/inverter_interface.h"
#include "espFoC/drivers/current_sensor_interface.h"
#include "espFoC/drivers/rotor_sensor_interface.h"
#include "espFoC/osal/os_interface.h"

/* Magic number used to validate axis pointers handed to the runtime tuner.
 * Lives only when CONFIG_ESP_FOC_TUNER_ENABLE is set so nano builds pay
 * zero overhead. Set in esp_foc_initialize_axis(). */
#define ESP_FOC_AXIS_MAGIC ((uint32_t)0xF0CA1515)

typedef struct esp_foc_axis_s esp_foc_axis_t;

typedef void (*esp_foc_motor_regulation_callback_t)(
    esp_foc_axis_t *axis,
    esp_foc_d_current_q16_t *id_ref,
    esp_foc_q_current_q16_t *iq_ref);

typedef struct {
    uint8_t axis_id;
} esp_foc_axis_cal_cache_t;

typedef struct {
    esp_foc_motor_direction_t natural_direction;  /* hint; alignment can override */
    int motor_pole_pairs;
    int motor_unit;
} esp_foc_motor_control_settings_t;

struct esp_foc_axis_s {
#if defined(CONFIG_ESP_FOC_TUNER_ENABLE)
    /* Validates this struct came from esp_foc_initialize_axis() before any
     * tuner-driven mutation. Defends against stale or wild axis pointers. */
    uint32_t magic;
#endif

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

    /* Mechanical speed from encoder PLL [rev/s Q16]; ISR writes, tasks read. */
    volatile q16_t current_speed;
    /* Last `read_counts`: engineering counts in Q16 (e.g. 0..4095 for AS5600). */
    q16_t rotor_shaft_ticks;
    /* Encoder position × natural_direction, same count units as read_counts. */
    q16_t rotor_position;
    /* At init: q16_from_float(1/cpr); shaft_rev = q16_mul(counts_q16, this). */
    q16_t encoder_inv_cpr_q16;
    /* θe_norm ∈ [0, Q16_ONE) ↔ one electrical turn; ISR writes only. */
    volatile q16_t rotor_elec_angle;
    esp_foc_estimator_q16_t rotor_estimator;

    int downsampling_low_speed;

    /** Nominal DC bus [V] Q16, read once at init from the inverter driver. */
    q16_t vdc_q16;
    /** Circular |Vdq| cap in pu (typically ESP_FOC_MOD_INDEX_LIMIT_Q16). */
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
    /* Current-sense low-pass cutoff (Hz, q16) currently programmed in
     * the isensor driver. The driver owns the biquad coefficients;
     * this field is just so the tuner can read the value back without
     * a driver-side accessor. Sample rate fed to the biquad designer
     * is current_filter_fs_hz_q16, captured at init. */
    q16_t current_filter_fc_hz_q16;
    q16_t current_filter_fs_hz_q16;

    /* ADC ISR atomic-writes Clarke output here; PWM ISR reads. Both
     * are 32-bit aligned q16, single-instruction stores on Xtensa /
     * RISC-V — no torn-read concern. The volatile keeps the compiler
     * from caching the read across loop iterations. */
    volatile q16_t latest_i_alpha;
    volatile q16_t latest_i_beta;

    esp_foc_inverter_t *inverter_driver;
    esp_foc_rotor_sensor_t *rotor_sensor_driver;
    esp_foc_isensor_t *isensor_driver;

    esp_foc_event_handle_t low_speed_ev;
    esp_foc_event_handle_t regulator_ev;

    esp_foc_motor_regulation_callback_t regulator_cb;
};
