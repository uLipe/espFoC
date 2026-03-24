/*
 * Mock drivers for espFoC unit tests.
 * Implement inverter, rotor sensor, and current sensor interfaces;
 * record calls and return configurable values for assertions.
 */
#pragma once

#include <stdint.h>
#include "espFoC/drivers/inverter_interface.h"
#include "espFoC/drivers/rotor_sensor_interface.h"
#include "espFoC/drivers/current_sensor_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --- Mock inverter --- */
typedef struct {
    esp_foc_inverter_t interface;
    float dc_link_V;
    float pwm_rate_hz;
    int set_voltages_count;
    float last_v_u, last_v_v, last_v_w;
#ifdef CONFIG_ESP_FOC_USE_FIXED_POINT
    int set_voltages_iq31_count;
    iq31_t last_v_u_iq31, last_v_v_iq31, last_v_w_iq31;
#endif
    int enable_count;
    int disable_count;
    int set_callback_count;
    esp_foc_inverter_callback_t saved_callback;
    void *saved_callback_arg;
} mock_inverter_t;

void mock_inverter_init(mock_inverter_t *m, float dc_link_V, float pwm_rate_hz);
esp_foc_inverter_t *mock_inverter_interface(mock_inverter_t *m);
void mock_inverter_trigger_callback(mock_inverter_t *m);

/* --- Mock rotor sensor --- */
typedef struct {
    esp_foc_rotor_sensor_t interface;
    float counts_per_rev;
    float counts;
    float accumulated;
    int set_to_zero_count;
    int read_counts_count;
    int read_accumulated_count;
    int set_simulation_count_count;
#ifdef CONFIG_ESP_FOC_USE_FIXED_POINT
    int read_counts_iq31_count;
    int read_accumulated_i64_count;
    int set_simulation_count_iq31_count;
    iq31_t last_angle_iq31;
    int64_t last_accum_i64;
#endif
} mock_rotor_sensor_t;

void mock_rotor_sensor_init(mock_rotor_sensor_t *m, float counts_per_rev);
esp_foc_rotor_sensor_t *mock_rotor_sensor_interface(mock_rotor_sensor_t *m);

/* --- Mock current sensor --- */
typedef struct {
    esp_foc_isensor_t interface;
    isensor_values_t values;
#ifdef CONFIG_ESP_FOC_USE_FIXED_POINT
    isensor_values_iq31_t values_iq31;
    int fetch_iq31_count;
#endif
    int fetch_count;
    int sample_count;
    int calibrate_count;
    int set_callback_count;
    isensor_callback_t saved_callback;
    void *saved_callback_param;
} mock_isensor_t;

void mock_isensor_init(mock_isensor_t *m);
esp_foc_isensor_t *mock_isensor_interface(mock_isensor_t *m);

#ifdef __cplusplus
}
#endif
