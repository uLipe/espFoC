/*
 * Mock drivers for espFoC unit tests.
 */
#pragma once

#include <stdint.h>
#include "espFoC/drivers/inverter_interface.h"
#include "espFoC/drivers/rotor_sensor_interface.h"
#include "espFoC/drivers/current_sensor_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    esp_foc_inverter_t interface;
    float dc_link_pu;
    float pwm_rate_hz;
    int set_voltages_count;
    q16_t last_v_u, last_v_v, last_v_w;
    int enable_count;
    int disable_count;
    int set_callback_count;
    esp_foc_inverter_callback_t saved_callback;
    void *saved_callback_arg;
} mock_inverter_t;

void mock_inverter_init(mock_inverter_t *m, float dc_link_pu, float pwm_rate_hz);
esp_foc_inverter_t *mock_inverter_interface(mock_inverter_t *m);
void mock_inverter_trigger_callback(mock_inverter_t *m);

typedef struct {
    esp_foc_rotor_sensor_t interface;
    float counts_per_rev;
    float counts;
    float accumulated;
    int set_to_zero_count;
    int read_counts_count;
    int read_accumulated_i64_count;
    int set_simulation_count_count;
    q16_t last_angle_q16;
    int64_t last_accum_i64;
} mock_rotor_sensor_t;

void mock_rotor_sensor_init(mock_rotor_sensor_t *m, float counts_per_rev);
esp_foc_rotor_sensor_t *mock_rotor_sensor_interface(mock_rotor_sensor_t *m);

typedef struct {
    esp_foc_isensor_t interface;
    isensor_values_t values;
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
