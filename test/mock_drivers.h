/*
 * Mock drivers for espFoC unit tests.
 */
#pragma once

#include <stdint.h>
#include "espFoC/drivers/esp_foc_inverter.h"
#include "espFoC/drivers/esp_foc_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    esp_foc_inverter_t interface;
    float dc_link_volts;
    float pwm_rate_hz;
    int set_duties_count;
    q16_t last_duty_a, last_duty_b, last_duty_c;
    int enable_count;
    int disable_count;
    int set_callback_count;
    esp_foc_inverter_callback_t saved_callback;
    void *saved_callback_arg;
    esp_foc_inverter_isensor_values_t isensor_values;
    int fetch_count;
    int sample_count;
    int calibrate_count;
    int set_filter_cutoff_count;
    float last_filter_fc;
    float last_filter_fs;
    int set_publish_targets_count;
    q16_t *publish_alpha_target;
    q16_t *publish_beta_target;
    q16_t *publish_iu_target;
    q16_t *publish_iv_target;
} mock_inverter_t;

void mock_inverter_init(mock_inverter_t *m, float dc_link_volts, float pwm_rate_hz);
esp_foc_inverter_t *mock_inverter_interface(mock_inverter_t *m);
void mock_inverter_trigger_callback(mock_inverter_t *m);

typedef struct {
    esp_foc_encoder_t interface;
    float counts_per_rev;
    float counts;
    float accumulated;
    int set_to_zero_count;
    int read_counts_count;
    int read_accumulated_i64_count;
    int set_simulation_count_count;
    q16_t last_angle_q16;
    int64_t last_accum_i64;
    int64_t scripted_reads[8];
    int scripted_count;
    int scripted_idx;
    float scripted_counts_reads[8];
    int scripted_counts_count;
    int scripted_counts_idx;
} mock_encoder_t;

void mock_encoder_init(mock_encoder_t *m, float counts_per_rev);
esp_foc_encoder_t *mock_encoder_interface(mock_encoder_t *m);
void mock_encoder_script_accumulated(mock_encoder_t *m,
                                     const int64_t *seq,
                                     int n);
void mock_encoder_script_counts(mock_encoder_t *m,
                                const float *seq,
                                int n);

#ifdef __cplusplus
}
#endif
