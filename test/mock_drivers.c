/*
 * Mock drivers for espFoC unit tests.
 */
#include "mock_drivers.h"
#include <string.h>
#include <stddef.h>
#include <math.h>
#include "espFoC/utils/esp_foc_q16.h"

#define MOCK_INVERTER_FROM_SELF(self) \
    ((mock_inverter_t *)((char *)(self) - offsetof(mock_inverter_t, interface)))
#define MOCK_ENCODER_FROM_SELF(self) \
    ((mock_encoder_t *)((char *)(self) - offsetof(mock_encoder_t, interface)))

static void mock_set_inverter_callback(esp_foc_inverter_t *self,
                                       esp_foc_inverter_callback_t callback,
                                       void *argument)
{
    mock_inverter_t *m = MOCK_INVERTER_FROM_SELF(self);
    m->set_callback_count++;
    m->saved_callback = callback;
    m->saved_callback_arg = argument;
}

static q16_t mock_get_dc_link_voltage(esp_foc_inverter_t *self)
{
    mock_inverter_t *m = MOCK_INVERTER_FROM_SELF(self);
    return q16_from_float(m->dc_link_volts);
}

static void mock_set_duties(esp_foc_inverter_t *self, q16_t duty_a, q16_t duty_b, q16_t duty_c)
{
    mock_inverter_t *m = MOCK_INVERTER_FROM_SELF(self);
    m->set_duties_count++;
    m->last_duty_a = duty_a;
    m->last_duty_b = duty_b;
    m->last_duty_c = duty_c;
}

static uint32_t mock_get_inverter_pwm_rate(esp_foc_inverter_t *self)
{
    mock_inverter_t *m = MOCK_INVERTER_FROM_SELF(self);
    return (uint32_t)(m->pwm_rate_hz + 0.5f);
}

static void mock_enable(esp_foc_inverter_t *self)
{
    mock_inverter_t *m = MOCK_INVERTER_FROM_SELF(self);
    m->enable_count++;
}

static void mock_disable(esp_foc_inverter_t *self)
{
    mock_inverter_t *m = MOCK_INVERTER_FROM_SELF(self);
    m->disable_count++;
}

static void mock_fetch_isensors(esp_foc_inverter_t *self, esp_foc_inverter_isensor_values_t *values)
{
    mock_inverter_t *m = MOCK_INVERTER_FROM_SELF(self);
    m->fetch_count++;
    if (values != NULL) {
        *values = m->isensor_values;
    }
}

static void mock_sample_isensors(esp_foc_inverter_t *self)
{
    mock_inverter_t *m = MOCK_INVERTER_FROM_SELF(self);
    m->sample_count++;
}

static void mock_calibrate_isensors(esp_foc_inverter_t *self, int calibration_rounds)
{
    mock_inverter_t *m = MOCK_INVERTER_FROM_SELF(self);
    m->calibrate_count++;
    (void)calibration_rounds;
}

static void mock_set_filter_cutoff(esp_foc_inverter_t *self, float fc, float fs)
{
    mock_inverter_t *m = MOCK_INVERTER_FROM_SELF(self);
    m->set_filter_cutoff_count++;
    m->last_filter_fc = fc;
    m->last_filter_fs = fs;
}

static void mock_set_publish_targets(esp_foc_inverter_t *self,
                                     q16_t *i_alpha_target,
                                     q16_t *i_beta_target,
                                     q16_t *i_u_target,
                                     q16_t *i_v_target)
{
    mock_inverter_t *m = MOCK_INVERTER_FROM_SELF(self);
    m->set_publish_targets_count++;
    m->publish_alpha_target = i_alpha_target;
    m->publish_beta_target = i_beta_target;
    m->publish_iu_target = i_u_target;
    m->publish_iv_target = i_v_target;
}

void mock_inverter_init(mock_inverter_t *m, float dc_link_volts, float pwm_rate_hz)
{
    memset(m, 0, sizeof(*m));
    m->dc_link_volts = dc_link_volts;
    m->pwm_rate_hz = pwm_rate_hz;
    m->interface.set_inverter_callback = mock_set_inverter_callback;
    m->interface.get_dc_link_voltage = mock_get_dc_link_voltage;
    m->interface.set_duties = mock_set_duties;
    m->interface.get_inverter_pwm_rate = mock_get_inverter_pwm_rate;
    m->interface.enable = mock_enable;
    m->interface.disable = mock_disable;
    m->interface.fetch_isensors = mock_fetch_isensors;
    m->interface.sample_isensors = mock_sample_isensors;
    m->interface.calibrate_isensors = mock_calibrate_isensors;
    m->interface.set_filter_cutoff = mock_set_filter_cutoff;
    m->interface.set_publish_targets = mock_set_publish_targets;
}

esp_foc_inverter_t *mock_inverter_interface(mock_inverter_t *m)
{
    return &m->interface;
}

void mock_inverter_trigger_callback(mock_inverter_t *m)
{
    if (m->saved_callback) {
        m->saved_callback(m->saved_callback_arg);
    }
}

static void mock_encoder_set_to_zero(esp_foc_encoder_t *self)
{
    mock_encoder_t *m = MOCK_ENCODER_FROM_SELF(self);
    m->set_to_zero_count++;
    m->counts = 0.0f;
    m->accumulated = 0.0f;
}

static uint32_t mock_encoder_get_counts_per_revolution(esp_foc_encoder_t *self)
{
    mock_encoder_t *m = MOCK_ENCODER_FROM_SELF(self);
    return (uint32_t)(m->counts_per_rev + 0.5f);
}

static q16_t mock_encoder_read_counts(esp_foc_encoder_t *self)
{
    mock_encoder_t *m = MOCK_ENCODER_FROM_SELF(self);
    m->read_counts_count++;
    if (m->scripted_counts_idx < m->scripted_counts_count) {
        m->counts = m->scripted_counts_reads[m->scripted_counts_idx++];
    }
    m->last_angle_q16 = q16_from_float(m->counts);
    return m->last_angle_q16;
}

static int64_t mock_encoder_read_accumulated_i64(esp_foc_encoder_t *self)
{
    mock_encoder_t *m = MOCK_ENCODER_FROM_SELF(self);
    m->read_accumulated_i64_count++;
    if (m->scripted_idx < m->scripted_count) {
        m->last_accum_i64 = m->scripted_reads[m->scripted_idx++];
    } else {
        m->last_accum_i64 = (int64_t)llroundf(m->accumulated);
    }
    return m->last_accum_i64;
}

void mock_encoder_script_counts(mock_encoder_t *m, const float *seq, int n)
{
    if (m == NULL || seq == NULL) {
        return;
    }
    if (n < 0) {
        n = 0;
    }
    int cap = (int)(sizeof(m->scripted_counts_reads) / sizeof(m->scripted_counts_reads[0]));
    if (n > cap) {
        n = cap;
    }
    for (int i = 0; i < n; ++i) {
        m->scripted_counts_reads[i] = seq[i];
    }
    m->scripted_counts_count = n;
    m->scripted_counts_idx = 0;
}

void mock_encoder_script_accumulated(mock_encoder_t *m, const int64_t *seq, int n)
{
    if (m == NULL || seq == NULL) {
        return;
    }
    if (n < 0) {
        n = 0;
    }
    int cap = (int)(sizeof(m->scripted_reads) / sizeof(m->scripted_reads[0]));
    if (n > cap) {
        n = cap;
    }
    for (int i = 0; i < n; ++i) {
        m->scripted_reads[i] = seq[i];
    }
    m->scripted_count = n;
    m->scripted_idx = 0;
}

static void mock_encoder_set_simulation_count(esp_foc_encoder_t *self, q16_t increment_normalized)
{
    mock_encoder_t *m = MOCK_ENCODER_FROM_SELF(self);
    m->set_simulation_count_count++;
    m->counts += q16_to_float(increment_normalized) * m->counts_per_rev;
    m->accumulated += q16_to_float(increment_normalized) * m->counts_per_rev;
}

void mock_encoder_init(mock_encoder_t *m, float counts_per_rev)
{
    memset(m, 0, sizeof(*m));
    m->counts_per_rev = counts_per_rev;
    m->interface.set_to_zero = mock_encoder_set_to_zero;
    m->interface.get_counts_per_revolution = mock_encoder_get_counts_per_revolution;
    m->interface.read_counts = mock_encoder_read_counts;
    m->interface.read_accumulated_counts_i64 = mock_encoder_read_accumulated_i64;
    m->interface.set_simulation_count = mock_encoder_set_simulation_count;
    m->interface.set_zero_offset_raw_12b = NULL;
    m->interface.get_zero_offset_12b = NULL;
}

esp_foc_encoder_t *mock_encoder_interface(mock_encoder_t *m)
{
    return &m->interface;
}
