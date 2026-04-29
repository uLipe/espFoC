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
#define MOCK_ROTOR_FROM_SELF(self) \
    ((mock_rotor_sensor_t *)((char *)(self) - offsetof(mock_rotor_sensor_t, interface)))
#define MOCK_ISENSOR_FROM_SELF(self) \
    ((mock_isensor_t *)((char *)(self) - offsetof(mock_isensor_t, interface)))

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
    return q16_from_float(m->dc_link_pu);
}

static void mock_set_voltages(esp_foc_inverter_t *self, q16_t v_u, q16_t v_v, q16_t v_w)
{
    mock_inverter_t *m = MOCK_INVERTER_FROM_SELF(self);
    m->set_voltages_count++;
    m->last_v_u = v_u;
    m->last_v_v = v_v;
    m->last_v_w = v_w;
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

void mock_inverter_init(mock_inverter_t *m, float dc_link_pu, float pwm_rate_hz)
{
    memset(m, 0, sizeof(*m));
    m->dc_link_pu = dc_link_pu;
    m->pwm_rate_hz = pwm_rate_hz;
    m->interface.set_inverter_callback = mock_set_inverter_callback;
    m->interface.get_dc_link_voltage = mock_get_dc_link_voltage;
    m->interface.set_voltages = mock_set_voltages;
    m->interface.get_inverter_pwm_rate = mock_get_inverter_pwm_rate;
    m->interface.enable = mock_enable;
    m->interface.disable = mock_disable;
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

static void mock_rotor_set_to_zero(esp_foc_rotor_sensor_t *self)
{
    mock_rotor_sensor_t *m = MOCK_ROTOR_FROM_SELF(self);
    m->set_to_zero_count++;
    m->counts = 0.0f;
    m->accumulated = 0.0f;
}

static uint32_t mock_rotor_get_counts_per_revolution(esp_foc_rotor_sensor_t *self)
{
    mock_rotor_sensor_t *m = MOCK_ROTOR_FROM_SELF(self);
    return (uint32_t)(m->counts_per_rev + 0.5f);
}

static q16_t mock_rotor_read_counts(esp_foc_rotor_sensor_t *self)
{
    mock_rotor_sensor_t *m = MOCK_ROTOR_FROM_SELF(self);
    m->read_counts_count++;
    {
        float c = fmodf(m->counts, m->counts_per_rev);
        if (c < 0.0f) {
            c += m->counts_per_rev;
        }
        m->last_angle_q16 = q16_from_float(c);
    }
    return m->last_angle_q16;
}

static int64_t mock_rotor_read_accumulated_i64(esp_foc_rotor_sensor_t *self)
{
    mock_rotor_sensor_t *m = MOCK_ROTOR_FROM_SELF(self);
    m->read_accumulated_i64_count++;
    if (m->scripted_idx < m->scripted_count) {
        m->last_accum_i64 = m->scripted_reads[m->scripted_idx++];
    } else {
        m->last_accum_i64 = (int64_t)llroundf(m->accumulated);
    }
    return m->last_accum_i64;
}

void mock_rotor_sensor_script_accumulated(mock_rotor_sensor_t *m,
                                          const int64_t *seq, int n)
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

static void mock_rotor_set_simulation_count(esp_foc_rotor_sensor_t *self, q16_t increment_normalized)
{
    mock_rotor_sensor_t *m = MOCK_ROTOR_FROM_SELF(self);
    m->set_simulation_count_count++;
    m->counts += q16_to_float(increment_normalized) * m->counts_per_rev;
    m->accumulated += q16_to_float(increment_normalized) * m->counts_per_rev;
}

void mock_rotor_sensor_init(mock_rotor_sensor_t *m, float counts_per_rev)
{
    memset(m, 0, sizeof(*m));
    m->counts_per_rev = counts_per_rev;
    m->interface.set_to_zero = mock_rotor_set_to_zero;
    m->interface.get_counts_per_revolution = mock_rotor_get_counts_per_revolution;
    m->interface.read_counts = mock_rotor_read_counts;
    m->interface.read_accumulated_counts_i64 = mock_rotor_read_accumulated_i64;
    m->interface.set_simulation_count = mock_rotor_set_simulation_count;
    m->interface.set_zero_offset_raw_12b = NULL;
    m->interface.get_zero_offset_12b = NULL;
}

esp_foc_rotor_sensor_t *mock_rotor_sensor_interface(mock_rotor_sensor_t *m)
{
    return &m->interface;
}

static void mock_isensor_fetch(esp_foc_isensor_t *self, isensor_values_t *values)
{
    mock_isensor_t *m = MOCK_ISENSOR_FROM_SELF(self);
    m->fetch_count++;
    *values = m->values;
}

static void mock_isensor_sample(esp_foc_isensor_t *self)
{
    mock_isensor_t *m = MOCK_ISENSOR_FROM_SELF(self);
    m->sample_count++;
}

static void mock_isensor_calibrate(esp_foc_isensor_t *self, int calibration_rounds)
{
    mock_isensor_t *m = MOCK_ISENSOR_FROM_SELF(self);
    m->calibrate_count++;
    (void)calibration_rounds;
}

static void mock_isensor_set_filter_cutoff(esp_foc_isensor_t *self, float fc, float fs)
{
    mock_isensor_t *m = MOCK_ISENSOR_FROM_SELF(self);
    m->set_filter_cutoff_count++;
    m->last_filter_fc = fc;
    m->last_filter_fs = fs;
}

static void mock_isensor_set_publish_targets(esp_foc_isensor_t *self,
                                             q16_t *i_alpha_target,
                                             q16_t *i_beta_target,
                                             q16_t *i_u_target,
                                             q16_t *i_v_target)
{
    mock_isensor_t *m = MOCK_ISENSOR_FROM_SELF(self);
    m->set_publish_targets_count++;
    m->publish_alpha_target = i_alpha_target;
    m->publish_beta_target  = i_beta_target;
    m->publish_iu_target = i_u_target;
    m->publish_iv_target = i_v_target;
}

static void mock_isensor_set_callback(esp_foc_isensor_t *self, isensor_callback_t cb, void *param)
{
    mock_isensor_t *m = MOCK_ISENSOR_FROM_SELF(self);
    m->set_callback_count++;
    m->saved_callback = cb;
    m->saved_callback_param = param;
}

void mock_isensor_init(mock_isensor_t *m)
{
    memset(m, 0, sizeof(*m));
    m->interface.fetch_isensors = mock_isensor_fetch;
    m->interface.sample_isensors = mock_isensor_sample;
    m->interface.calibrate_isensors = mock_isensor_calibrate;
    m->interface.set_isensor_callback = mock_isensor_set_callback;
    m->interface.set_filter_cutoff = mock_isensor_set_filter_cutoff;
    m->interface.set_publish_targets = mock_isensor_set_publish_targets;
}

esp_foc_isensor_t *mock_isensor_interface(mock_isensor_t *m)
{
    return &m->interface;
}
