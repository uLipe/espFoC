/*
 * Mock drivers for espFoC unit tests.
 */
#include "mock_drivers.h"
#include <string.h>
#include <stddef.h>
#include <math.h>
#if CONFIG_ESP_FOC_USE_FIXED_POINT
#include "espFoC/utils/esp_foc_iq31.h"
#endif

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

static float mock_get_dc_link_voltage(esp_foc_inverter_t *self)
{
    mock_inverter_t *m = MOCK_INVERTER_FROM_SELF(self);
    return m->dc_link_V;
}

static void mock_set_voltages(esp_foc_inverter_t *self, float v_u, float v_v, float v_w)
{
    mock_inverter_t *m = MOCK_INVERTER_FROM_SELF(self);
    m->set_voltages_count++;
    m->last_v_u = v_u;
    m->last_v_v = v_v;
    m->last_v_w = v_w;
}

#if CONFIG_ESP_FOC_USE_FIXED_POINT
static void mock_set_voltages_iq31(esp_foc_inverter_t *self, iq31_t v_u, iq31_t v_v, iq31_t v_w)
{
    mock_inverter_t *m = MOCK_INVERTER_FROM_SELF(self);
    m->set_voltages_iq31_count++;
    m->last_v_u_iq31 = v_u;
    m->last_v_v_iq31 = v_v;
    m->last_v_w_iq31 = v_w;
}
#endif

static void mock_phase_remap(esp_foc_inverter_t *self)
{
    (void)self;
}

static float mock_get_inverter_pwm_rate(esp_foc_inverter_t *self)
{
    mock_inverter_t *m = MOCK_INVERTER_FROM_SELF(self);
    return m->pwm_rate_hz;
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

void mock_inverter_init(mock_inverter_t *m, float dc_link_V, float pwm_rate_hz)
{
    memset(m, 0, sizeof(*m));
    m->dc_link_V = dc_link_V;
    m->pwm_rate_hz = pwm_rate_hz;
    m->interface.set_inverter_callback = mock_set_inverter_callback;
    m->interface.get_dc_link_voltage = mock_get_dc_link_voltage;
    m->interface.set_voltages = mock_set_voltages;
    m->interface.phase_remap = mock_phase_remap;
    m->interface.get_inverter_pwm_rate = mock_get_inverter_pwm_rate;
    m->interface.enable = mock_enable;
    m->interface.disable = mock_disable;
#if CONFIG_ESP_FOC_USE_FIXED_POINT
    m->interface.set_voltages_iq31 = mock_set_voltages_iq31;
#endif
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

/* --- Rotor sensor mock --- */
static void mock_rotor_set_to_zero(esp_foc_rotor_sensor_t *self)
{
    mock_rotor_sensor_t *m = MOCK_ROTOR_FROM_SELF(self);
    m->set_to_zero_count++;
    m->counts = 0.0f;
    m->accumulated = 0.0f;
}

static float mock_rotor_get_counts_per_revolution(esp_foc_rotor_sensor_t *self)
{
    mock_rotor_sensor_t *m = MOCK_ROTOR_FROM_SELF(self);
    return m->counts_per_rev;
}

static float mock_rotor_read_counts(esp_foc_rotor_sensor_t *self)
{
    mock_rotor_sensor_t *m = MOCK_ROTOR_FROM_SELF(self);
    m->read_counts_count++;
    return m->counts;
}

static float mock_rotor_read_accumulated_counts(esp_foc_rotor_sensor_t *self)
{
    mock_rotor_sensor_t *m = MOCK_ROTOR_FROM_SELF(self);
    m->read_accumulated_count++;
    return m->accumulated;
}

static void mock_rotor_set_simulation_count(esp_foc_rotor_sensor_t *self, float increment)
{
    mock_rotor_sensor_t *m = MOCK_ROTOR_FROM_SELF(self);
    m->set_simulation_count_count++;
    m->counts += increment;
    m->accumulated += increment;
}

#if CONFIG_ESP_FOC_USE_FIXED_POINT
static iq31_t mock_rotor_read_counts_iq31(esp_foc_rotor_sensor_t *self)
{
    mock_rotor_sensor_t *m = MOCK_ROTOR_FROM_SELF(self);
    m->read_counts_iq31_count++;
    m->last_angle_iq31 = iq31_from_float(m->counts / m->counts_per_rev);
    return m->last_angle_iq31;
}

static int64_t mock_rotor_read_accumulated_i64(esp_foc_rotor_sensor_t *self)
{
    mock_rotor_sensor_t *m = MOCK_ROTOR_FROM_SELF(self);
    m->read_accumulated_i64_count++;
    m->last_accum_i64 = (int64_t)llroundf(m->accumulated);
    return m->last_accum_i64;
}

static void mock_rotor_set_simulation_count_iq31(esp_foc_rotor_sensor_t *self, iq31_t increment_normalized)
{
    mock_rotor_sensor_t *m = MOCK_ROTOR_FROM_SELF(self);
    m->set_simulation_count_iq31_count++;
    m->counts += iq31_to_float(increment_normalized) * m->counts_per_rev;
    m->accumulated += iq31_to_float(increment_normalized) * m->counts_per_rev;
}
#endif

void mock_rotor_sensor_init(mock_rotor_sensor_t *m, float counts_per_rev)
{
    memset(m, 0, sizeof(*m));
    m->counts_per_rev = counts_per_rev;
    m->interface.set_to_zero = mock_rotor_set_to_zero;
    m->interface.get_counts_per_revolution = mock_rotor_get_counts_per_revolution;
    m->interface.read_counts = mock_rotor_read_counts;
    m->interface.read_accumulated_counts = mock_rotor_read_accumulated_counts;
    m->interface.set_simulation_count = mock_rotor_set_simulation_count;
#if CONFIG_ESP_FOC_USE_FIXED_POINT
    m->interface.read_counts_iq31 = mock_rotor_read_counts_iq31;
    m->interface.read_accumulated_counts_i64 = mock_rotor_read_accumulated_i64;
    m->interface.set_simulation_count_iq31 = mock_rotor_set_simulation_count_iq31;
#endif
}

esp_foc_rotor_sensor_t *mock_rotor_sensor_interface(mock_rotor_sensor_t *m)
{
    return &m->interface;
}

/* --- Current sensor mock --- */
static void mock_isensor_fetch(esp_foc_isensor_t *self, isensor_values_t *values)
{
    mock_isensor_t *m = MOCK_ISENSOR_FROM_SELF(self);
    m->fetch_count++;
    *values = m->values;
}

#if CONFIG_ESP_FOC_USE_FIXED_POINT
static void mock_isensor_fetch_iq31(esp_foc_isensor_t *self, isensor_values_iq31_t *values)
{
    mock_isensor_t *m = MOCK_ISENSOR_FROM_SELF(self);
    m->fetch_iq31_count++;
    m->values_iq31.iu_axis_0 = iq31_from_float(m->values.iu_axis_0);
    m->values_iq31.iv_axis_0 = iq31_from_float(m->values.iv_axis_0);
    m->values_iq31.iw_axis_0 = iq31_from_float(m->values.iw_axis_0);
    m->values_iq31.iu_axis_1 = iq31_from_float(m->values.iu_axis_1);
    m->values_iq31.iv_axis_1 = iq31_from_float(m->values.iv_axis_1);
    m->values_iq31.iw_axis_1 = iq31_from_float(m->values.iw_axis_1);
    *values = m->values_iq31;
}
#endif

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
#if CONFIG_ESP_FOC_USE_FIXED_POINT
    m->interface.fetch_isensors_iq31 = mock_isensor_fetch_iq31;
#endif
}

esp_foc_isensor_t *mock_isensor_interface(mock_isensor_t *m)
{
    return &m->interface;
}
