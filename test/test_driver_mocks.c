/*
 * Unit tests for driver interfaces using mocks.
 * Verifies that mock inverter, rotor sensor, and current sensor record
 * calls and return values correctly (simplest level).
 */
#include <unity.h>
#include <math.h>
#include "mock_drivers.h"

#define FLOAT_TOL 1e-5f

/* --- Mock inverter API --- */
TEST_CASE("mock inverter: set_voltages records args", "[espFoC][driver_mock]")
{
    mock_inverter_t inv;
    mock_inverter_init(&inv, 24.0f, 20000.0f);
    esp_foc_inverter_t *iface = mock_inverter_interface(&inv);

    iface->set_voltages(iface, 0.5f, 0.25f, -0.25f);

    TEST_ASSERT_EQUAL(1, inv.set_voltages_count);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.5f, inv.last_v_u);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.25f, inv.last_v_v);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, -0.25f, inv.last_v_w);
}

TEST_CASE("mock inverter: get_dc_link_voltage and get_pwm_rate", "[espFoC][driver_mock]")
{
    mock_inverter_t inv;
    mock_inverter_init(&inv, 48.0f, 40000.0f);
    esp_foc_inverter_t *iface = mock_inverter_interface(&inv);

    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 48.0f, iface->get_dc_link_voltage(iface));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 40000.0f, iface->get_inverter_pwm_rate(iface));
}

TEST_CASE("mock inverter: enable and disable counted", "[espFoC][driver_mock]")
{
    mock_inverter_t inv;
    mock_inverter_init(&inv, 24.0f, 10000.0f);
    esp_foc_inverter_t *iface = mock_inverter_interface(&inv);

    iface->enable(iface);
    iface->enable(iface);
    iface->disable(iface);

    TEST_ASSERT_EQUAL(2, inv.enable_count);
    TEST_ASSERT_EQUAL(1, inv.disable_count);
}

static int mock_triggered_flag;
static void mock_callback_set_flag(void *arg)
{
    (void)arg;
    mock_triggered_flag = 1;
}

TEST_CASE("mock inverter: set_callback and trigger", "[espFoC][driver_mock]")
{
    mock_inverter_t inv;
    mock_inverter_init(&inv, 24.0f, 10000.0f);
    esp_foc_inverter_t *iface = mock_inverter_interface(&inv);

    mock_triggered_flag = 0;
    iface->set_inverter_callback(iface, mock_callback_set_flag, NULL);
    TEST_ASSERT_EQUAL(1, inv.set_callback_count);

    mock_inverter_trigger_callback(&inv);
    TEST_ASSERT_EQUAL(1, mock_triggered_flag);
}

/* --- Mock rotor sensor API --- */
TEST_CASE("mock rotor: read_counts and get_counts_per_rev", "[espFoC][driver_mock]")
{
    mock_rotor_sensor_t rot;
    mock_rotor_sensor_init(&rot, 4096.0f);
    rot.counts = 1024.0f;
    esp_foc_rotor_sensor_t *iface = mock_rotor_sensor_interface(&rot);

    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 4096.0f, iface->get_counts_per_revolution(iface));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 1024.0f, iface->read_counts(iface));
    TEST_ASSERT_EQUAL(1, rot.read_counts_count);
}

TEST_CASE("mock rotor: set_to_zero clears counts", "[espFoC][driver_mock]")
{
    mock_rotor_sensor_t rot;
    mock_rotor_sensor_init(&rot, 2048.0f);
    rot.counts = 100.0f;
    esp_foc_rotor_sensor_t *iface = mock_rotor_sensor_interface(&rot);

    iface->set_to_zero(iface);
    TEST_ASSERT_EQUAL(1, rot.set_to_zero_count);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.0f, rot.counts);
}

TEST_CASE("mock rotor: set_simulation_count increments", "[espFoC][driver_mock]")
{
    mock_rotor_sensor_t rot;
    mock_rotor_sensor_init(&rot, 1000.0f);
    esp_foc_rotor_sensor_t *iface = mock_rotor_sensor_interface(&rot);

    iface->set_simulation_count(iface, 1.5f);
    iface->set_simulation_count(iface, 2.5f);
    TEST_ASSERT_EQUAL(2, rot.set_simulation_count_count);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 4.0f, rot.counts);
}

/* --- Mock current sensor API --- */
TEST_CASE("mock isensor: fetch and calibrate counted", "[espFoC][driver_mock]")
{
    mock_isensor_t isen;
    mock_isensor_init(&isen);
    isen.values.iu_axis_0 = 0.1f;
    isen.values.iv_axis_0 = -0.05f;
    isen.values.iw_axis_0 = -0.05f;
    esp_foc_isensor_t *iface = mock_isensor_interface(&isen);

    isensor_values_t val;
    iface->fetch_isensors(iface, &val);
    TEST_ASSERT_EQUAL(1, isen.fetch_count);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.1f, val.iu_axis_0);

    iface->calibrate_isensors(iface, 10);
    iface->sample_isensors(iface);
    TEST_ASSERT_EQUAL(1, isen.calibrate_count);
    TEST_ASSERT_EQUAL(1, isen.sample_count);
}
