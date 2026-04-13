/*
 * Unit tests for driver interfaces using mocks (IQ31-only API).
 */
#include <unity.h>
#include <math.h>
#include "mock_drivers.h"
#include "espFoC/utils/esp_foc_q16.h"

#define FLOAT_TOL 1e-5f

TEST_CASE("mock inverter: set_voltages records args", "[espFoC][driver_mock]")
{
    mock_inverter_t inv;
    mock_inverter_init(&inv, 1.0f, 20000.0f);
    esp_foc_inverter_t *iface = mock_inverter_interface(&inv);

    q16_t u = q16_from_float(0.5f);
    q16_t v = q16_from_float(0.25f);
    q16_t w = q16_from_float(-0.25f);
    iface->set_voltages(iface, u, v, w);

    TEST_ASSERT_EQUAL(1, inv.set_voltages_count);
    TEST_ASSERT_EQUAL(u, inv.last_v_u);
    TEST_ASSERT_EQUAL(v, inv.last_v_v);
    TEST_ASSERT_EQUAL(w, inv.last_v_w);
}

TEST_CASE("mock inverter: get_dc_link_voltage and get_pwm_rate", "[espFoC][driver_mock]")
{
    mock_inverter_t inv;
    mock_inverter_init(&inv, 1.0f, 40000.0f);
    esp_foc_inverter_t *iface = mock_inverter_interface(&inv);

    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 1.0f, q16_to_float(iface->get_dc_link_voltage(iface)));
    TEST_ASSERT_EQUAL(40000u, iface->get_inverter_pwm_rate(iface));
}

TEST_CASE("mock inverter: enable and disable counted", "[espFoC][driver_mock]")
{
    mock_inverter_t inv;
    mock_inverter_init(&inv, 1.0f, 10000.0f);
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
    mock_inverter_init(&inv, 1.0f, 10000.0f);
    esp_foc_inverter_t *iface = mock_inverter_interface(&inv);

    mock_triggered_flag = 0;
    iface->set_inverter_callback(iface, mock_callback_set_flag, NULL);
    TEST_ASSERT_EQUAL(1, inv.set_callback_count);

    mock_inverter_trigger_callback(&inv);
    TEST_ASSERT_EQUAL(1, mock_triggered_flag);
}

TEST_CASE("mock rotor: read_counts and get_counts_per_rev", "[espFoC][driver_mock]")
{
    mock_rotor_sensor_t rot;
    mock_rotor_sensor_init(&rot, 4096.0f);
    rot.counts = 1024.0f;
    esp_foc_rotor_sensor_t *iface = mock_rotor_sensor_interface(&rot);

    TEST_ASSERT_EQUAL(4096u, iface->get_counts_per_revolution(iface));
    q16_t ang = iface->read_counts(iface);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 1024.0f / 4096.0f, q16_to_float(ang));
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

    iface->set_simulation_count(iface, q16_from_float(0.0015f));
    iface->set_simulation_count(iface, q16_from_float(0.0025f));
    TEST_ASSERT_EQUAL(2, rot.set_simulation_count_count);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 4.0f, rot.counts);
}

TEST_CASE("mock isensor: fetch and calibrate counted", "[espFoC][driver_mock]")
{
    mock_isensor_t isen;
    mock_isensor_init(&isen);
    isen.values.iu_axis_0 = q16_from_float(0.1f);
    isen.values.iv_axis_0 = q16_from_float(-0.05f);
    isen.values.iw_axis_0 = q16_from_float(-0.05f);
    esp_foc_isensor_t *iface = mock_isensor_interface(&isen);

    isensor_values_t val;
    iface->fetch_isensors(iface, &val);
    TEST_ASSERT_EQUAL(1, isen.fetch_count);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.1f, q16_to_float(val.iu_axis_0));

    iface->calibrate_isensors(iface, 10);
    iface->sample_isensors(iface);
    TEST_ASSERT_EQUAL(1, isen.calibrate_count);
    TEST_ASSERT_EQUAL(1, isen.sample_count);
}
