/*
 * Unit tests for mock drivers (Q16 API).
 */
#include <unity.h>
#include <math.h>
#include "mock_drivers.h"
#include "espFoC/utils/esp_foc_q16.h"

#define FLOAT_TOL 1e-4f

TEST_CASE("mock inverter: set_voltages records Q16 args", "[espFoC][driver_mock]")
{
    mock_inverter_t inv;
    mock_inverter_init(&inv, 1.0f, 20000.0f);
    esp_foc_inverter_t *iface = mock_inverter_interface(&inv);

    q16_t a = q16_from_float(0.5f);
    q16_t b = q16_from_float(0.25f);
    q16_t c = q16_from_float(0.0f);
    iface->set_voltages(iface, a, b, c);

    TEST_ASSERT_EQUAL(1, inv.set_voltages_count);
    TEST_ASSERT_EQUAL(a, inv.last_v_u);
    TEST_ASSERT_EQUAL(b, inv.last_v_v);
    TEST_ASSERT_EQUAL(c, inv.last_v_w);
}

TEST_CASE("mock isensor: fetch_isensors returns Q16 values", "[espFoC][driver_mock]")
{
    mock_isensor_t isen;
    mock_isensor_init(&isen);
    isen.values.iu_axis_0 = q16_from_float(0.25f);
    isen.values.iv_axis_0 = q16_from_float(-0.25f);
    isen.values.iw_axis_0 = q16_from_float(0.0f);
    isen.values.iu_axis_1 = q16_from_float(0.1f);
    isen.values.iv_axis_1 = q16_from_float(0.1f);
    isen.values.iw_axis_1 = q16_from_float(-0.2f);
    esp_foc_isensor_t *iface = mock_isensor_interface(&isen);

    isensor_values_t q;
    iface->fetch_isensors(iface, &q);
    TEST_ASSERT_EQUAL(1, isen.fetch_count);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.25f, q16_to_float(q.iu_axis_0));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, -0.25f, q16_to_float(q.iv_axis_0));
}

TEST_CASE("mock rotor: read_counts and accumulated i64", "[espFoC][driver_mock]")
{
    mock_rotor_sensor_t rot;
    mock_rotor_sensor_init(&rot, 4096.0f);
    rot.counts = 2048.0f;
    rot.accumulated = 5000.4f;
    esp_foc_rotor_sensor_t *iface = mock_rotor_sensor_interface(&rot);

    q16_t ang = iface->read_counts(iface);
    TEST_ASSERT_EQUAL(1, rot.read_counts_count);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.5f, q16_to_float(ang));

    int64_t acc = iface->read_accumulated_counts_i64(iface);
    TEST_ASSERT_EQUAL(1, rot.read_accumulated_i64_count);
    TEST_ASSERT_EQUAL(5000LL, acc);
}

TEST_CASE("mock rotor: set_simulation_count scales by counts_per_rev", "[espFoC][driver_mock]")
{
    mock_rotor_sensor_t rot;
    mock_rotor_sensor_init(&rot, 1000.0f);
    esp_foc_rotor_sensor_t *iface = mock_rotor_sensor_interface(&rot);

    iface->set_simulation_count(iface, q16_from_float(0.25f));
    TEST_ASSERT_EQUAL(1, rot.set_simulation_count_count);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 250.0f, rot.counts);
}
