/*
 * Unit tests for mock drivers (Q16 API).
 */
#include <unity.h>
#include <math.h>
#include "mock_drivers.h"
#include "espFoC/utils/esp_foc_q16.h"

#define FLOAT_TOL 1e-4f

TEST_CASE("mock inverter: set_duties records Q16 args", "[espFoC][driver_mock]")
{
    mock_inverter_t inv;
    mock_inverter_init(&inv, 48.0f, 20000.0f);
    esp_foc_inverter_t *iface = mock_inverter_interface(&inv);

    q16_t a = q16_from_float(0.5f);
    q16_t b = q16_from_float(0.25f);
    q16_t c = q16_from_float(0.0f);
    iface->set_duties(iface, a, b, c);

    TEST_ASSERT_EQUAL(1, inv.set_duties_count);
    TEST_ASSERT_EQUAL(a, inv.last_duty_a);
    TEST_ASSERT_EQUAL(b, inv.last_duty_b);
    TEST_ASSERT_EQUAL(c, inv.last_duty_c);
}

TEST_CASE("mock inverter: fetch_isensors returns Q16 values", "[espFoC][driver_mock]")
{
    mock_inverter_t inv;
    mock_inverter_init(&inv, 12.0f, 20000.0f);
    inv.isensor_values.iu_axis_0 = q16_from_float(0.25f);
    inv.isensor_values.iv_axis_0 = q16_from_float(-0.25f);
    inv.isensor_values.iw_axis_0 = q16_from_float(0.0f);
    inv.isensor_values.iu_axis_1 = q16_from_float(0.1f);
    inv.isensor_values.iv_axis_1 = q16_from_float(0.1f);
    inv.isensor_values.iw_axis_1 = q16_from_float(-0.2f);
    esp_foc_inverter_t *iface = mock_inverter_interface(&inv);

    esp_foc_inverter_isensor_values_t q;
    iface->fetch_isensors(iface, &q);
    TEST_ASSERT_EQUAL(1, inv.fetch_count);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.25f, q16_to_float(q.iu_axis_0));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, -0.25f, q16_to_float(q.iv_axis_0));
}

TEST_CASE("mock rotor: read_counts and accumulated i64", "[espFoC][driver_mock]")
{
    mock_encoder_t rot;
    mock_encoder_init(&rot, 4096.0f);
    rot.counts = 2048.0f;
    rot.accumulated = 5000.4f;
    esp_foc_encoder_t *iface = mock_encoder_interface(&rot);

    q16_t ang = iface->read_counts(iface);
    TEST_ASSERT_EQUAL(1, rot.read_counts_count);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 2048.0f, q16_to_float(ang));

    int64_t acc = iface->read_accumulated_counts_i64(iface);
    TEST_ASSERT_EQUAL(1, rot.read_accumulated_i64_count);
    TEST_ASSERT_EQUAL(5000LL, acc);
}

TEST_CASE("mock rotor: set_simulation_count scales by counts_per_rev", "[espFoC][driver_mock]")
{
    mock_encoder_t rot;
    mock_encoder_init(&rot, 1000.0f);
    esp_foc_encoder_t *iface = mock_encoder_interface(&rot);

    iface->set_simulation_count(iface, q16_from_float(0.25f));
    TEST_ASSERT_EQUAL(1, rot.set_simulation_count_count);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 250.0f, rot.counts);
}
