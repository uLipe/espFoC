/*
 * Unit tests for mock driver IQ31 API (only when CONFIG_ESP_FOC_USE_FIXED_POINT=y).
 */
#include <unity.h>
#include <math.h>
#include "mock_drivers.h"

#if CONFIG_ESP_FOC_USE_FIXED_POINT
#include "espFoC/utils/esp_foc_iq31.h"

#define FLOAT_TOL 1e-4f

TEST_CASE("mock inverter: set_voltages_iq31 records IQ31 args", "[espFoC][driver_mock][iq31]")
{
    mock_inverter_t inv;
    mock_inverter_init(&inv, 24.0f, 20000.0f);
    esp_foc_inverter_t *iface = mock_inverter_interface(&inv);

    iq31_t a = iq31_from_float(0.5f);
    iq31_t b = iq31_from_float(0.25f);
    iq31_t c = iq31_from_float(0.0f);
    iface->set_voltages_iq31(iface, a, b, c);

    TEST_ASSERT_EQUAL(1, inv.set_voltages_iq31_count);
    TEST_ASSERT_EQUAL(a, inv.last_v_u_iq31);
    TEST_ASSERT_EQUAL(b, inv.last_v_v_iq31);
    TEST_ASSERT_EQUAL(c, inv.last_v_w_iq31);
}

TEST_CASE("mock isensor: fetch_isensors_iq31 matches float values", "[espFoC][driver_mock][iq31]")
{
    mock_isensor_t isen;
    mock_isensor_init(&isen);
    isen.values.iu_axis_0 = 0.25f;
    isen.values.iv_axis_0 = -0.25f;
    isen.values.iw_axis_0 = 0.0f;
    isen.values.iu_axis_1 = 0.1f;
    isen.values.iv_axis_1 = 0.1f;
    isen.values.iw_axis_1 = -0.2f;
    esp_foc_isensor_t *iface = mock_isensor_interface(&isen);

    isensor_values_iq31_t q;
    iface->fetch_isensors_iq31(iface, &q);
    TEST_ASSERT_EQUAL(1, isen.fetch_iq31_count);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.25f, iq31_to_float(q.iu_axis_0));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, -0.25f, iq31_to_float(q.iv_axis_0));
}

TEST_CASE("mock rotor: read_counts_iq31 and accumulated i64", "[espFoC][driver_mock][iq31]")
{
    mock_rotor_sensor_t rot;
    mock_rotor_sensor_init(&rot, 4096.0f);
    rot.counts = 2048.0f;
    rot.accumulated = 5000.4f;
    esp_foc_rotor_sensor_t *iface = mock_rotor_sensor_interface(&rot);

    iq31_t ang = iface->read_counts_iq31(iface);
    TEST_ASSERT_EQUAL(1, rot.read_counts_iq31_count);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.5f, iq31_to_float(ang));

    int64_t acc = iface->read_accumulated_counts_i64(iface);
    TEST_ASSERT_EQUAL(1, rot.read_accumulated_i64_count);
    TEST_ASSERT_EQUAL(5000LL, acc);
}

TEST_CASE("mock rotor: set_simulation_count_iq31 scales by counts_per_rev", "[espFoC][driver_mock][iq31]")
{
    mock_rotor_sensor_t rot;
    mock_rotor_sensor_init(&rot, 1000.0f);
    esp_foc_rotor_sensor_t *iface = mock_rotor_sensor_interface(&rot);

    /* normalized increment 0.25 turn = 250 ticks */
    iface->set_simulation_count_iq31(iface, iq31_from_float(0.25f));
    TEST_ASSERT_EQUAL(1, rot.set_simulation_count_iq31_count);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 250.0f, rot.counts);
}

#endif /* CONFIG_ESP_FOC_USE_FIXED_POINT */
