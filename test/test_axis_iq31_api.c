/*
 * Unit tests for dedicated IQ31 public API.
 */
#include <string.h>
#include <unity.h>
#include <sdkconfig.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "espFoC/esp_foc.h"
#include "espFoC/esp_foc_iq31_api.h"
#include "espFoC/utils/esp_foc_iq31.h"
#include "mock_drivers.h"

#if CONFIG_ESP_FOC_USE_FIXED_POINT

static esp_foc_axis_iq31_t axis_q31;
static mock_inverter_t mock_inv;
static mock_rotor_sensor_t mock_rotor;
static mock_isensor_t mock_isensor;
static esp_foc_motor_control_settings_iq31_t settings_q31;

static void setup_sensored_iq31_mocks(void)
{
    memset(&axis_q31, 0, sizeof(axis_q31));
    mock_inverter_init(&mock_inv, 24.0f, 20000.0f);
    mock_rotor_sensor_init(&mock_rotor, 4096.0f);
    mock_isensor_init(&mock_isensor);
    settings_q31.natural_direction = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW;
    settings_q31.motor_pole_pairs = 7;
    settings_q31.motor_resistance = iq31_from_float(1.0f);
    settings_q31.motor_inductance = iq31_from_float(0.001f);
    settings_q31.motor_inertia = iq31_from_float(0.0001f);
    settings_q31.motor_unit = 0;
}

TEST_CASE("axis_iq31 api: init valid sensored config", "[espFoC][iq31_api]")
{
    setup_sensored_iq31_mocks();
    esp_foc_err_t err = esp_foc_initialize_axis_iq31(
        &axis_q31,
        mock_inverter_interface(&mock_inv),
        mock_rotor_sensor_interface(&mock_rotor),
        mock_isensor_interface(&mock_isensor),
        settings_q31);

    TEST_ASSERT_EQUAL(ESP_FOC_OK, err);
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_NOT_ALIGNED, axis_q31.rotor_aligned);
    TEST_ASSERT_NOT_EQUAL(0, mock_inv.set_voltages_iq31_count);
    TEST_ASSERT_NOT_EQUAL(0, axis_q31.max_voltage);
    TEST_ASSERT_NOT_EQUAL(0, axis_q31.dc_link_to_normalized);
}

TEST_CASE("axis_iq31 api: invalid args are rejected", "[espFoC][iq31_api]")
{
    setup_sensored_iq31_mocks();

    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG,
                      esp_foc_initialize_axis_iq31(NULL,
                                                   mock_inverter_interface(&mock_inv),
                                                   mock_rotor_sensor_interface(&mock_rotor),
                                                   mock_isensor_interface(&mock_isensor),
                                                   settings_q31));
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG,
                      esp_foc_initialize_axis_iq31(&axis_q31,
                                                   NULL,
                                                   mock_rotor_sensor_interface(&mock_rotor),
                                                   mock_isensor_interface(&mock_isensor),
                                                   settings_q31));
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG,
                      esp_foc_initialize_axis_iq31(&axis_q31,
                                                   mock_inverter_interface(&mock_inv),
                                                   NULL,
                                                   mock_isensor_interface(&mock_isensor),
                                                   settings_q31));
}

TEST_CASE("axis_iq31 api: align and run happy path", "[espFoC][iq31_api]")
{
    setup_sensored_iq31_mocks();
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_initialize_axis_iq31(
                                      &axis_q31,
                                      mock_inverter_interface(&mock_inv),
                                      mock_rotor_sensor_interface(&mock_rotor),
                                      mock_isensor_interface(&mock_isensor),
                                      settings_q31));

    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_align_axis_iq31(&axis_q31));
    TEST_ASSERT_EQUAL(ESP_FOC_OK, axis_q31.rotor_aligned);

    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_run_iq31(&axis_q31));
    for (int i = 0; i < 30; i++) {
        mock_inverter_trigger_callback(&mock_inv);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    TEST_ASSERT_TRUE(mock_inv.set_voltages_iq31_count > 1);
    TEST_ASSERT_TRUE(mock_isensor.fetch_iq31_count > 0);
    TEST_ASSERT_TRUE(mock_rotor.read_counts_iq31_count > 0);
}

static int iq31_reg_cb_called;
static void test_reg_cb_iq31(esp_foc_axis_iq31_t *axis,
                             esp_foc_d_current_iq31_t *id_ref,
                             esp_foc_q_current_iq31_t *iq_ref,
                             esp_foc_d_voltage_iq31_t *ud_ff,
                             esp_foc_q_voltage_iq31_t *uq_ff)
{
    (void)axis;
    iq31_reg_cb_called++;
    id_ref->raw = 0;
    iq_ref->raw = iq31_from_float(0.3f);
    ud_ff->raw = 0;
    uq_ff->raw = 0;
}

TEST_CASE("axis_iq31 api: set callback validates args and stores", "[espFoC][iq31_api]")
{
    setup_sensored_iq31_mocks();
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_initialize_axis_iq31(
                                      &axis_q31,
                                      mock_inverter_interface(&mock_inv),
                                      mock_rotor_sensor_interface(&mock_rotor),
                                      mock_isensor_interface(&mock_isensor),
                                      settings_q31));

    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG, esp_foc_set_regulation_callback_iq31(NULL, test_reg_cb_iq31));
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG, esp_foc_set_regulation_callback_iq31(&axis_q31, NULL));
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_set_regulation_callback_iq31(&axis_q31, test_reg_cb_iq31));
    iq31_reg_cb_called = 0;
    axis_q31.regulator_cb(&axis_q31, &axis_q31.target_i_d, &axis_q31.target_i_q, &axis_q31.target_u_d, &axis_q31.target_u_q);
    TEST_ASSERT_EQUAL(1, iq31_reg_cb_called);
}

TEST_CASE("axis_iq31 api: edge near saturation settings", "[espFoC][iq31_api]")
{
    setup_sensored_iq31_mocks();
    settings_q31.motor_resistance = iq31_from_float(0.99f);
    settings_q31.motor_inductance = iq31_from_float(0.99f);
    settings_q31.motor_inertia = iq31_from_float(0.99f);
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_initialize_axis_iq31(
                                      &axis_q31,
                                      mock_inverter_interface(&mock_inv),
                                      mock_rotor_sensor_interface(&mock_rotor),
                                      mock_isensor_interface(&mock_isensor),
                                      settings_q31));
    TEST_ASSERT_TRUE(axis_q31.max_voltage > 0);
    TEST_ASSERT_TRUE(axis_q31.dt > 0);
    TEST_ASSERT_TRUE(axis_q31.inv_dt > 0);
}

#endif

