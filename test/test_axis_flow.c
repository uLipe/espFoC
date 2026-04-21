/*
 * Unit tests for espFoC axis flow with mock drivers (IQ31 pipeline).
 */
#include <string.h>
#include <unity.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "espFoC/esp_foc.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "mock_drivers.h"

#define FLOAT_TOL 1e-4f

static esp_foc_axis_t axis;
static mock_inverter_t mock_inv;
static mock_rotor_sensor_t mock_rotor;
static esp_foc_motor_control_settings_t settings;

static void setup_sensored_mocks(void)
{
    memset(&axis, 0, sizeof(axis));
    mock_inverter_init(&mock_inv, 1.0f, 20000.0f);
    mock_rotor_sensor_init(&mock_rotor, 4096.0f);
    mock_rotor.counts = 0.0f;
    settings.natural_direction = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW;
    settings.motor_pole_pairs = 7;
    settings.motor_unit = 0;
}

TEST_CASE("axis init: inverter and rotor mocks called", "[espFoC][axis_flow]")
{
    setup_sensored_mocks();

    esp_foc_err_t err = esp_foc_initialize_axis(
        &axis,
        mock_inverter_interface(&mock_inv),
        mock_rotor_sensor_interface(&mock_rotor),
        NULL,
        settings);

    TEST_ASSERT_EQUAL(ESP_FOC_OK, err);
    TEST_ASSERT_TRUE(mock_inv.set_voltages_count >= 1);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.0f, q16_to_float(mock_inv.last_v_u));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.0f, q16_to_float(mock_inv.last_v_v));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.0f, q16_to_float(mock_inv.last_v_w));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 1.0f, q16_to_float(axis.dc_link_voltage));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 20000.0f, q16_to_float(axis.inv_dt));
}

TEST_CASE("axis align: enable and set_voltages sequence", "[espFoC][axis_flow]")
{
    setup_sensored_mocks();
    esp_foc_initialize_axis(&axis, mock_inverter_interface(&mock_inv),
                            mock_rotor_sensor_interface(&mock_rotor), NULL, settings);

    int set_voltages_before = mock_inv.set_voltages_count;
    int enable_before = mock_inv.enable_count;
    int set_to_zero_before = mock_rotor.set_to_zero_count;

    esp_foc_err_t err = esp_foc_align_axis(&axis);

    TEST_ASSERT_EQUAL(ESP_FOC_OK, err);
    TEST_ASSERT_EQUAL(ESP_FOC_OK, axis.rotor_aligned);
    TEST_ASSERT_EQUAL(enable_before + 1, mock_inv.enable_count);
    TEST_ASSERT_TRUE(mock_inv.set_voltages_count > set_voltages_before);
    /* New alignment zeroes the encoder twice: once after the initial
     * parking and once after the natural-direction probe. */
    TEST_ASSERT_EQUAL(set_to_zero_before + 2, mock_rotor.set_to_zero_count);
}

TEST_CASE("axis init: invalid args return error", "[espFoC][axis_flow]")
{
    setup_sensored_mocks();

    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG,
                      esp_foc_initialize_axis(NULL, mock_inverter_interface(&mock_inv),
                                              mock_rotor_sensor_interface(&mock_rotor), NULL, settings));
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG,
                      esp_foc_initialize_axis(&axis, NULL,
                                              mock_rotor_sensor_interface(&mock_rotor), NULL, settings));
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG,
                      esp_foc_initialize_axis(&axis, mock_inverter_interface(&mock_inv), NULL, NULL, settings));
}

TEST_CASE("axis align: already aligned returns error", "[espFoC][axis_flow]")
{
    setup_sensored_mocks();
    esp_foc_initialize_axis(&axis, mock_inverter_interface(&mock_inv),
                            mock_rotor_sensor_interface(&mock_rotor), NULL, settings);
    esp_foc_align_axis(&axis);

    esp_foc_err_t err = esp_foc_align_axis(&axis);

    TEST_ASSERT_EQUAL(ESP_FOC_ERR_AXIS_INVALID_STATE, err);
}

static int axis_test_regulator_called;
static void axis_test_regulator_cb(esp_foc_axis_t *ax,
                                   esp_foc_d_current_q16_t *id_ref,
                                   esp_foc_q_current_q16_t *iq_ref,
                                   esp_foc_d_voltage_q16_t *ud_ff,
                                   esp_foc_q_voltage_q16_t *uq_ff)
{
    (void)ax;
    (void)id_ref;
    (void)iq_ref;
    (void)ud_ff;
    (void)uq_ff;
    axis_test_regulator_called = 1;
}

TEST_CASE("axis set_regulation_callback: stored and invalid args", "[espFoC][axis_flow]")
{
    setup_sensored_mocks();
    esp_foc_initialize_axis(&axis, mock_inverter_interface(&mock_inv),
                            mock_rotor_sensor_interface(&mock_rotor), NULL, settings);
    esp_foc_align_axis(&axis);

    axis_test_regulator_called = 0;

    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG, esp_foc_set_regulation_callback(NULL, axis_test_regulator_cb));
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG, esp_foc_set_regulation_callback(&axis, NULL));

    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_set_regulation_callback(&axis, axis_test_regulator_cb));
    TEST_ASSERT_NOT_NULL(axis.regulator_cb);
    axis.regulator_cb(&axis, &axis.target_i_d, &axis.target_i_q, &axis.target_u_d, &axis.target_u_q);
    TEST_ASSERT_EQUAL(1, axis_test_regulator_called);
}

static int axis_run_regulator_called_count;
static void axis_run_regulator_cb(esp_foc_axis_t *ax,
                                esp_foc_d_current_q16_t *id_ref,
                                esp_foc_q_current_q16_t *iq_ref,
                                esp_foc_d_voltage_q16_t *ud_ff,
                                esp_foc_q_voltage_q16_t *uq_ff)
{
    (void)ax;
    axis_run_regulator_called_count++;
    id_ref->raw = 0;
    iq_ref->raw = q16_from_float(0.5f);
    ud_ff->raw = 0;
    uq_ff->raw = 0;
}

TEST_CASE("axis run: regulator runs and set_voltages receives FOC output", "[espFoC][axis_flow]")
{
    setup_sensored_mocks();
    esp_foc_initialize_axis(&axis, mock_inverter_interface(&mock_inv),
                            mock_rotor_sensor_interface(&mock_rotor), NULL, settings);
    esp_foc_align_axis(&axis);
    axis_run_regulator_called_count = 0;
    esp_foc_set_regulation_callback(&axis, axis_run_regulator_cb);

    esp_foc_err_t err = esp_foc_run(&axis);
    TEST_ASSERT_EQUAL(ESP_FOC_OK, err);

    for (int i = 0; i < 40; i++) {
        mock_inverter_trigger_callback(&mock_inv);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(200));

    TEST_ASSERT_TRUE(mock_inv.set_voltages_count >= 1);
    TEST_ASSERT_TRUE(axis_run_regulator_called_count >= 1);
}
