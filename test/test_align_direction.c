/*
 * MIT License
 *
 * Tests for the natural-direction probe in esp_foc_align_axis.
 *
 * Each scenario primes the mock rotor with a scripted accumulated-tick
 * sequence: the alignment routine reads "before" (after re-zero) and
 * "after" (post Vq probe), and the delta decides the natural direction.
 */

#include <string.h>
#include <unity.h>
#include "espFoC/esp_foc.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "mock_drivers.h"

static esp_foc_axis_t s_axis;
static mock_inverter_t s_inv;
static mock_rotor_sensor_t s_rotor;
static mock_isensor_t s_isensor;

static void setup(esp_foc_motor_direction_t hint)
{
    esp_foc_motor_control_settings_t s = {0};
    memset(&s_axis, 0, sizeof(s_axis));
    mock_inverter_init(&s_inv, 1.0f, 20000.0f);
    mock_rotor_sensor_init(&s_rotor, 4096.0f);
    mock_isensor_init(&s_isensor);
    s.natural_direction = hint;
    s.motor_pole_pairs = 7;
    s.motor_unit = 0;
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_initialize_axis(
        &s_axis,
        mock_inverter_interface(&s_inv),
        mock_rotor_sensor_interface(&s_rotor),
        mock_isensor_interface(&s_isensor),
        s));
}

TEST_CASE("alignment: positive deflection sets natural_direction = CW",
          "[espFoC][align]")
{
    /* Hint says CCW so any failure to detect would leave the wrong
     * direction in place — makes the test sensitive. */
    setup(ESP_FOC_MOTOR_NATURAL_DIRECTION_CCW);
    /* Two reads: ticks_zero (after set_to_zero), ticks_after (post probe).
     * +250 counts >> ESP_FOC_DIR_PROBE_MIN_COUNTS (50) so CW wins. */
    int64_t script[2] = {0, 250};
    mock_rotor_sensor_script_accumulated(&s_rotor, script, 2);
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_align_axis(&s_axis));
    TEST_ASSERT_EQUAL_INT32(Q16_ONE, s_axis.natural_direction);
}

TEST_CASE("alignment: negative deflection sets natural_direction = CCW",
          "[espFoC][align]")
{
    setup(ESP_FOC_MOTOR_NATURAL_DIRECTION_CW);
    int64_t script[2] = {0, -250};
    mock_rotor_sensor_script_accumulated(&s_rotor, script, 2);
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_align_axis(&s_axis));
    TEST_ASSERT_EQUAL_INT32(Q16_MINUS_ONE, s_axis.natural_direction);
}

TEST_CASE("alignment: stuck rotor keeps the settings hint",
          "[espFoC][align]")
{
    setup(ESP_FOC_MOTOR_NATURAL_DIRECTION_CCW);
    /* 5 counts << 50 threshold — inconclusive, hint must survive. */
    int64_t script[2] = {0, 5};
    mock_rotor_sensor_script_accumulated(&s_rotor, script, 2);
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_align_axis(&s_axis));
    TEST_ASSERT_EQUAL_INT32(Q16_MINUS_ONE, s_axis.natural_direction);
}

TEST_CASE("alignment: rejects double-call without re-init",
          "[espFoC][align]")
{
    setup(ESP_FOC_MOTOR_NATURAL_DIRECTION_CW);
    int64_t script[2] = {0, 100};
    mock_rotor_sensor_script_accumulated(&s_rotor, script, 2);
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_align_axis(&s_axis));
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_AXIS_INVALID_STATE,
                      esp_foc_align_axis(&s_axis));
}
