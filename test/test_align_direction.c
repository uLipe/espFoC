/*
 * MIT License
 *
 * Tests for the natural-direction probe in esp_foc_align_axis.
 */

#include <string.h>
#include <unity.h>
#include "espFoC/esp_foc.h"
#include "espFoC/calibration/esp_foc_calibration.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "mock_drivers.h"

static esp_foc_axis_t s_axis;
static mock_inverter_t s_inv;
static mock_encoder_t s_rotor;

static void setup(esp_foc_motor_direction_t hint)
{
    esp_foc_motor_control_settings_t s = {0};
#if defined(CONFIG_ESP_FOC_CALIBRATION_NVS)
    (void)esp_foc_calibration_erase();
#endif
    memset(&s_axis, 0, sizeof(s_axis));
    mock_inverter_init(&s_inv, 1.0f, 20000.0f);
    mock_encoder_init(&s_rotor, 4096.0f);
    s.natural_direction = hint;
    s.motor_pole_pairs = 7;
    s.motor_unit = 0;
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_initialize_axis(
        &s_axis,
        mock_inverter_interface(&s_inv),
        mock_encoder_interface(&s_rotor),
        s));
}

TEST_CASE("alignment: positive deflection sets natural_direction = CW",
          "[espFoC][align]")
{
    setup(ESP_FOC_MOTOR_NATURAL_DIRECTION_CCW);
    float script[2] = {0.0f, 250.0f};
    mock_encoder_script_counts(&s_rotor, script, 2);
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_align_axis(&s_axis));
    TEST_ASSERT_EQUAL_INT32(Q16_ONE, s_axis.natural_direction);
}

TEST_CASE("alignment: negative deflection sets natural_direction = CCW",
          "[espFoC][align]")
{
    setup(ESP_FOC_MOTOR_NATURAL_DIRECTION_CW);
    float script[2] = {0.0f, -250.0f};
    mock_encoder_script_counts(&s_rotor, script, 2);
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_align_axis(&s_axis));
    TEST_ASSERT_EQUAL_INT32(Q16_MINUS_ONE, s_axis.natural_direction);
}

TEST_CASE("alignment: stuck rotor keeps the settings hint",
          "[espFoC][align]")
{
    setup(ESP_FOC_MOTOR_NATURAL_DIRECTION_CCW);
    float script[2] = {0.0f, 5.0f};
    mock_encoder_script_counts(&s_rotor, script, 2);
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_align_axis(&s_axis));
    TEST_ASSERT_EQUAL_INT32(Q16_MINUS_ONE, s_axis.natural_direction);
}

TEST_CASE("alignment: rejects double-call without re-init",
          "[espFoC][align]")
{
    setup(ESP_FOC_MOTOR_NATURAL_DIRECTION_CW);
    float script[2] = {0.0f, 100.0f};
    mock_encoder_script_counts(&s_rotor, script, 2);
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_align_axis(&s_axis));
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_AXIS_INVALID_STATE,
                      esp_foc_align_axis(&s_axis));
}
