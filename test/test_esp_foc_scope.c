/*
 * Unit tests for esp_foc_scope initialization.
 */
#include "sdkconfig.h"
#if CONFIG_ESP_FOC_SCOPE

#include <string.h>
#include <unity.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "espFoC/esp_foc.h"
#include "espFoC/stream/esp_foc_scope.h"
#include "mock_drivers.h"

TEST_CASE("scope: initialize starts daemon", "[espFoC][scope]")
{
    esp_foc_scope_initalize();
    TEST_ASSERT_TRUE(esp_foc_scope_is_initialized());
}

TEST_CASE("scope: initialize is idempotent", "[espFoC][scope]")
{
    esp_foc_scope_initalize();
    esp_foc_scope_initalize();
    TEST_ASSERT_TRUE(esp_foc_scope_is_initialized());
}

TEST_CASE("scope: axis init wires 17 channels", "[espFoC][scope]")
{
    esp_foc_axis_t axis;
    mock_inverter_t mock_inv;
    mock_rotor_sensor_t mock_rotor;
    esp_foc_motor_control_settings_t settings = {
        .natural_direction = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW,
        .motor_pole_pairs = 7,
        .motor_unit = 0,
    };

    memset(&axis, 0, sizeof(axis));
    mock_inverter_init(&mock_inv, 1.0f, 20000.0f);
    mock_rotor_sensor_init(&mock_rotor, 4096.0f);

    TEST_ASSERT_EQUAL(ESP_FOC_OK,
                      esp_foc_initialize_axis(&axis,
                                              mock_inverter_interface(&mock_inv),
                                              mock_rotor_sensor_interface(&mock_rotor),
                                              NULL,
                                              settings));
    TEST_ASSERT_TRUE(esp_foc_scope_is_initialized());
#if CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS == 17
    TEST_ASSERT_EQUAL(-1, esp_foc_scope_add_channel(&axis.target_i_d.raw, 0));
#endif
}

#endif /* CONFIG_ESP_FOC_SCOPE */
