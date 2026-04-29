/*
 * MIT License
 *
 * Smoke test for the build-time autotuned gains header. With the legacy
 * float fallback gone in 3.0, every axis init must end up with the
 * autogen Kp/Ki/integrator_limit baked into the controllers.
 */

#include <string.h>
#include <unity.h>
#include "espFoC/esp_foc.h"
#include "espFoC/esp_foc_axis_tuning.h"
#include "espFoC/utils/esp_foc_design_mpz.h"
#include "espFoC/esp_foc_controls.h"
#include "mock_drivers.h"

static esp_foc_axis_t s_axis;
static mock_inverter_t s_inv;
static mock_rotor_sensor_t s_rotor;
static mock_isensor_t s_isensor;

TEST_CASE("autogen: axis init with zero motor params uses autogen gains",
          "[espFoC][autogen]")
{
#if !defined(CONFIG_ESP_FOC_USE_AUTOGEN_GAINS)
    TEST_IGNORE_MESSAGE("autogen gains disabled at Kconfig level");
#else
    esp_foc_motor_control_settings_t settings;
    memset(&s_axis, 0, sizeof(s_axis));
    /* Use the same PWM rate the autotuner was configured with so the
     * runtime loop_ts_us matches the build-time one. */
    mock_inverter_init(&s_inv, 1.0f,
                       (float)CONFIG_ESP_FOC_PWM_RATE_HZ);
    mock_rotor_sensor_init(&s_rotor, 4096.0f);
    mock_isensor_init(&s_isensor);
    settings.natural_direction = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW;
    settings.motor_pole_pairs = 7;
    settings.motor_unit = 0;

    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_initialize_axis(
        &s_axis,
        mock_inverter_interface(&s_inv),
        mock_rotor_sensor_interface(&s_rotor),
        mock_isensor_interface(&s_isensor),
        settings));

    q16_t kp_axis, ki_axis, lim_axis;
    esp_foc_axis_get_current_pi_gains_q16(&s_axis, &kp_axis, &ki_axis, &lim_axis);
    TEST_ASSERT_NOT_EQUAL(0, kp_axis);
    TEST_ASSERT_NOT_EQUAL(0, ki_axis);
    TEST_ASSERT_NOT_EQUAL(0, lim_axis);

    /* Both controllers must hold the same gains. */
    TEST_ASSERT_EQUAL_INT32(s_axis.torque_controller[0].kp,
                            s_axis.torque_controller[1].kp);
    TEST_ASSERT_EQUAL_INT32(s_axis.torque_controller[0].ki,
                            s_axis.torque_controller[1].ki);
#endif
}

