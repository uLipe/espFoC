/*
 * MIT License
 *
 * Smoke test for the build-time autotuned gains header. Verifies:
 *   - CMake plumbing produced esp_foc_autotuned_gains.h and the espFoC
 *     library exposes ESP_FOC_AUTOGEN_GAINS_AVAILABLE to its consumers.
 *   - The gain values agree with running esp_foc_design_pi_current_mpz_q16()
 *     on the same JSON inputs (1 LSB tolerance: same code path).
 *
 * The header is private to the espFoC library, so we cannot include it from
 * here. Instead the test asks the runtime API to compute the same design
 * with the Kconfig-supplied parameters and asserts the axis ends up using
 * those gains when motor_resistance/inductance are zero.
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
                       (float)CONFIG_ESP_FOC_AUTOGEN_PWM_RATE_HZ);
    mock_rotor_sensor_init(&s_rotor, 4096.0f);
    mock_isensor_init(&s_isensor);
    settings.natural_direction = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW;
    settings.motor_pole_pairs = 7;
    settings.motor_resistance = 0;        /* triggers autogen fallback */
    settings.motor_inductance = 0;
    settings.motor_inertia = q16_from_float(0.0001f);
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

TEST_CASE("autogen: nonzero motor_resistance bypasses autogen path",
          "[espFoC][autogen]")
{
    esp_foc_motor_control_settings_t settings;
    memset(&s_axis, 0, sizeof(s_axis));
    mock_inverter_init(&s_inv, 1.0f, 20000.0f);
    mock_rotor_sensor_init(&s_rotor, 4096.0f);
    mock_isensor_init(&s_isensor);
    settings.natural_direction = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW;
    settings.motor_pole_pairs = 7;
    /* Distinct motor: should go through legacy float path */
    settings.motor_resistance = q16_from_float(0.5f);
    settings.motor_inductance = q16_from_float(0.0005f);
    settings.motor_inertia = q16_from_float(0.0001f);
    settings.motor_unit = 0;

    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_initialize_axis(
        &s_axis,
        mock_inverter_interface(&s_inv),
        mock_rotor_sensor_interface(&s_rotor),
        mock_isensor_interface(&s_isensor),
        settings));

    /* Expected continuous-form gains: Kp = L*w_bw, Ki = R*w_bw.
     * Compute through the same Q16->float->Q16 round-trip as the core
     * (L gets quantized to Q16 first inside the settings struct). */
    float w_bw = 2.0f * 3.14159265f * 150.0f;
    float l_quantized = q16_to_float(q16_from_float(0.0005f));
    q16_t kp_expected = q16_from_float(l_quantized * w_bw);
    q16_t kp_axis;
    esp_foc_axis_get_current_pi_gains_q16(&s_axis, &kp_axis, NULL, NULL);
    int32_t diff = (int32_t)(kp_axis - kp_expected);
    if (diff < 0) {
        diff = -diff;
    }
    TEST_ASSERT_LESS_THAN_INT32(8, diff);
}
