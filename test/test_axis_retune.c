/*
 * MIT License
 *
 * Unit tests for the runtime axis tuning API (esp_foc_axis_tuning.h).
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

static void setup_axis(float dc_link, float pwm_rate)
{
    esp_foc_motor_control_settings_t settings;
    memset(&s_axis, 0, sizeof(s_axis));
    mock_inverter_init(&s_inv, dc_link, pwm_rate);
    mock_rotor_sensor_init(&s_rotor, 4096.0f);
    mock_isensor_init(&s_isensor);
    settings.natural_direction = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW;
    settings.motor_pole_pairs = 7;
    settings.motor_unit = 0;

    esp_foc_err_t err = esp_foc_initialize_axis(
        &s_axis,
        mock_inverter_interface(&s_inv),
        mock_rotor_sensor_interface(&s_rotor),
        mock_isensor_interface(&s_isensor),
        settings);
    TEST_ASSERT_EQUAL(ESP_FOC_OK, err);
}

/* --- 2.1 set/get round-trip -------------------------------------------- */

TEST_CASE("axis_tuning: set_gains then get_gains returns same values",
          "[espFoC][axis_tuning]")
{
    setup_axis(1.0f, 20000.0f);
    q16_t kp_set = q16_from_float(2.5f);
    q16_t ki_set = q16_from_float(123.4f);
    q16_t lim_set = q16_from_float(7.0f);

    TEST_ASSERT_EQUAL(ESP_FOC_OK,
        esp_foc_axis_set_current_pi_gains_q16(&s_axis, kp_set, ki_set, lim_set));

    q16_t kp_got, ki_got, lim_got;
    esp_foc_axis_get_current_pi_gains_q16(&s_axis, &kp_got, &ki_got, &lim_got);
    TEST_ASSERT_EQUAL_INT32(kp_set, kp_got);
    TEST_ASSERT_EQUAL_INT32(ki_set, ki_got);
    TEST_ASSERT_EQUAL_INT32(lim_set, lim_got);

    /* Both controllers must hold the same values (D and Q legs share design). */
    TEST_ASSERT_EQUAL_INT32(kp_set, s_axis.torque_controller[1].kp);
    TEST_ASSERT_EQUAL_INT32(ki_set, s_axis.torque_controller[1].ki);
    TEST_ASSERT_EQUAL_INT32(lim_set, s_axis.torque_controller[1].integrator_limit);
}

/* --- 2.2 retune == direct design call ---------------------------------- */

TEST_CASE("axis_tuning: retune output matches direct MPZ design",
          "[espFoC][axis_tuning]")
{
    setup_axis(1.0f, 20000.0f);

    q16_t r = q16_from_float(1.08f);
    q16_t l = q16_from_float(0.0018f);
    q16_t bw = q16_from_float(150.0f);

    TEST_ASSERT_EQUAL(ESP_FOC_OK,
        esp_foc_axis_retune_current_pi_q16(&s_axis, r, l, bw));

    /* Recompute via the design helper. retune now reads the loop dt
     * straight from the axis PID so the test mirrors that exact
     * derivation — under ISR_HOT_PATH the effective decimation is 1
     * so deriving it from the static DOWNSAMPLING constant would
     * disagree with what the axis actually carries. */
    q16_t loop_dt_q16 = s_axis.torque_controller[0].dt;
    uint32_t loop_ts_us = (uint32_t)(((int64_t)loop_dt_q16 * 1000000LL
                                      + (int64_t)Q16_ONE / 2)
                                     / (int64_t)Q16_ONE);
    esp_foc_pi_design_input_t in = {
        .motor_r_ohm = r,
        .motor_l_h   = l,
        .loop_ts_us  = loop_ts_us,
        .bandwidth_hz = bw,
        .v_max       = s_axis.max_voltage,
    };
    esp_foc_pi_design_output_t expected;
    TEST_ASSERT_EQUAL_INT(ESP_FOC_DESIGN_OK,
        esp_foc_design_pi_current_mpz_q16(&in, &expected));

    q16_t kp_got, ki_got, lim_got;
    esp_foc_axis_get_current_pi_gains_q16(&s_axis, &kp_got, &ki_got, &lim_got);
    TEST_ASSERT_EQUAL_INT32(expected.kp, kp_got);
    TEST_ASSERT_EQUAL_INT32(expected.ki, ki_got);
    TEST_ASSERT_EQUAL_INT32(expected.integrator_limit, lim_got);
}

/* --- 2.3 retune resets integrator state -------------------------------- */

TEST_CASE("axis_tuning: retune resets integrator and prev_error",
          "[espFoC][axis_tuning]")
{
    setup_axis(1.0f, 20000.0f);

    /* Pollute integrator state. */
    s_axis.torque_controller[0].integrator = 0x1234567890LL;
    s_axis.torque_controller[0].prev_error = q16_from_float(0.5f);
    s_axis.torque_controller[1].integrator = -0x1234567890LL;
    s_axis.torque_controller[1].prev_error = q16_from_float(-0.5f);

    TEST_ASSERT_EQUAL(ESP_FOC_OK,
        esp_foc_axis_retune_current_pi_q16(&s_axis,
                                           q16_from_float(1.0f),
                                           q16_from_float(0.001f),
                                           q16_from_float(200.0f)));

    TEST_ASSERT_TRUE(s_axis.torque_controller[0].integrator == 0);
    TEST_ASSERT_EQUAL_INT32(0, s_axis.torque_controller[0].prev_error);
    TEST_ASSERT_TRUE(s_axis.torque_controller[1].integrator == 0);
    TEST_ASSERT_EQUAL_INT32(0, s_axis.torque_controller[1].prev_error);
}

/* --- 2.4 invalid args rejected ----------------------------------------- */

TEST_CASE("axis_tuning: rejects null / non-positive args",
          "[espFoC][axis_tuning]")
{
    setup_axis(1.0f, 20000.0f);

    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG,
        esp_foc_axis_retune_current_pi_q16(NULL, 1, 1, 1));

    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG,
        esp_foc_axis_retune_current_pi_q16(&s_axis, 0,
                                           q16_from_float(0.001f),
                                           q16_from_float(150.0f)));
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG,
        esp_foc_axis_retune_current_pi_q16(&s_axis, q16_from_float(1.0f), 0,
                                           q16_from_float(150.0f)));
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG,
        esp_foc_axis_retune_current_pi_q16(&s_axis,
                                           q16_from_float(1.0f),
                                           q16_from_float(0.001f), 0));

    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG,
        esp_foc_axis_set_current_pi_gains_q16(NULL, 1, 1, 1));
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG,
        esp_foc_axis_set_current_pi_gains_q16(&s_axis, -1, 1, 1));
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG,
        esp_foc_axis_set_current_pi_gains_q16(&s_axis, 1, -1, 1));
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG,
        esp_foc_axis_set_current_pi_gains_q16(&s_axis, 1, 1, -1));
}

/* --- 2.5 get with NULL outputs is safe --------------------------------- */

TEST_CASE("axis_tuning: get_gains accepts NULL output pointers",
          "[espFoC][axis_tuning]")
{
    setup_axis(1.0f, 20000.0f);
    q16_t kp_only;
    esp_foc_axis_get_current_pi_gains_q16(&s_axis, &kp_only, NULL, NULL);
    esp_foc_axis_get_current_pi_gains_q16(&s_axis, NULL, NULL, NULL);
    /* Test passes if it didn't crash; sanity-check kp is the value initialized
     * by esp_foc_initialize_axis (non-zero for the default settings). */
    TEST_ASSERT_NOT_EQUAL(0, kp_only);
}

/* --- 2.6 set/retune sequence: last write wins -------------------------- */

TEST_CASE("axis_tuning: last write wins between set and retune",
          "[espFoC][axis_tuning]")
{
    setup_axis(1.0f, 20000.0f);

    q16_t kp_manual = q16_from_float(9.5f);
    q16_t ki_manual = q16_from_float(900.0f);
    q16_t lim_manual = q16_from_float(11.0f);
    TEST_ASSERT_EQUAL(ESP_FOC_OK,
        esp_foc_axis_set_current_pi_gains_q16(&s_axis,
                                              kp_manual, ki_manual, lim_manual));
    q16_t kp_got;
    esp_foc_axis_get_current_pi_gains_q16(&s_axis, &kp_got, NULL, NULL);
    TEST_ASSERT_EQUAL_INT32(kp_manual, kp_got);

    TEST_ASSERT_EQUAL(ESP_FOC_OK,
        esp_foc_axis_retune_current_pi_q16(&s_axis,
                                           q16_from_float(1.0f),
                                           q16_from_float(0.001f),
                                           q16_from_float(150.0f)));
    esp_foc_axis_get_current_pi_gains_q16(&s_axis, &kp_got, NULL, NULL);
    TEST_ASSERT_NOT_EQUAL(kp_manual, kp_got);
}
