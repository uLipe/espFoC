/*
 * MIT License
 *
 * Unit tests for the runtime tuner backend (esp_foc_tuner.h). Exercises the
 * request handler directly (no transport) to validate the wire protocol and
 * that read/write/exec map correctly to the underlying axis tuning API.
 */

#include <string.h>
#include <unity.h>
#include "espFoC/esp_foc.h"
#include "espFoC/esp_foc_tuner.h"
#include "espFoC/esp_foc_axis_tuning.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "mock_drivers.h"

#if defined(CONFIG_ESP_FOC_TUNER_ENABLE)

static esp_foc_axis_t s_axis;
static mock_inverter_t s_inv;
static mock_rotor_sensor_t s_rotor;
static mock_isensor_t s_isensor;

static void setup_attached_axis(void)
{
    esp_foc_motor_control_settings_t settings;
    memset(&s_axis, 0, sizeof(s_axis));
    mock_inverter_init(&s_inv, 1.0f, 20000.0f);
    mock_rotor_sensor_init(&s_rotor, 4096.0f);
    mock_isensor_init(&s_isensor);
    settings.natural_direction = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW;
    settings.motor_pole_pairs = 7;
    settings.motor_resistance = q16_from_float(1.0f);
    settings.motor_inductance = q16_from_float(0.001f);
    settings.motor_inertia = q16_from_float(0.0001f);
    settings.motor_unit = 0;

    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_initialize_axis(
        &s_axis,
        mock_inverter_interface(&s_inv),
        mock_rotor_sensor_interface(&s_rotor),
        mock_isensor_interface(&s_isensor),
        settings));
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_tuner_attach_axis(0, &s_axis));
}

static void serialize_q16_le(uint8_t *dst, q16_t v)
{
    uint32_t u = (uint32_t)v;
    dst[0] = (uint8_t)(u & 0xFF);
    dst[1] = (uint8_t)((u >> 8) & 0xFF);
    dst[2] = (uint8_t)((u >> 16) & 0xFF);
    dst[3] = (uint8_t)((u >> 24) & 0xFF);
}

static q16_t deserialize_q16_le(const uint8_t *src)
{
    uint32_t u = (uint32_t)src[0] |
                 ((uint32_t)src[1] << 8) |
                 ((uint32_t)src[2] << 16) |
                 ((uint32_t)src[3] << 24);
    return (q16_t)u;
}

TEST_CASE("tuner: rejects unknown axis id", "[espFoC][tuner]")
{
    setup_attached_axis();
    uint8_t resp[4];
    size_t resp_len = sizeof(resp);
    esp_foc_err_t err = esp_foc_tuner_handle_request(
        9, ESP_FOC_TUNER_OP_READ, ESP_FOC_TUNER_PARAM_KP_Q16,
        NULL, 0, resp, &resp_len);
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG, err);
}

TEST_CASE("tuner: read kp/ki/lim returns axis state", "[espFoC][tuner]")
{
    setup_attached_axis();

    uint8_t resp[4];
    size_t resp_len;

    resp_len = sizeof(resp);
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_tuner_handle_request(
        0, ESP_FOC_TUNER_OP_READ, ESP_FOC_TUNER_PARAM_KP_Q16,
        NULL, 0, resp, &resp_len));
    TEST_ASSERT_EQUAL(4, resp_len);
    TEST_ASSERT_EQUAL_INT32(s_axis.torque_controller[0].kp,
                            deserialize_q16_le(resp));

    resp_len = sizeof(resp);
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_tuner_handle_request(
        0, ESP_FOC_TUNER_OP_READ, ESP_FOC_TUNER_PARAM_KI_Q16,
        NULL, 0, resp, &resp_len));
    TEST_ASSERT_EQUAL_INT32(s_axis.torque_controller[0].ki,
                            deserialize_q16_le(resp));

    resp_len = sizeof(resp);
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_tuner_handle_request(
        0, ESP_FOC_TUNER_OP_READ, ESP_FOC_TUNER_PARAM_INT_LIM_Q16,
        NULL, 0, resp, &resp_len));
    TEST_ASSERT_EQUAL_INT32(s_axis.torque_controller[0].integrator_limit,
                            deserialize_q16_le(resp));
}

TEST_CASE("tuner: write kp updates axis atomically", "[espFoC][tuner]")
{
    setup_attached_axis();

    q16_t target_kp = q16_from_float(7.5f);
    uint8_t payload[4];
    serialize_q16_le(payload, target_kp);
    size_t resp_len = 0;

    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_tuner_handle_request(
        0, ESP_FOC_TUNER_OP_WRITE, ESP_FOC_TUNER_WRITE_KP_Q16,
        payload, sizeof(payload), NULL, &resp_len));

    q16_t kp_got;
    esp_foc_axis_get_current_pi_gains_q16(&s_axis, &kp_got, NULL, NULL);
    TEST_ASSERT_EQUAL_INT32(target_kp, kp_got);
}

TEST_CASE("tuner: exec recompute matches direct retune call",
          "[espFoC][tuner]")
{
    setup_attached_axis();

    q16_t r  = q16_from_float(1.08f);
    q16_t l  = q16_from_float(0.0018f);
    q16_t bw = q16_from_float(150.0f);

    uint8_t payload[12];
    serialize_q16_le(payload + 0, r);
    serialize_q16_le(payload + 4, l);
    serialize_q16_le(payload + 8, bw);
    size_t resp_len = 0;

    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_tuner_handle_request(
        0, ESP_FOC_TUNER_OP_EXEC, ESP_FOC_TUNER_CMD_RECOMPUTE_GAINS,
        payload, sizeof(payload), NULL, &resp_len));

    q16_t kp_via_tuner;
    esp_foc_axis_get_current_pi_gains_q16(&s_axis, &kp_via_tuner, NULL, NULL);

    /* Reset axis and call direct API for comparison. */
    setup_attached_axis();
    TEST_ASSERT_EQUAL(ESP_FOC_OK,
        esp_foc_axis_retune_current_pi_q16(&s_axis, r, l, bw));
    q16_t kp_direct;
    esp_foc_axis_get_current_pi_gains_q16(&s_axis, &kp_direct, NULL, NULL);

    TEST_ASSERT_EQUAL_INT32(kp_direct, kp_via_tuner);
}

TEST_CASE("tuner: exec with short payload is rejected", "[espFoC][tuner]")
{
    setup_attached_axis();
    uint8_t payload[8] = {0};  /* RECOMPUTE needs 12 bytes */
    size_t resp_len = 0;
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG, esp_foc_tuner_handle_request(
        0, ESP_FOC_TUNER_OP_EXEC, ESP_FOC_TUNER_CMD_RECOMPUTE_GAINS,
        payload, sizeof(payload), NULL, &resp_len));
}

#else

/* Tuner disabled: keep this TU non-empty so the test build stays consistent. */
TEST_CASE("tuner: disabled at Kconfig", "[espFoC][tuner]")
{
    TEST_IGNORE_MESSAGE("CONFIG_ESP_FOC_TUNER_ENABLE not set");
}

#endif
