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

/* --- magic / state query / motion / override --------------------------- */

TEST_CASE("tuner: attach refuses an axis with no magic",
          "[espFoC][tuner]")
{
    esp_foc_axis_t bogus;
    memset(&bogus, 0, sizeof(bogus));
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_AXIS_INVALID_STATE,
                      esp_foc_tuner_attach_axis(1, &bogus));
}

TEST_CASE("tuner: attach with NULL detaches without error",
          "[espFoC][tuner]")
{
    setup_attached_axis();
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_tuner_attach_axis(0, NULL));
    /* After detach all requests on this axis must be rejected. */
    uint8_t resp = 0;
    size_t resp_len = sizeof(resp);
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG, esp_foc_tuner_handle_request(
        0, ESP_FOC_TUNER_OP_READ, ESP_FOC_TUNER_PARAM_AXIS_STATE,
        NULL, 0, &resp, &resp_len));
}

TEST_CASE("tuner: read AXIS_STATE returns initialized + (un)aligned",
          "[espFoC][tuner]")
{
    setup_attached_axis();
    uint8_t resp = 0;
    size_t resp_len = sizeof(resp);
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_tuner_handle_request(
        0, ESP_FOC_TUNER_OP_READ, ESP_FOC_TUNER_PARAM_AXIS_STATE,
        NULL, 0, &resp, &resp_len));
    TEST_ASSERT_EQUAL_UINT(1, resp_len);
    /* Axis was initialized but not aligned/run nor under override yet. */
    TEST_ASSERT_TRUE(resp & ESP_FOC_AXIS_STATE_INITIALIZED);
    TEST_ASSERT_FALSE(resp & ESP_FOC_AXIS_STATE_ALIGNED);
    TEST_ASSERT_FALSE(resp & ESP_FOC_AXIS_STATE_RUNNING);
    TEST_ASSERT_FALSE(resp & ESP_FOC_AXIS_STATE_TUNER_OVERRIDE);
}

TEST_CASE("tuner: override ON refused on un-aligned axis",
          "[espFoC][tuner]")
{
    setup_attached_axis();
    /* Fresh axis is not aligned yet; override must be rejected to keep the
     * motor safe. */
    size_t resp_len = 0;
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_NOT_ALIGNED, esp_foc_tuner_handle_request(
        0, ESP_FOC_TUNER_OP_EXEC, ESP_FOC_TUNER_CMD_OVERRIDE_ON,
        NULL, 0, NULL, &resp_len));
}

TEST_CASE("tuner: override ON then OFF flips state bit",
          "[espFoC][tuner]")
{
    setup_attached_axis();
    /* Manually mark the axis aligned (bypass real alignment for this unit
     * test; align_axis needs hardware sequence). */
    s_axis.rotor_aligned = ESP_FOC_OK;

    size_t resp_len = 0;
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_tuner_handle_request(
        0, ESP_FOC_TUNER_OP_EXEC, ESP_FOC_TUNER_CMD_OVERRIDE_ON,
        NULL, 0, NULL, &resp_len));
    uint8_t state = 0;
    size_t rl = sizeof(state);
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_tuner_handle_request(
        0, ESP_FOC_TUNER_OP_READ, ESP_FOC_TUNER_PARAM_AXIS_STATE,
        NULL, 0, &state, &rl));
    TEST_ASSERT_TRUE(state & ESP_FOC_AXIS_STATE_TUNER_OVERRIDE);

    resp_len = 0;
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_tuner_handle_request(
        0, ESP_FOC_TUNER_OP_EXEC, ESP_FOC_TUNER_CMD_OVERRIDE_OFF,
        NULL, 0, NULL, &resp_len));
    rl = sizeof(state);
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_tuner_handle_request(
        0, ESP_FOC_TUNER_OP_READ, ESP_FOC_TUNER_PARAM_AXIS_STATE,
        NULL, 0, &state, &rl));
    TEST_ASSERT_FALSE(state & ESP_FOC_AXIS_STATE_TUNER_OVERRIDE);
}

TEST_CASE("tuner: motion target writes refused while override is OFF",
          "[espFoC][tuner]")
{
    setup_attached_axis();
    s_axis.rotor_aligned = ESP_FOC_OK;
    /* override is OFF (default). Writing a motion target must fail. */
    uint8_t payload[4];
    serialize_q16_le(payload, q16_from_float(2.5f));
    size_t resp_len = 0;
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_AXIS_INVALID_STATE,
        esp_foc_tuner_handle_request(0, ESP_FOC_TUNER_OP_WRITE,
            ESP_FOC_TUNER_WRITE_TARGET_IQ_Q16,
            payload, sizeof(payload), NULL, &resp_len));
}

TEST_CASE("tuner: read LOOP_FS_HZ returns the value init programmed",
          "[espFoC][tuner]")
{
    setup_attached_axis();
    /* esp_foc_initialize_axis stores the loop sample rate (PWM rate
     * under ISR_HOT_PATH or pwm_rate / decimation under the legacy
     * task path) on the axis. Tuner just reads it back. */
    uint8_t resp[4] = {0};
    size_t rl = sizeof(resp);
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_tuner_handle_request(
        0, ESP_FOC_TUNER_OP_READ, ESP_FOC_TUNER_PARAM_LOOP_FS_HZ_Q16,
        NULL, 0, resp, &rl));
    TEST_ASSERT_EQUAL(4, rl);
    q16_t fs = deserialize_q16_le(resp);
    TEST_ASSERT_EQUAL_INT32(s_axis.current_filter_fs_hz_q16, fs);
    TEST_ASSERT_TRUE(fs > 0);
}

TEST_CASE("tuner: read I_FILTER_FC returns the value init programmed",
          "[espFoC][tuner]")
{
    setup_attached_axis();
    /* esp_foc_initialize_axis programs the cutoff via mock isensor's
     * set_filter_cutoff and stores fc on the axis itself. */
    TEST_ASSERT_TRUE(s_isensor.set_filter_cutoff_count >= 1);

    uint8_t resp[4] = {0};
    size_t rl = sizeof(resp);
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_tuner_handle_request(
        0, ESP_FOC_TUNER_OP_READ, ESP_FOC_TUNER_PARAM_I_FILTER_FC_Q16,
        NULL, 0, resp, &rl));
    TEST_ASSERT_EQUAL(4, rl);
    q16_t fc = deserialize_q16_le(resp);
    TEST_ASSERT_EQUAL_INT32(s_axis.current_filter_fc_hz_q16, fc);
}

TEST_CASE("tuner: write I_FILTER_FC re-runs the designer on the driver",
          "[espFoC][tuner]")
{
    setup_attached_axis();
    int before = s_isensor.set_filter_cutoff_count;

    uint8_t pl[4];
    q16_t new_fc = q16_from_float(700.0f);
    serialize_q16_le(pl, new_fc);
    size_t resp_len = 0;
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_tuner_handle_request(
        0, ESP_FOC_TUNER_OP_WRITE, ESP_FOC_TUNER_WRITE_I_FILTER_FC_Q16,
        pl, sizeof(pl), NULL, &resp_len));

    TEST_ASSERT_EQUAL(before + 1, s_isensor.set_filter_cutoff_count);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 700.0f, s_isensor.last_filter_fc);
    TEST_ASSERT_EQUAL_INT32(new_fc, s_axis.current_filter_fc_hz_q16);
}

TEST_CASE("tuner: motion writes land in shadow when override is ON",
          "[espFoC][tuner]")
{
    setup_attached_axis();
    s_axis.rotor_aligned = ESP_FOC_OK;

    size_t resp_len = 0;
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_tuner_handle_request(
        0, ESP_FOC_TUNER_OP_EXEC, ESP_FOC_TUNER_CMD_OVERRIDE_ON,
        NULL, 0, NULL, &resp_len));

    q16_t targets[4] = {
        q16_from_float(0.10f),  /* id */
        q16_from_float(2.50f),  /* iq */
        q16_from_float(0.30f),  /* ud */
        q16_from_float(4.00f),  /* uq */
    };
    esp_foc_tuner_id_t ids[4] = {
        ESP_FOC_TUNER_WRITE_TARGET_ID_Q16,
        ESP_FOC_TUNER_WRITE_TARGET_IQ_Q16,
        ESP_FOC_TUNER_WRITE_TARGET_UD_Q16,
        ESP_FOC_TUNER_WRITE_TARGET_UQ_Q16,
    };
    for (int i = 0; i < 4; ++i) {
        uint8_t pl[4];
        serialize_q16_le(pl, targets[i]);
        resp_len = 0;
        TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_tuner_handle_request(
            0, ESP_FOC_TUNER_OP_WRITE, ids[i],
            pl, sizeof(pl), NULL, &resp_len));
    }

    TEST_ASSERT_EQUAL_INT32(targets[0], s_axis.tuner_override.target_id);
    TEST_ASSERT_EQUAL_INT32(targets[1], s_axis.tuner_override.target_iq);
    TEST_ASSERT_EQUAL_INT32(targets[2], s_axis.tuner_override.target_ud);
    TEST_ASSERT_EQUAL_INT32(targets[3], s_axis.tuner_override.target_uq);
    /* Public targets stay zero — nothing happened to axis->target_* yet. */
    TEST_ASSERT_EQUAL_INT32(0, s_axis.target_i_d.raw);
    TEST_ASSERT_EQUAL_INT32(0, s_axis.target_i_q.raw);
}

#else

/* Tuner disabled: keep this TU non-empty so the test build stays consistent. */
TEST_CASE("tuner: disabled at Kconfig", "[espFoC][tuner]")
{
    TEST_IGNORE_MESSAGE("CONFIG_ESP_FOC_TUNER_ENABLE not set");
}

#endif
