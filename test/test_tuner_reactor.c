/*
 * MIT License
 *
 * End-to-end tests for the tuner reactor: feed bytes into
 * esp_foc_tuner_process_byte() and observe the response frame emitted via
 * the (weakly defined) esp_foc_tuner_send_callback hook.
 *
 * Strongly overrides the weak callback so we can capture what the firmware
 * would push onto the wire. Runs entirely in-process under QEMU with no
 * physical transport.
 */

#include <string.h>
#include <unity.h>
#include "espFoC/esp_foc.h"
#include "espFoC/esp_foc_tuner.h"
#include "espFoC/esp_foc_axis_tuning.h"
#include "espFoC/esp_foc_link.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "mock_drivers.h"

#if defined(CONFIG_ESP_FOC_TUNER_ENABLE)

static esp_foc_axis_t s_axis;
static mock_inverter_t s_inv;
static mock_rotor_sensor_t s_rotor;
static mock_isensor_t s_isensor;

#define CAPTURE_CAP 256
static uint8_t s_capture[CAPTURE_CAP];
static size_t s_capture_len;

void esp_foc_tuner_send_callback(const uint8_t *buf, size_t len)
{
    /* Append to the capture buffer. The protocol always answers with a
     * single frame per request, so a 256-byte buffer is plenty. */
    if (s_capture_len + len > sizeof(s_capture)) {
        len = sizeof(s_capture) - s_capture_len;
    }
    memcpy(&s_capture[s_capture_len], buf, len);
    s_capture_len += len;
}

static void setup_axis(void)
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
    s_capture_len = 0;
    esp_foc_tuner_reactor_reset();
}

/* Build an application-level request: [op:u8][id:u16 LE][axis:u8][cmd_payload]
 * and wrap it in a tuner-channel link frame; push every byte through
 * esp_foc_tuner_process_byte. */
static void send_request(uint8_t seq,
                         esp_foc_tuner_op_t op,
                         esp_foc_tuner_id_t id,
                         uint8_t axis_id,
                         const uint8_t *cmd_payload,
                         size_t cmd_payload_len)
{
    uint8_t app[ESP_FOC_LINK_MAX_PAYLOAD];
    TEST_ASSERT_TRUE(4 + cmd_payload_len <= sizeof(app));
    app[0] = (uint8_t)op;
    app[1] = (uint8_t)((uint16_t)id & 0xFF);
    app[2] = (uint8_t)(((uint16_t)id >> 8) & 0xFF);
    app[3] = axis_id;
    if (cmd_payload_len > 0) {
        memcpy(&app[4], cmd_payload, cmd_payload_len);
    }

    uint8_t frame[ESP_FOC_LINK_MAX_FRAME];
    size_t frame_len = 0;
    TEST_ASSERT_EQUAL(ESP_FOC_LINK_OK,
        esp_foc_link_encode(ESP_FOC_LINK_CH_TUNER, seq, app, 4 + cmd_payload_len,
                            frame, sizeof(frame), &frame_len));

    s_capture_len = 0;
    for (size_t i = 0; i < frame_len; ++i) {
        esp_foc_tuner_process_byte(frame[i]);
    }
}

/* Decode the most recent response frame written into s_capture and pull
 * the application status / payload out of it. */
static esp_foc_err_t decode_response(uint8_t expected_seq,
                                     uint8_t *resp_payload,
                                     size_t  *resp_payload_len)
{
    esp_foc_link_decoder_t dec;
    esp_foc_link_decoder_reset(&dec);
    esp_foc_link_status_t st = ESP_FOC_LINK_ERR_NEED_MORE;
    for (size_t i = 0; i < s_capture_len; ++i) {
        st = esp_foc_link_decoder_push(&dec, s_capture[i]);
    }
    TEST_ASSERT_EQUAL(ESP_FOC_LINK_OK, st);
    TEST_ASSERT_EQUAL(ESP_FOC_LINK_CH_TUNER, esp_foc_link_decoder_channel(&dec));
    TEST_ASSERT_EQUAL(expected_seq, esp_foc_link_decoder_seq(&dec));
    const uint8_t *app = esp_foc_link_decoder_payload(&dec);
    size_t alen = esp_foc_link_decoder_payload_len(&dec);
    TEST_ASSERT_TRUE(alen >= 2);  /* status + seq */
    int8_t status = (int8_t)app[0];
    TEST_ASSERT_EQUAL(expected_seq, app[1]);
    if (resp_payload != NULL && resp_payload_len != NULL) {
        size_t copy = alen - 2;
        if (copy > *resp_payload_len) {
            copy = *resp_payload_len;
        }
        memcpy(resp_payload, &app[2], copy);
        *resp_payload_len = copy;
    }
    return (esp_foc_err_t)status;
}

/* --- Reactor tests ----------------------------------------------------- */

TEST_CASE("reactor: read kp returns axis Kp through link frame",
          "[espFoC][reactor]")
{
    setup_axis();
    send_request(0x10, ESP_FOC_TUNER_OP_READ, ESP_FOC_TUNER_PARAM_KP_Q16,
                 0, NULL, 0);

    uint8_t payload[16];
    size_t plen = sizeof(payload);
    TEST_ASSERT_EQUAL(ESP_FOC_OK, decode_response(0x10, payload, &plen));
    TEST_ASSERT_EQUAL_UINT(4, plen);
    /* Decode q16 LE and verify it matches the axis. */
    int32_t got = (int32_t)((uint32_t)payload[0] |
                            ((uint32_t)payload[1] << 8) |
                            ((uint32_t)payload[2] << 16) |
                            ((uint32_t)payload[3] << 24));
    q16_t expected;
    esp_foc_axis_get_current_pi_gains_q16(&s_axis, &expected, NULL, NULL);
    TEST_ASSERT_EQUAL_INT32(expected, got);
}

TEST_CASE("reactor: write kp lands on axis", "[espFoC][reactor]")
{
    setup_axis();
    q16_t target = q16_from_float(3.14f);
    uint8_t cmd[4] = {
        (uint8_t)target,
        (uint8_t)(target >> 8),
        (uint8_t)(target >> 16),
        (uint8_t)(target >> 24),
    };
    send_request(0x55, ESP_FOC_TUNER_OP_WRITE, ESP_FOC_TUNER_WRITE_KP_Q16,
                 0, cmd, sizeof(cmd));

    size_t plen = 0;
    TEST_ASSERT_EQUAL(ESP_FOC_OK, decode_response(0x55, NULL, &plen));
    q16_t got;
    esp_foc_axis_get_current_pi_gains_q16(&s_axis, &got, NULL, NULL);
    TEST_ASSERT_EQUAL_INT32(target, got);
}

TEST_CASE("reactor: short request payload returns INVALID_ARG",
          "[espFoC][reactor]")
{
    setup_axis();
    /* Build a frame whose application payload is too short (just 2 bytes). */
    uint8_t app[2] = {0x01, 0x10};
    uint8_t frame[ESP_FOC_LINK_MAX_FRAME];
    size_t frame_len = 0;
    TEST_ASSERT_EQUAL(ESP_FOC_LINK_OK,
        esp_foc_link_encode(ESP_FOC_LINK_CH_TUNER, 0xAA, app, sizeof(app),
                            frame, sizeof(frame), &frame_len));
    s_capture_len = 0;
    for (size_t i = 0; i < frame_len; ++i) {
        esp_foc_tuner_process_byte(frame[i]);
    }
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG, decode_response(0xAA, NULL, NULL));
}

TEST_CASE("reactor: scope-channel frame is silently ignored",
          "[espFoC][reactor]")
{
    setup_axis();
    /* Encode a scope-channel frame; the reactor must NOT respond. */
    uint8_t pl[4] = {0xAA, 0xBB, 0xCC, 0xDD};
    uint8_t frame[ESP_FOC_LINK_MAX_FRAME];
    size_t frame_len = 0;
    TEST_ASSERT_EQUAL(ESP_FOC_LINK_OK,
        esp_foc_link_encode(ESP_FOC_LINK_CH_SCOPE, 0, pl, sizeof(pl),
                            frame, sizeof(frame), &frame_len));
    s_capture_len = 0;
    for (size_t i = 0; i < frame_len; ++i) {
        esp_foc_tuner_process_byte(frame[i]);
    }
    TEST_ASSERT_EQUAL_UINT(0, s_capture_len);
}

TEST_CASE("reactor: corrupt CRC produces no response",
          "[espFoC][reactor]")
{
    setup_axis();
    /* Build a valid READ KP frame and then flip the last byte. */
    uint8_t app[4] = {ESP_FOC_TUNER_OP_READ, 0x10, 0x00, 0};
    uint8_t frame[ESP_FOC_LINK_MAX_FRAME];
    size_t frame_len = 0;
    TEST_ASSERT_EQUAL(ESP_FOC_LINK_OK,
        esp_foc_link_encode(ESP_FOC_LINK_CH_TUNER, 0x07, app, sizeof(app),
                            frame, sizeof(frame), &frame_len));
    frame[frame_len - 1] ^= 0xFF;
    s_capture_len = 0;
    for (size_t i = 0; i < frame_len; ++i) {
        esp_foc_tuner_process_byte(frame[i]);
    }
    TEST_ASSERT_EQUAL_UINT(0, s_capture_len);
}

TEST_CASE("reactor: exec recompute through link returns OK",
          "[espFoC][reactor]")
{
    setup_axis();
    q16_t r  = q16_from_float(1.08f);
    q16_t l  = q16_from_float(0.0018f);
    q16_t bw = q16_from_float(150.0f);
    uint8_t cmd[12];
    for (int i = 0; i < 4; ++i) {
        cmd[i + 0] = (uint8_t)((uint32_t)r  >> (8 * i));
        cmd[i + 4] = (uint8_t)((uint32_t)l  >> (8 * i));
        cmd[i + 8] = (uint8_t)((uint32_t)bw >> (8 * i));
    }
    send_request(0x33, ESP_FOC_TUNER_OP_EXEC, ESP_FOC_TUNER_CMD_RECOMPUTE_GAINS,
                 0, cmd, sizeof(cmd));
    size_t plen = 0;
    TEST_ASSERT_EQUAL(ESP_FOC_OK, decode_response(0x33, NULL, &plen));
}

#endif /* CONFIG_ESP_FOC_TUNER_ENABLE */
