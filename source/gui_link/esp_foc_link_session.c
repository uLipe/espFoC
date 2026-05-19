/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include <string.h>
#include "espFoC/gui_link/esp_foc_link_session.h"
#include "espFoC/gui_link/esp_foc_tuner.h"
#include "espFoC/gui_link/esp_foc_scope.h"
#include "espFoC/osal/os_interface.h"

extern void esp_foc_tuner_send_callback(const uint8_t *buf, size_t len);
extern uint32_t esp_foc_tuner_firmware_type(void);

void esp_foc_tuner_dispatch_link_frame(uint8_t seq,
                                       const uint8_t *payload,
                                       size_t payload_len);

static esp_foc_link_decoder_t s_dec;
static bool s_dec_ready;

static bool s_connected;
static bool s_force_connected;
static bool s_scope_stream;
static bool s_hb_started;
static bool s_ack_since_last_hb;
static uint8_t s_hb_miss_streak;
static uint8_t s_hb_counter;
static volatile bool s_tx_high_prio;

static uint8_t s_primary_axis_state(void);
static int8_t s_primary_axis_last_err(void);

bool esp_foc_link_try_acquire_tx_low_prio(void)
{
    if (s_tx_high_prio) {
        return false;
    }
    return true;
}

void esp_foc_link_release_tx_low_prio(void)
{
    (void)0;
}

static void link_send_frame(esp_foc_link_channel_t ch,
                            uint8_t seq,
                            const uint8_t *payload,
                            size_t len,
                            bool high_prio)
{
    uint8_t frame[ESP_FOC_LINK_MAX_FRAME];
    size_t frame_len = 0;

    if (high_prio) {
        s_tx_high_prio = true;
    }
    if (esp_foc_link_encode(ch, seq, payload, len,
                            frame, sizeof(frame), &frame_len) == ESP_FOC_LINK_OK) {
        esp_foc_tuner_send_callback(frame, frame_len);
    }
    if (high_prio) {
        s_tx_high_prio = false;
    }
}

static void send_heartbeat_fw(void)
{
    uint8_t body[4];

    if (!s_connected && !s_force_connected) {
        return;
    }
    if (!s_ack_since_last_hb) {
        if (s_hb_miss_streak < 255) {
            s_hb_miss_streak++;
        }
        if (s_hb_miss_streak >= ESP_FOC_LINK_HEARTBEAT_MISS_MAX) {
            esp_foc_link_session_end();
            return;
        }
    } else {
        s_hb_miss_streak = 0;
    }
    s_ack_since_last_hb = false;

    s_hb_counter++;
    body[0] = ESP_FOC_HB_MSG_FW;
    body[1] = s_hb_counter;
    body[2] = s_primary_axis_state();
    body[3] = (uint8_t)s_primary_axis_last_err();
    link_send_frame(ESP_FOC_LINK_CH_HEARTBEAT, 0, body, sizeof(body), true);
}

static void hb_task(void *arg)
{
    (void)arg;
    while (1) {
        esp_foc_sleep_ms(ESP_FOC_LINK_HEARTBEAT_PERIOD_MS);
        send_heartbeat_fw();
    }
}

static void on_hb_ack(const uint8_t *payload, size_t len)
{
    if (len < 2 || payload[0] != ESP_FOC_HB_MSG_ACK) {
        return;
    }
    if (!s_connected && !s_force_connected) {
        return;
    }
    (void)payload[1];
    s_ack_since_last_hb = true;
    s_hb_miss_streak = 0;
}

bool esp_foc_link_session_is_connected(void)
{
    return s_connected || s_force_connected;
}

bool esp_foc_link_session_scope_streaming(void)
{
    return s_scope_stream && esp_foc_link_session_is_connected();
}

void esp_foc_link_session_end(void)
{
    s_connected = false;
    s_scope_stream = false;
    s_hb_miss_streak = 0;
    s_ack_since_last_hb = false;
    esp_foc_scope_set_stream_enabled(false);
}

void esp_foc_link_session_force_connected(bool connected)
{
    s_force_connected = connected;
    if (!connected) {
        s_scope_stream = false;
        esp_foc_scope_set_stream_enabled(false);
    }
}

void esp_foc_link_session_start(void)
{
    if (s_hb_started) {
        return;
    }
    s_hb_started = true;
    (void)esp_foc_task_spawn(hb_task, NULL, 3072, 4, NULL);
}

static uint8_t s_primary_axis_state(void)
{
    uint8_t resp = 0;
    size_t rl = sizeof(resp);
    if (esp_foc_tuner_handle_request(0, ESP_FOC_TUNER_OP_READ,
                                     ESP_FOC_TUNER_PARAM_AXIS_STATE,
                                     NULL, 0, &resp, &rl) == ESP_FOC_OK) {
        return resp;
    }
    return 0;
}

static int8_t s_primary_axis_last_err(void)
{
    uint8_t resp = 0;
    size_t rl = sizeof(resp);
    if (esp_foc_tuner_handle_request(0, ESP_FOC_TUNER_OP_READ,
                                     ESP_FOC_TUNER_PARAM_AXIS_LAST_ERR,
                                     NULL, 0, &resp, &rl) == ESP_FOC_OK) {
        return (int8_t)resp;
    }
    return 0;
}

bool esp_foc_link_session_tuner_may_dispatch(esp_foc_tuner_op_t op,
                                             esp_foc_tuner_id_t id,
                                             bool *silent_drop)
{
    if (silent_drop != NULL) {
        *silent_drop = false;
    }
    if (op == ESP_FOC_TUNER_OP_EXEC && id == ESP_FOC_TUNER_CMD_CONNECT) {
        if (s_connected) {
            if (silent_drop != NULL) {
                *silent_drop = true;
            }
            return false;
        }
        return true;
    }
    if (!esp_foc_link_session_is_connected()) {
        if (silent_drop != NULL) {
            *silent_drop = true;
        }
        return false;
    }
    return true;
}

void esp_foc_link_session_on_connect(void)
{
    s_connected = true;
    s_hb_miss_streak = 0;
    s_ack_since_last_hb = true;
    s_hb_counter = 0;
}

void esp_foc_link_session_on_disconnect(void)
{
    esp_foc_link_session_end();
}

void esp_foc_link_session_on_scope_start(void)
{
    s_scope_stream = true;
    esp_foc_scope_set_stream_enabled(true);
}

void esp_foc_link_session_on_scope_stop(void)
{
    s_scope_stream = false;
    esp_foc_scope_set_stream_enabled(false);
}

void esp_foc_link_reactor_reset(void)
{
    esp_foc_link_decoder_reset(&s_dec);
    s_dec_ready = true;
}

void esp_foc_link_process_byte(uint8_t byte)
{
    if (!s_dec_ready) {
        esp_foc_link_reactor_reset();
    }
    if (esp_foc_link_decoder_push(&s_dec, byte) != ESP_FOC_LINK_OK) {
        return;
    }

    const esp_foc_link_channel_t ch =
        (esp_foc_link_channel_t)esp_foc_link_decoder_channel(&s_dec);
    const uint8_t seq = esp_foc_link_decoder_seq(&s_dec);
    const uint8_t *payload = esp_foc_link_decoder_payload(&s_dec);
    const size_t plen = esp_foc_link_decoder_payload_len(&s_dec);

    if (ch == ESP_FOC_LINK_CH_HEARTBEAT) {
        on_hb_ack(payload, plen);
    } else if (ch == ESP_FOC_LINK_CH_TUNER) {
        esp_foc_tuner_dispatch_link_frame(seq, payload, plen);
    }
    esp_foc_link_decoder_reset(&s_dec);
}
