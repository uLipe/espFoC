/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include <assert.h>
#include <string.h>
#include <stdbool.h>
#include "espFoC/esp_foc.h"
#include "espFoC/stream/esp_foc_stream_frame.h"
#include "espFoC/stream/esp_foc_stream_bridge.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#ifndef CONFIG_ESP_FOC_SCOPE_DAEMON_PRIORITY
#define CONFIG_ESP_FOC_SCOPE_DAEMON_PRIORITY 3
#endif

const char *TAG = "ESP_FOC_SCOPE";

struct scope_frame {
    q16_t data[CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS];
};

static uint32_t used_channels = 0;
static q16_t *scope_channels[CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS];
static esp_foc_event_handle_t scope_ev;
static esp_foc_axis_t *s_axis;

static bool scope_enable = false;
static struct scope_frame scope_buffer[2][CONFIG_ESP_FOC_SCOPE_BUFFER_SIZE];
static bool ping_pong_switch = false;
static volatile uint8_t s_tx_bank;
static uint32_t rd_buff_index = 0;
static uint32_t wr_buff_index = 0;
static uint16_t s_frame_seq;

static inline void put_u16_le(uint8_t *d, uint16_t v)
{
    d[0] = (uint8_t)(v & 0xFFU);
    d[1] = (uint8_t)((v >> 8) & 0xFFU);
}

static inline void put_i32_le(uint8_t *d, int32_t v)
{
    d[0] = (uint8_t)((uint32_t)v & 0xFFU);
    d[1] = (uint8_t)(((uint32_t)v >> 8) & 0xFFU);
    d[2] = (uint8_t)(((uint32_t)v >> 16) & 0xFFU);
    d[3] = (uint8_t)(((uint32_t)v >> 24) & 0xFFU);
}

static bool scope_may_transmit(void)
{
#if defined(CONFIG_ESP_FOC_SCOPE_TX_ALWAYS)
    (void)s_axis;
    return true;
#else
    if (s_axis == NULL) {
        return false;
    }
    return (s_axis->state == ESP_FOC_AXIS_STATE_RUNNING
            || s_axis->state == ESP_FOC_AXIS_STATE_BENCH);
#endif
}

static void scope_send_espf_frame(const struct scope_frame *sample)
{
    uint8_t out[ESP_FOC_STREAM_FIXED_BYTES
                 + 4U * CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS];
    const uint16_t n_ch = (uint16_t)CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS;
    uint8_t *wp = out;

    wp[0] = ESP_FOC_STREAM_MAGIC_HDR_0;
    wp[1] = ESP_FOC_STREAM_MAGIC_HDR_1;
    wp[2] = ESP_FOC_STREAM_MAGIC_HDR_2;
    wp[3] = ESP_FOC_STREAM_MAGIC_HDR_3;
    wp += 4;
    put_u16_le(wp, s_frame_seq++);
    put_u16_le(wp + 2, n_ch);
    wp += 4;
    for (int i = 0; i < CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS; i++) {
        put_i32_le(wp, (int32_t)sample->data[i]);
        wp += 4;
    }
    wp[0] = ESP_FOC_STREAM_MAGIC_FTR_0;
    wp[1] = ESP_FOC_STREAM_MAGIC_FTR_1;
    wp[2] = ESP_FOC_STREAM_MAGIC_FTR_2;
    wp[3] = ESP_FOC_STREAM_MAGIC_FTR_3;

    esp_foc_stream_bridge_send_frame(out, esp_foc_stream_frame_total_bytes(n_ch));
}

static void esp_foc_scope_daemon_thread(void *arg)
{
    struct scope_frame *next_sample;

    (void)arg;
    ESP_LOGI(TAG, "daemon running (runner prio %d, RT prio %u)",
             CONFIG_ESP_FOC_SCOPE_DAEMON_PRIORITY,
             (unsigned)(configMAX_PRIORITIES - CONFIG_ESP_FOC_SCOPE_DAEMON_PRIORITY));
    memset(&scope_buffer, 0, sizeof(scope_buffer));

    while (1) {
        esp_foc_wait_notifier();
        const uint8_t bank = s_tx_bank;
        rd_buff_index = 0;
        while (rd_buff_index < CONFIG_ESP_FOC_SCOPE_BUFFER_SIZE) {
            next_sample = &scope_buffer[bank][rd_buff_index];
            if (scope_may_transmit()) {
                scope_send_espf_frame(next_sample);
            }
            rd_buff_index++;
            esp_foc_runner_yield();
        }
    }
}

void esp_foc_scope_bind_axis(esp_foc_axis_t *axis)
{
    s_axis = axis;
}

esp_foc_err_t esp_foc_scope_wire_axis(esp_foc_axis_t *axis)
{
    if (axis == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
#if CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS != 17
    ESP_LOGE(TAG, "wire_axis needs CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS=17");
    return ESP_FOC_ERR_INVALID_ARG;
#else
    esp_foc_scope_bind_axis(axis);
    used_channels = 0;

    if (esp_foc_scope_add_channel(&axis->target_i_d.raw, 0) != 0) {
        return ESP_FOC_ERR_UNKNOWN;
    }
    if (esp_foc_scope_add_channel(&axis->i_d.raw, 1) != 0) {
        return ESP_FOC_ERR_UNKNOWN;
    }
    if (esp_foc_scope_add_channel(&axis->target_i_q.raw, 2) != 0) {
        return ESP_FOC_ERR_UNKNOWN;
    }
    if (esp_foc_scope_add_channel(&axis->i_q.raw, 3) != 0) {
        return ESP_FOC_ERR_UNKNOWN;
    }
    if (esp_foc_scope_add_channel(&axis->u_d.raw, 4) != 0) {
        return ESP_FOC_ERR_UNKNOWN;
    }
    if (esp_foc_scope_add_channel(&axis->u_q.raw, 5) != 0) {
        return ESP_FOC_ERR_UNKNOWN;
    }
    if (esp_foc_scope_add_channel(
            (const q16_t *)(const volatile void *)&axis->rotor_estimator.theta_meas_mech,
            6) != 0) {
        return ESP_FOC_ERR_UNKNOWN;
    }
    if (esp_foc_scope_add_channel(
            (const q16_t *)(const volatile void *)&axis->rotor_elec_angle, 7) != 0) {
        return ESP_FOC_ERR_UNKNOWN;
    }
    if (esp_foc_scope_add_channel(&axis->rotor_estimator.omega_est_mech, 8) != 0) {
        return ESP_FOC_ERR_UNKNOWN;
    }
    if (esp_foc_scope_add_channel(&axis->u_u.raw, 9) != 0) {
        return ESP_FOC_ERR_UNKNOWN;
    }
    if (esp_foc_scope_add_channel(&axis->u_v.raw, 10) != 0) {
        return ESP_FOC_ERR_UNKNOWN;
    }
    if (esp_foc_scope_add_channel(&axis->u_w.raw, 11) != 0) {
        return ESP_FOC_ERR_UNKNOWN;
    }
    if (esp_foc_scope_add_channel(&axis->i_u, 12) != 0) {
        return ESP_FOC_ERR_UNKNOWN;
    }
    if (esp_foc_scope_add_channel(&axis->i_v, 13) != 0) {
        return ESP_FOC_ERR_UNKNOWN;
    }
    if (esp_foc_scope_add_channel(&axis->i_alpha.raw, 14) != 0) {
        return ESP_FOC_ERR_UNKNOWN;
    }
    if (esp_foc_scope_add_channel(&axis->i_beta.raw, 15) != 0) {
        return ESP_FOC_ERR_UNKNOWN;
    }
    if (esp_foc_scope_add_channel(&esp_foc_debug_scope_hot_path_dt_us_q16, 16) != 0) {
        return ESP_FOC_ERR_UNKNOWN;
    }

    ESP_LOGI(TAG, "wired axis %p (17ch)", (void *)axis);
    return ESP_FOC_OK;
#endif
}

bool esp_foc_scope_is_initialized(void)
{
    return scope_enable && scope_ev != NULL && esp_foc_runner_is_alive(scope_ev);
}

void esp_foc_scope_initalize(void)
{
    if (scope_enable) {
        return;
    }

    esp_foc_stream_bridge_init();

    void *daemon_handle = NULL;
    if (esp_foc_create_runner(esp_foc_scope_daemon_thread, NULL,
                              CONFIG_ESP_FOC_SCOPE_DAEMON_PRIORITY,
                              &daemon_handle) != 0) {
        ESP_LOGE(TAG, "scope daemon task create failed");
        return;
    }

    scope_ev = (esp_foc_event_handle_t)daemon_handle;
    s_frame_seq = 0;
    scope_enable = true;
    esp_foc_sleep_ms(10);
}

static inline bool scope_capture_frame(void)
{
    struct scope_frame *next_sample =
        &scope_buffer[!ping_pong_switch][wr_buff_index];
    for (int i = 0; i < CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS; i++) {
        if (used_channels & (1 << i)) {
            next_sample->data[i] = *scope_channels[i];
        } else {
            next_sample->data[i] = 0;
        }
    }
    wr_buff_index++;
    if (wr_buff_index >= CONFIG_ESP_FOC_SCOPE_BUFFER_SIZE) {
        wr_buff_index = 0;
        s_tx_bank = (uint8_t)!ping_pong_switch;
        ping_pong_switch ^= 1;
        return true;
    }
    return false;
}

static void esp_foc_scope_data_push_impl(void)
{
    if (!scope_enable || scope_ev == NULL) {
        return;
    }
    if (scope_capture_frame()) {
        esp_foc_notify_runner(scope_ev);
    }
}

void esp_foc_scope_data_push(void)
{
    esp_foc_scope_data_push_impl();
}

void esp_foc_scope_data_push_from_isr(void)
{
    esp_foc_scope_data_push_impl();
}

int esp_foc_scope_add_channel(const q16_t *data_to_wire, int channel_number)
{
    if (used_channels & (1 << channel_number)) {
        ESP_LOGE(TAG, "Channel already used, select another! ");
        return -1;
    }

    if (channel_number > (CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS - 1)) {
        ESP_LOGE(TAG, "Invalid channel!");
        return -1;
    }

    esp_foc_critical_enter();
    used_channels |= (1U << channel_number);
    scope_channels[channel_number] = (q16_t *)data_to_wire;
    esp_foc_critical_leave();

    ESP_LOGD(TAG, "Wire data %p is attached to channel %d", data_to_wire, channel_number);
    return 0;
}
