/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <assert.h>
#include <string.h>
#include <stdbool.h>
#include "espFoC/esp_foc.h"
#include "espFoC/esp_foc_link.h"
#include "esp_log.h"
#if defined(CONFIG_ESP_FOC_SCOPE_LEGACY_CSV) && CONFIG_ESP_FOC_SCOPE_LEGACY_CSV
#include <stdio.h>
#include "espFoC/utils/esp_foc_q16.h"
#endif

const char *TAG = "ESP_FOC_SCOPE";

#if !defined(CONFIG_ESP_FOC_SCOPE_LEGACY_CSV) || !CONFIG_ESP_FOC_SCOPE_LEGACY_CSV
/* Binary SCOPE v1: 0xFF 'S' 'C' 0x01, uint16le n, n × int32le (q16_t). */
#define SCOPE_WIRE_V1 0x01U
/* No cast: #if and constexpr-like checks in some GCC builds reject
 * (unsigned) around CONFIG_* in the preprocessor. */
#define SCOPE_BIN_BODY_LEN  (6U + 4U * CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS)
_Static_assert(SCOPE_BIN_BODY_LEN <= ESP_FOC_LINK_MAX_PAYLOAD,
    "Scope v1 frame exceeds link payload; reduce CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS");

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
#endif

struct scope_frame {
    /* Buffered as raw Q16 so the data_push path stays float-free —
     * Xtensa FPU context save in an ISR is non-trivial and the
     * conversion is cheap to defer to the daemon thread anyway. */
    q16_t data[CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS];
};

static uint32_t used_channels = 0;
static q16_t *scope_channels[CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS];
static esp_foc_event_handle_t scope_ev;

static bool scope_enable = false;
static struct scope_frame scope_buffer[2][CONFIG_ESP_FOC_SCOPE_BUFFER_SIZE];
static bool ping_pong_switch = false;
static uint32_t rd_buff_index = 0;
static uint32_t wr_buff_index = 0;

static void esp_foc_scope_daemon_thread(void *arg)
{
    struct scope_frame *next_sample;

    (void)arg;
    memset(&scope_buffer, 0, sizeof(scope_buffer));
    scope_ev = esp_foc_get_event_handle();
    esp_foc_wait_notifier();

    while(1) {

        if(rd_buff_index >= CONFIG_ESP_FOC_SCOPE_BUFFER_SIZE) {
            esp_foc_wait_notifier();
        }

        next_sample = &scope_buffer[ping_pong_switch][rd_buff_index];
#if defined(CONFIG_ESP_FOC_SCOPE_LEGACY_CSV) && CONFIG_ESP_FOC_SCOPE_LEGACY_CSV
    {
        int idx = 0;
        char line_buf[512];

        if(used_channels != 0) {
            for (int j = 0; j < CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS; j++) {
                if(used_channels & (1U << j)) {
                    idx += sprintf(&line_buf[idx], "%f,",
                        q16_to_float(next_sample->data[j]));
                }
            }
            if (idx > 0) {
                line_buf[idx - 1] = '\n';
            }
        } else {
            idx = sprintf(&line_buf[0], "%f",
                q16_to_float(next_sample->data[0]));
            line_buf[idx++] = '\n';
            line_buf[idx]   = 0;
        }
        esp_foc_send_buffer_callback((const uint8_t *)line_buf, (size_t)idx);
    }
#else
    {
        static uint8_t out_buf[ESP_FOC_LINK_MAX_PAYLOAD];
        const size_t olen = SCOPE_BIN_BODY_LEN;
        uint8_t *wp;
        int i;

        wp = out_buf;
        *wp++ = 0xFFU;
        *wp++ = (uint8_t) 'S';
        *wp++ = (uint8_t) 'C';
        *wp++ = SCOPE_WIRE_V1;
        put_u16_le(wp, (uint16_t) CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS);
        wp += 2;
        for (i = 0; i < CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS; i++) {
            put_i32_le(wp, (int32_t) next_sample->data[i]);
            wp += 4;
        }
        esp_foc_send_buffer_callback((const uint8_t *)out_buf, olen);
    }
#endif
        rd_buff_index++;
    }
}

void esp_foc_scope_initalize(void)
{
    if(!scope_enable) {
        scope_enable = true;
        esp_foc_init_bus_callback();
        esp_foc_create_runner(esp_foc_scope_daemon_thread, NULL, -1);
        esp_foc_sleep_ms(10);
    }
}

/* Snapshot every wired channel into the active write buffer. Returns
 * true when the buffer just rolled over (caller is responsible for
 * waking the daemon — the two public push variants below handle the
 * wake with the appropriate FreeRTOS API for their context). */
static inline bool scope_capture_frame(void)
{
    struct scope_frame *next_sample =
        &scope_buffer[!ping_pong_switch][wr_buff_index];
    for(int i = 0; i < CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS; i++) {
        if(used_channels & (1 << i)) {
            next_sample->data[i] = *scope_channels[i];
        } else {
            next_sample->data[i] = 0;
        }
    }
    wr_buff_index++;
    if(wr_buff_index >= CONFIG_ESP_FOC_SCOPE_BUFFER_SIZE) {
        rd_buff_index = 0;
        wr_buff_index = 0;
        ping_pong_switch ^= 1;
        return true;
    }
    return false;
}

void esp_foc_scope_data_push(void)
{
    if(!scope_enable) {
        ESP_LOGW(TAG, "ESP FOC scope is not ready yet, skipping...");
        return;
    }
    if(scope_capture_frame()) {
        esp_foc_send_notification(scope_ev);
    }
}

void esp_foc_scope_data_push_from_isr(void)
{
    /* ISR-context push for callers that live in IRAM (e.g. the FOC
     * hot path inside the PWM ISR). Same buffer protocol as the
     * task-context push, but the wake uses the FromISR notification
     * variant so we never call xTaskNotifyGive from interrupt land.
     *
     * No ESP_LOGW guard here on purpose — printing inside an ISR
     * would be way worse than silently dropping the rare case where
     * push fires before scope_initalize(). */
    if(!scope_enable) {
        return;
    }
    if(scope_capture_frame()) {
        esp_foc_send_notification_from_isr(scope_ev);
    }
}

int esp_foc_scope_add_channel(const q16_t *data_to_wire, int channel_number)
{
    if(used_channels & (1 << channel_number)) {
        ESP_LOGE(TAG, "Channel already used, select another! ");
        return -1;
    }

    if(channel_number > (CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS - 1)) {
        ESP_LOGE(TAG, "Invalid channel!");
        return -1;
    }

    esp_foc_critical_enter();
    used_channels |= (1 << channel_number);
    scope_channels[channel_number] = (q16_t *)data_to_wire;
    esp_foc_critical_leave();

    ESP_LOGI(TAG, "Wire data %p is attached to channel %d", data_to_wire, channel_number);
    return 0;
}