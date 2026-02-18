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

#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include "espFoC/esp_foc.h"
#include "esp_log.h"

const char *TAG = "ESP_FOC_SCOPE";

struct scope_frame {
    float data[CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS];
};

static uint32_t used_channels = 0;
static float *scope_channels[CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS];
static esp_foc_event_handle_t scope_ev;

static bool scope_enable = false;
static struct scope_frame scope_buffer[2][CONFIG_ESP_FOC_SCOPE_BUFFER_SIZE];
static bool ping_pong_switch = false;
static uint32_t rd_buff_index = 0;
static uint32_t wr_buff_index = 0;

__attribute__((weak)) void esp_foc_init_bus_callback(void)
{
    ESP_LOGW("ESP_FOC_SCOPE", "Missing implementation of the init buffer callback for the scope!");
    esp_foc_sleep_ms(100);
}


__attribute__((weak)) void esp_foc_send_buffer_callback(const uint8_t *buffer, int size)
{
    (void)buffer;
    (void)size;
    ESP_LOGW("ESP_FOC_SCOPE", "Missing implementation of the buffer callback for the scope!");
    esp_foc_sleep_ms(100);
}

static void esp_foc_scope_daemon_thread(void *arg)
{
    int idx = 0;
    char out_buf[512] = {0,};
    struct scope_frame *next_sample;
    memset(&scope_buffer, 0, sizeof(scope_buffer));
    scope_ev = esp_foc_get_event_handle();
    esp_foc_wait_notifier();

    while(1) {

        if(rd_buff_index >= CONFIG_ESP_FOC_SCOPE_BUFFER_SIZE) {
            esp_foc_wait_notifier();
        }

        idx = 0;
        next_sample = &scope_buffer[ping_pong_switch][rd_buff_index];
        if(used_channels != 0) {
            for(int i = 0; i < CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS; i++) {
                if(used_channels & (1 << i)) {
                    idx += sprintf(&out_buf[idx],"%f,",next_sample->data[i]);
                }
            }
            out_buf[idx-1] = '\n';
            out_buf[idx] = 0;
        } else {
            /* send at least one channel */
            idx += sprintf(&out_buf[idx],"%f,",next_sample->data[0]);
            out_buf[idx-1] = '\n';
            out_buf[idx] = 0;
        }

        esp_foc_send_buffer_callback((const uint8_t *)out_buf, idx);
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

void esp_foc_scope_data_push(void)
{
    if(!scope_enable) {
        ESP_LOGW(TAG, "ESP FOC scope is not ready yet, skipping...");
        return;
    }

    struct scope_frame *next_sample = &scope_buffer[!ping_pong_switch][wr_buff_index];
    for(int i = 0; i < CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS; i++) {
        if(used_channels & (1 << i)) {
            next_sample->data[i] = *scope_channels[i];
        } else {
            next_sample->data[i] = 0.0f;
        }
    }

    wr_buff_index++;
    if(wr_buff_index >= CONFIG_ESP_FOC_SCOPE_BUFFER_SIZE) {
        rd_buff_index = 0;
        wr_buff_index = 0;
        ping_pong_switch ^= 1;
        esp_foc_send_notification(scope_ev);
    }
}

int esp_foc_scope_add_channel(const float *data_to_wire, int channel_number)
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
    scope_channels[channel_number] = (float *)data_to_wire;
    esp_foc_critical_leave();

    ESP_LOGI(TAG, "Wire data %p is attached to channel %d", data_to_wire, channel_number);
    return 0;
}