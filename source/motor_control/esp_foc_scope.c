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

static bool scope_enable = false;
static esp_foc_control_data_t scope_buffer[2][CONFIG_ESP_FOC_SCOPE_BUFFER_SIZE];
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
    char out_buf[512] = {0,};
    esp_foc_control_data_t *next_sample;
    memset(&scope_buffer, 0, sizeof(scope_buffer));

    while(1) {

        next_sample = &scope_buffer[ping_pong_switch][rd_buff_index];

            // control_data->u_alpha = axis->u_alpha;
        // control_data->u_beta = axis->u_beta;

        // control_data->i_alpha = axis->i_alpha;
        // control_data->i_beta = axis->i_beta;

        // control_data->u = axis->u_u;
        // control_data->v = axis->u_v;
        // control_data->w = axis->u_w;

        // control_data->out_q = axis->u_q;
        // control_data->out_d = axis->u_d;
        // control_data->dt.raw = axis->dt;

        // control_data->speed.raw = axis->current_speed;
        // control_data->target_position.raw = axis->target_position;
        // control_data->target_speed.raw = axis->target_speed;

        // control_data->rotor_position.raw =  axis->open_loop_observer->get_angle(axis->open_loop_observer);
        // control_data->observer_angle.raw = axis->current_observer->get_angle(axis->current_observer);
        // control_data->rotor_position.raw =  axis->rotor_position;
        // control_data->extrapolated_rotor_position.raw =  axis->extrapolated_rotor_position;
        // control_data->observer_angle.raw = 0.0f;

        // control_data->i_u.raw = axis->i_u;
        // control_data->i_v.raw = axis->i_v;
        // control_data->i_w.raw = axis->i_w;

        // control_data->i_q = axis->i_q;
        // control_data->i_d = axis->i_d;


        sprintf(out_buf, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",
            next_sample->dt.raw,
            next_sample->u_alpha.raw,
            next_sample->u_beta.raw,
            next_sample->i_alpha.raw,
            next_sample->i_beta.raw,
            next_sample->i_d.raw,
            next_sample->i_q.raw,
            next_sample->observer_angle.raw,
            next_sample->rotor_position.raw,
            next_sample->target_position.raw,
            next_sample->speed.raw,
            next_sample->target_speed.raw
        );

        rd_buff_index++;
        if(rd_buff_index >= CONFIG_ESP_FOC_SCOPE_BUFFER_SIZE) {
            esp_foc_critical_enter();
            rd_buff_index = 0;
            wr_buff_index = 0;
            ping_pong_switch ^= 1;
            esp_foc_critical_leave();
        }

        esp_foc_send_buffer_callback((const uint8_t *)out_buf, strlen(out_buf) + 1);
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

void esp_foc_scope_data_push(esp_foc_control_data_t *control_data)
{
    if(!control_data || wr_buff_index >= CONFIG_ESP_FOC_SCOPE_BUFFER_SIZE) {
        return;
    }

    esp_foc_critical_enter();

    memcpy(&scope_buffer[!ping_pong_switch][wr_buff_index],
        control_data,
        sizeof(*control_data));

    wr_buff_index++;

    esp_foc_critical_leave();
}