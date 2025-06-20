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
#include <sdkconfig.h>
#include "espFoC/esp_foc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_attr.h"
#include "esp_log.h"

static bool scope_enable = false;
static esp_foc_control_data_t scope_buffer[2][CONFIG_ESP_FOC_SCOPE_BUFFER_SIZE];
static bool ping_pong_switch = false;
static uint32_t rd_buff_index = 0;
static uint32_t wr_buff_index = 0;
static char out_buf[2 * sizeof(esp_foc_control_data_t)];

IRAM_ATTR static void esp_foc_scope_daemon_thread(void *arg)
{
    esp_foc_control_data_t *next_sample;
    memset(&scope_buffer, 0, sizeof(scope_buffer));

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(UART_NUM_2, 2 * sizeof(esp_foc_control_data_t), 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, CONFIG_ESP_FOC_SCOPE_TXD_PIN, CONFIG_ESP_FOC_SCOPE_RXD_PIN, -1, -1);

    while(1) {
        next_sample = &scope_buffer[ping_pong_switch][rd_buff_index];
        sprintf(out_buf, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",
            next_sample->dt.raw,
            next_sample->u_alpha.raw,
            next_sample->u_beta.raw,
            next_sample->w.raw,
            next_sample->i_alpha.raw,
            next_sample->i_beta.raw,
            next_sample->i_w.raw,
            next_sample->i_q.raw,
            next_sample->observer_angle.raw,
            next_sample->rotor_position.raw,
            next_sample->speed.raw);
        rd_buff_index++;

        uart_write_bytes(UART_NUM_2, (const char *) out_buf, strlen(out_buf));

        if(rd_buff_index >= CONFIG_ESP_FOC_SCOPE_BUFFER_SIZE) {
            esp_foc_critical_enter();
            rd_buff_index = 0;
            wr_buff_index = 0;
            ping_pong_switch ^= 1;
            esp_foc_critical_leave();
        }

    }
}

void esp_foc_scope_initalize(void)
{
    if(!scope_enable) {
        scope_enable = true;
        esp_foc_create_runner(esp_foc_scope_daemon_thread, NULL, 4);
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