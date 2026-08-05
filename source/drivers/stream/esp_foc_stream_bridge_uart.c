/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include <sdkconfig.h>

#if defined(CONFIG_ESP_FOC_STREAM_BRIDGE_UART)

#include "esp_log.h"
#include "driver/uart.h"
#include "espFoC/stream/esp_foc_stream_bridge.h"

static const char *TAG = "espfoc_stream_uart";

#define UART_NUM    ((uart_port_t)CONFIG_ESP_FOC_STREAM_UART_NUM)
#define UART_BAUD   (CONFIG_ESP_FOC_STREAM_UART_BAUD)

static volatile bool s_ready;

void esp_foc_stream_bridge_init(void)
{
    if (s_ready) {
        return;
    }
    const uart_config_t cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, 256, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM,
                                 CONFIG_ESP_FOC_STREAM_UART_TX_PIN,
                                 CONFIG_ESP_FOC_STREAM_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    s_ready = true;
    ESP_LOGI(TAG, "scope stream UART%d %d bps TX=%d",
             (int)UART_NUM, UART_BAUD, CONFIG_ESP_FOC_STREAM_UART_TX_PIN);
}

void esp_foc_stream_bridge_send_frame(const uint8_t *data, size_t len)
{
    if (!s_ready || data == NULL || len == 0) {
        return;
    }
    uart_write_bytes(UART_NUM, (const char *)data, len);
}

#endif
