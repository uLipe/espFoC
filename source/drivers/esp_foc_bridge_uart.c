/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include <sdkconfig.h>

#if defined(CONFIG_ESP_FOC_BRIDGE_UART)

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "espFoC/esp_foc_tuner.h"
#include "espFoC/esp_foc_bridge_uart.h"

static const char *TAG = "ESPFOC_BRIDGE_UART";

#define UART_NUM           ((uart_port_t)CONFIG_ESP_FOC_BRIDGE_UART_NUM)
#define UART_BAUD          (CONFIG_ESP_FOC_BRIDGE_UART_BAUD)
#define UART_TX_PIN        (CONFIG_ESP_FOC_BRIDGE_UART_TX_PIN)
#define UART_RX_PIN        (CONFIG_ESP_FOC_BRIDGE_UART_RX_PIN)
#define UART_RX_BUFFER     2048
#define UART_TX_BUFFER     1024
#define READER_STACK_BYTES 4096

static volatile bool s_bus_ready = false;

static void reader_task(void *arg)
{
    (void)arg;
    uint8_t buf[256];
    while (1) {
        int n = uart_read_bytes(UART_NUM, buf, sizeof(buf),
                                pdMS_TO_TICKS(50));
        if (n > 0) {
            for (int i = 0; i < n; ++i) {
                esp_foc_tuner_process_byte(buf[i]);
            }
        }
    }
}

void esp_foc_tuner_init_bus_callback(void)
{
    if (s_bus_ready) {
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
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_RX_BUFFER, UART_TX_BUFFER,
                                        0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    esp_foc_tuner_reactor_reset();
    /* Cores fixed at 1; the reader task is light and shouldn't fight the
     * control loop on PRO_CPU. Pinning is left to the user via xTaskCreate
     * if they need a specific layout. */
    BaseType_t ok = xTaskCreate(reader_task, "espfoc_uart_rx",
                                READER_STACK_BYTES, NULL,
                                tskIDLE_PRIORITY + 4, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to spawn UART reader task");
    }
    s_bus_ready = true;
    ESP_LOGI(TAG, "tuner UART bridge ready (UART%d, %d bps, TX=%d, RX=%d)",
             (int)UART_NUM, UART_BAUD, UART_TX_PIN, UART_RX_PIN);
}

int esp_foc_tuner_recv_callback(uint8_t *buf, size_t max)
{
    if (buf == NULL || max == 0) {
        return 0;
    }
    int n = uart_read_bytes(UART_NUM, buf, max, 0);
    return (n < 0) ? 0 : n;
}

void esp_foc_tuner_send_callback(const uint8_t *buf, size_t len)
{
    if (buf == NULL || len == 0) {
        return;
    }
    uart_write_bytes(UART_NUM, (const char *)buf, len);
}

void esp_foc_bridge_uart_link_anchor(void)
{
    /* Reference exported so a single TU pulls the bridge in even when the
     * linker garbage-collects unused sections. */
}

#else  /* CONFIG_ESP_FOC_BRIDGE_UART not set */

void esp_foc_bridge_uart_link_anchor(void) { }

#endif
