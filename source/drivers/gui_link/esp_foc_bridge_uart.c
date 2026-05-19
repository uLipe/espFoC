/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include <sdkconfig.h>

#if defined(CONFIG_ESP_FOC_BRIDGE_UART)

#include "esp_log.h"
#include "driver/uart.h"
#include "espFoC/osal/os_interface.h"
#include "espFoC/gui_link/esp_foc_link.h"
#include "espFoC/gui_link/esp_foc_tuner.h"
#include "espFoC/gui_link/esp_foc_link_session.h"
#include "espFoC/drivers/gui_link/esp_foc_bridge_uart.h"

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
                                esp_foc_ms_to_wait_ticks(50));
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
    /* Reader task priority is low (idle+4) so it does not contend with the FOC runners. */
    if (esp_foc_task_spawn(reader_task, NULL, READER_STACK_BYTES, 4, NULL) != 0) {
        ESP_LOGE(TAG, "failed to spawn UART reader task");
    }
    s_bus_ready = true;
    esp_foc_link_session_start();
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

#if defined(CONFIG_ESP_FOC_SCOPE)
/* Shared code path with USB-CDC: wrap the scope's CSV payload in a
 * SCOPE-channel link frame so the host can demux it alongside tuner
 * responses on the same bus. seq rolls independently of the tuner. */
static uint8_t s_scope_seq = 0;

void esp_foc_init_bus_callback(void)
{
    /* Make sure the tuner path has initialised the UART. Idempotent. */
    esp_foc_tuner_init_bus_callback();
}

void esp_foc_send_buffer_callback(const uint8_t *buffer, int size)
{
    if (buffer == NULL || size <= 0) {
        return;
    }
    if (!esp_foc_link_session_scope_streaming()) {
        return;
    }
    if (!esp_foc_link_try_acquire_tx_low_prio()) {
        return;
    }
    if ((size_t)size > ESP_FOC_LINK_MAX_PAYLOAD) {
        /* Scope CSV overflowed the link MTU — drop the sample rather
         * than risk splitting a CSV line across two frames. */
        ESP_LOGW(TAG, "scope CSV %d > max %u, dropped",
                 size, (unsigned)ESP_FOC_LINK_MAX_PAYLOAD);
        return;
    }
    uint8_t frame[ESP_FOC_LINK_MAX_FRAME];
    size_t frame_len = 0;
    esp_foc_link_status_t st = esp_foc_link_encode(
        ESP_FOC_LINK_CH_SCOPE, s_scope_seq++, buffer, (size_t)size,
        frame, sizeof(frame), &frame_len);
    if (st != ESP_FOC_LINK_OK) {
        return;
    }
    uart_write_bytes(UART_NUM, (const char *)frame, frame_len);
}
#endif

void esp_foc_bridge_uart_link_anchor(void)
{
    /* Reference exported so a single TU pulls the bridge in even when the
     * linker garbage-collects unused sections. */
}

#else  /* CONFIG_ESP_FOC_BRIDGE_UART not set */

void esp_foc_bridge_uart_link_anchor(void) { }

#endif
