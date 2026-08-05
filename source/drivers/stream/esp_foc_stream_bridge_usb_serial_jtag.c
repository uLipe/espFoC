/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include "esp_log.h"
#include "driver/usb_serial_jtag.h"
#include "espFoC/stream/esp_foc_stream_bridge.h"

static const char *TAG = "espfoc_stream_usj";

#define USJ_TX_BUFFER 8192

static volatile bool s_ready;

static bool usj_try_write(const uint8_t *buf, size_t len)
{
    size_t off = 0;

    if (buf == NULL || len == 0) {
        return false;
    }
    while (off < len) {
        int w = usb_serial_jtag_write_bytes(buf + off, len - off, 0);
        if (w <= 0) {
            return false;
        }
        off += (size_t)w;
    }
    return true;
}

void esp_foc_stream_bridge_init(void)
{
    if (s_ready) {
        return;
    }
    if (!usb_serial_jtag_is_driver_installed()) {
        const usb_serial_jtag_driver_config_t cfg = {
            .tx_buffer_size = USJ_TX_BUFFER,
            .rx_buffer_size = 256,
        };
        ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));
    }
    s_ready = true;
    ESP_LOGI(TAG, "scope stream on USB Serial/JTAG (TX only, ttyACM*)");
}

void esp_foc_stream_bridge_send_frame(const uint8_t *data, size_t len)
{
    if (!s_ready || data == NULL || len == 0) {
        return;
    }
    (void)usj_try_write(data, len);
}
