/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include "esp_log.h"
#include "driver/usb_serial_jtag.h"
#include "espFoC/osal/os_interface.h"
#include "espFoC/stream/esp_foc_stream_bridge.h"

static const char *TAG = "espfoc_stream_usj";

#define USJ_TX_BUFFER 4096

static volatile bool s_ready;
static volatile bool s_logged_first_tx;
static uint32_t s_tx_frames;
static uint32_t s_tx_drops;

static void usj_write_all(const uint8_t *buf, size_t len)
{
    if (buf == NULL || len == 0) {
        return;
    }
    size_t off = 0;
    const TickType_t chunk_wait = esp_foc_ms_to_wait_ticks(20);
    while (off < len) {
        int w = usb_serial_jtag_write_bytes(buf + off, len - off, chunk_wait);
        if (w <= 0) {
            s_tx_drops++;
            break;
        }
        off += (size_t)w;
    }
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
    if (!s_logged_first_tx) {
        s_logged_first_tx = true;
        ESP_LOGI(TAG, "first ESPF frame TX (%u bytes) — connect espFoC Tool on ttyACM*",
                 (unsigned)len);
    }
    usj_write_all(data, len);
    s_tx_frames++;
    if ((s_tx_frames & 0x3FFU) == 0U) {
        ESP_LOGI(TAG, "scope TX: frames=%lu drops=%lu",
                 (unsigned long)s_tx_frames, (unsigned long)s_tx_drops);
    }
}
