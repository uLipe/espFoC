/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include <sdkconfig.h>

#if defined(CONFIG_ESP_FOC_BRIDGE_USBCDC)

#include "esp_log.h"
#include "driver/usb_serial_jtag.h"
#include "espFoC/osal/os_interface.h"
#include "espFoC/gui_link/esp_foc_link.h"
#include "espFoC/gui_link/esp_foc_tuner.h"
#include "espFoC/gui_link/esp_foc_link_session.h"
#include "espFoC/drivers/gui_link/esp_foc_bridge_usbcdc.h"

static const char *TAG = "ESPFOC_BRIDGE_USJ";

#define USJ_RX_BUFFER      2048
#define USJ_TX_BUFFER      1024
#define READER_STACK_BYTES 4096

static volatile bool s_bus_ready = false;

static void reader_task(void *arg)
{
    (void)arg;
    uint8_t buf[256];
    while (1) {
        int n = usb_serial_jtag_read_bytes(buf, sizeof(buf),
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
    if (!usb_serial_jtag_is_driver_installed()) {
        usb_serial_jtag_driver_config_t cfg = {
            .tx_buffer_size = USJ_TX_BUFFER,
            .rx_buffer_size = USJ_RX_BUFFER,
        };
        ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));
    } else {
        ESP_LOGW(TAG,
                 "USB Serial/JTAG driver already installed; tuner shares it with console");
    }
    esp_foc_tuner_reactor_reset();
    if (esp_foc_task_spawn(reader_task, NULL, READER_STACK_BYTES, 4, NULL) != 0) {
        ESP_LOGE(TAG, "failed to spawn USB Serial/JTAG reader task");
    }
    s_bus_ready = true;
    esp_foc_link_session_start();
    ESP_LOGI(TAG, "tuner USB Serial/JTAG bridge ready");
}

int esp_foc_tuner_recv_callback(uint8_t *buf, size_t max)
{
    if (buf == NULL || max == 0) {
        return 0;
    }
    int n = usb_serial_jtag_read_bytes(buf, max, 0);
    return (n < 0) ? 0 : n;
}

void esp_foc_tuner_send_callback(const uint8_t *buf, size_t len)
{
    if (buf == NULL || len == 0) {
        return;
    }
    (void)usb_serial_jtag_write_bytes(buf, len, 0);
}

#if defined(CONFIG_ESP_FOC_SCOPE)
static uint8_t s_scope_seq = 0;

void esp_foc_init_bus_callback(void)
{
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
        ESP_LOGW(TAG, "scope frame %d > max %u, dropped",
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
    (void)usb_serial_jtag_write_bytes(frame, frame_len, 0);
}
#endif

void esp_foc_bridge_usbcdc_link_anchor(void) { }

#else /* CONFIG_ESP_FOC_BRIDGE_USBCDC not set */

void esp_foc_bridge_usbcdc_link_anchor(void) { }

#endif
