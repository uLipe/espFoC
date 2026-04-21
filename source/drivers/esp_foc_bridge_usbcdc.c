/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include <sdkconfig.h>

#if defined(CONFIG_ESP_FOC_BRIDGE_USBCDC)

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_cdc_acm.h"
#include "espFoC/esp_foc_link.h"
#include "espFoC/esp_foc_tuner.h"
#include "espFoC/esp_foc_bridge_usbcdc.h"

static const char *TAG = "ESPFOC_BRIDGE_USB";
#define READER_STACK_BYTES 4096

static volatile bool s_bus_ready = false;

/* TinyUSB delivers RX data from its own task via this callback. We forward
 * each byte to the tuner reactor; the work is small enough to be done in
 * place without bouncing through another queue. */
static void rx_cb(int itf, cdcacm_event_t *event)
{
    (void)event;
    uint8_t buf[64];
    size_t n = 0;
    if (tinyusb_cdcacm_read((tinyusb_cdcacm_itf_t)itf, buf, sizeof(buf), &n) != ESP_OK) {
        return;
    }
    for (size_t i = 0; i < n; ++i) {
        esp_foc_tuner_process_byte(buf[i]);
    }
}

static void line_state_cb(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rts = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "CDC line state: itf=%d DTR=%d RTS=%d", itf, dtr, rts);
}

void esp_foc_tuner_init_bus_callback(void)
{
    if (s_bus_ready) {
        return;
    }
    const tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .cdc_port                     = TINYUSB_CDC_ACM_0,
        .callback_rx                  = &rx_cb,
        .callback_rx_wanted_char      = NULL,
        .callback_line_state_changed  = NULL,
        .callback_line_coding_changed = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&acm_cfg));
    ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
                        TINYUSB_CDC_ACM_0,
                        CDC_EVENT_LINE_STATE_CHANGED,
                        &line_state_cb));

    esp_foc_tuner_reactor_reset();
    s_bus_ready = true;
    ESP_LOGI(TAG, "tuner USB-CDC bridge ready");
}

int esp_foc_tuner_recv_callback(uint8_t *buf, size_t max)
{
    /* All ingress traffic is pushed by TinyUSB through rx_cb already; this
     * shim exists only to satisfy the public API for callers that prefer
     * polling over the push model. */
    if (buf == NULL || max == 0) {
        return 0;
    }
    size_t n = 0;
    if (tinyusb_cdcacm_read(TINYUSB_CDC_ACM_0, buf, max, &n) != ESP_OK) {
        return 0;
    }
    return (int)n;
}

void esp_foc_tuner_send_callback(const uint8_t *buf, size_t len)
{
    if (buf == NULL || len == 0) {
        return;
    }
    tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, buf, len);
    /* Short timeout: we never block the reactor for long; if the host is
     * slow to drain its endpoint we drop newer bytes rather than stall. */
    (void)tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, pdMS_TO_TICKS(20));
}

#if defined(CONFIG_ESP_FOC_SCOPE)
/* Mirror of the UART bridge: wrap scope CSV in a SCOPE-channel link
 * frame so the host sees both streams demuxed on the same USB pipe. */
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
    if ((size_t)size > ESP_FOC_LINK_MAX_PAYLOAD) {
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
    tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, frame, frame_len);
    (void)tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, pdMS_TO_TICKS(20));
}
#endif

void esp_foc_bridge_usbcdc_link_anchor(void) { }

#else /* CONFIG_ESP_FOC_BRIDGE_USBCDC not set */

void esp_foc_bridge_usbcdc_link_anchor(void) { }

#endif
