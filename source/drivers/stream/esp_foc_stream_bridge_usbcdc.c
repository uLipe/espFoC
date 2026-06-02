/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include <sdkconfig.h>

#if defined(CONFIG_ESP_FOC_STREAM_BRIDGE_USBCDC) && \
    (defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) || \
     defined(CONFIG_IDF_TARGET_ESP32P4))

#include "esp_log.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_cdc_acm.h"
#include "espFoC/stream/esp_foc_stream_bridge.h"

static const char *TAG = "espfoc_stream_usbcdc";

static volatile bool s_ready;

void esp_foc_stream_bridge_init(void)
{
    if (s_ready) {
        return;
    }
    const tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .cdc_port                     = TINYUSB_CDC_ACM_0,
        .callback_rx                  = NULL,
        .callback_rx_wanted_char      = NULL,
        .callback_line_state_changed  = NULL,
        .callback_line_coding_changed = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&acm_cfg));
    s_ready = true;
    ESP_LOGI(TAG, "scope stream on USB-CDC (TX only)");
}

void esp_foc_stream_bridge_send_frame(const uint8_t *data, size_t len)
{
    if (!s_ready || data == NULL || len == 0) {
        return;
    }
    tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, data, len);
    (void)tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
}

#endif
