/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

/**
 * @file esp_foc_bridge_usbcdc.h
 * @brief USB CDC bridge for the espFoC tuner / scope link.
 *
 * Implements the weak callbacks declared by esp_foc_tuner.h. Backend is
 * selected at build time:
 * - ESP32-S2/S3/P4: TinyUSB CDC-ACM (espressif/esp_tinyusb)
 * - ESP32-C3/C5/C6/H2/C61: USB Serial/JTAG (esp_driver_usb_serial_jtag)
 *
 * Compiled when CONFIG_ESP_FOC_BRIDGE_USBCDC is set.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void esp_foc_bridge_usbcdc_link_anchor(void);

#ifdef __cplusplus
}
#endif
