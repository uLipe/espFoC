/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

/**
 * @file esp_foc_bridge_usbcdc.h
 * @brief TinyUSB CDC-ACM bridge for the espFoC tuner / scope link.
 *
 * Implements the weak callbacks declared by esp_foc_tuner.h on top of
 * the espressif/esp_tinyusb managed component. The same physical USB
 * cable used to flash the board carries the tuner traffic — no extra
 * adapter required for ESP32-S2/S3/P4.
 *
 * Compiled into the espFoC component when CONFIG_ESP_FOC_BRIDGE_USBCDC
 * is set.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void esp_foc_bridge_usbcdc_link_anchor(void);

#ifdef __cplusplus
}
#endif
