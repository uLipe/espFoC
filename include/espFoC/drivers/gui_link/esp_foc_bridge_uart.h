/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

/**
 * @file esp_foc_bridge_uart.h
 * @brief UART bridge for the espFoC tuner / scope link.
 *
 * Implements the weak callbacks declared by esp_foc_tuner.h so any
 * espFoC build can talk to the host TunerStudio over a regular UART:
 *
 *   - esp_foc_tuner_init_bus_callback()   -> driver install + RX task
 *   - esp_foc_tuner_recv_callback()       -> shim around uart_read_bytes
 *   - esp_foc_tuner_send_callback()       -> shim around uart_write_bytes
 *
 * Pin/baud configuration lives in Kconfig; the bridge spawns a single
 * RX task that pumps incoming bytes into esp_foc_tuner_process_byte().
 *
 * Compiled into the espFoC component when CONFIG_ESP_FOC_BRIDGE_UART
 * is set. Mutually exclusive with the USB-CDC bridge.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* The bridge installs itself automatically the first time
 * esp_foc_tuner_init_bus_callback() is invoked, so no public symbol is
 * exported here beyond the weak overrides defined in the C file. This
 * header exists so user code can include it explicitly to force the
 * bridge to be linked in even when -fdata-sections is in play. */
void esp_foc_bridge_uart_link_anchor(void);

#ifdef __cplusplus
}
#endif
