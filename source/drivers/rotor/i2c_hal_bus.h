/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Private I2C master bus helper — HAL/LL only (no driver/i2c.h).
 * Sync write/write_read: two phases from task, poll `done` (init/calib).
 * Runtime read: current-address only; task event-waits; ISR copies RX + posts.
 * Async write_read: kick once; ISR chains A→B; one done callback.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "hal/i2c_hal.h"
#include "espFoC/osal/esp_foc_osal.h"

typedef void (*esp_foc_i2c_hal_done_cb_t)(void *arg, esp_err_t err);

typedef enum {
    ESP_FOC_I2C_PHASE_IDLE = 0,
    ESP_FOC_I2C_PHASE_A,
    ESP_FOC_I2C_PHASE_A_DONE, /* async: END seen; task must start phase B */
    ESP_FOC_I2C_PHASE_B,
} esp_foc_i2c_phase_t;

typedef struct {
    bool inited;
    int port;
    int sda;
    int scl;
    uint32_t hz;
    i2c_hal_context_t hal;
    intr_handle_t intr;

    volatile esp_foc_i2c_phase_t phase;
    volatile bool busy;
    volatile bool done;
    volatile bool async_chain;
    volatile bool need_clear;   /* ISR saw NACK/timeout; next kick clocks the bus free */
    volatile esp_err_t last_err;
    volatile uint32_t last_st;
    volatile uint64_t busy_since_us;
    esp_foc_event_handle_t waiter;

    uint8_t addr7;
    uint8_t wr[8];
    size_t wr_len;
    uint8_t *rd;
    size_t rd_len;
    esp_foc_i2c_hal_done_cb_t cb;
    void *cb_arg;
} esp_foc_i2c_hal_bus_t;

esp_err_t esp_foc_i2c_hal_bus_init(esp_foc_i2c_hal_bus_t *bus,
                                   int port,
                                   int sda,
                                   int scl,
                                   uint32_t hz);
void esp_foc_i2c_hal_bus_deinit(esp_foc_i2c_hal_bus_t *bus);

bool esp_foc_i2c_hal_busy(const esp_foc_i2c_hal_bus_t *bus);

/** Abort in-flight xfer (FSM reset). Safe from task context. */
void esp_foc_i2c_hal_recover(esp_foc_i2c_hal_bus_t *bus);

/**
 * Non-blocking write-then-read, or current-address read when wr_len == 0
 * (START+R … NACK+STOP, no register write). wr_len > 0 still uses the C6
 * END split; the ISR starts phase B. Final `cb` runs once from the I2C ISR.
 */
esp_err_t esp_foc_i2c_hal_write_read_async(esp_foc_i2c_hal_bus_t *bus,
                                           uint8_t addr7,
                                           const uint8_t *wr,
                                           size_t wr_len,
                                           uint8_t *rd,
                                           size_t rd_len,
                                           esp_foc_i2c_hal_done_cb_t cb,
                                           void *cb_arg);

/**
 * Blocking current-address read (START+R … NACK+STOP). Task programs HW and
 * event-waits; I2C ISR copies RX and posts. No sleep, no phase-A write.
 */
esp_err_t esp_foc_i2c_hal_read(esp_foc_i2c_hal_bus_t *bus,
                               uint8_t addr7,
                               uint8_t *rd,
                               size_t rd_len);

/** Blocking write-read for init/calib. Polls done; does not use task notify. */
esp_err_t esp_foc_i2c_hal_write_read(esp_foc_i2c_hal_bus_t *bus,
                                     uint8_t addr7,
                                     const uint8_t *wr,
                                     size_t wr_len,
                                     uint8_t *rd,
                                     size_t rd_len);

/** Blocking write-only (register programming at init). */
esp_err_t esp_foc_i2c_hal_write(esp_foc_i2c_hal_bus_t *bus,
                                uint8_t addr7,
                                const uint8_t *wr,
                                size_t wr_len);
