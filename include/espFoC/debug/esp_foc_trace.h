/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Sentinel / event class for IRAM binary hot-path tracing.
 *
 * Timing convention (when stamped by the inverter / ADC ISR):
 * - TEZ_EXIT / DMA_EOF: a = duration_us, b = duration_cpu_cycles
 */
typedef enum {
    ESP_FOC_TRACE_TEZ_ENTER = 0x1001,
    ESP_FOC_TRACE_TEZ_EXIT  = 0x1002,
    ESP_FOC_TRACE_DUTY      = 0x1003,
    ESP_FOC_TRACE_DMA_EOF   = 0x1004,
    ESP_FOC_TRACE_SAMPLE_RD = 0x1005,
    ESP_FOC_TRACE_USER_MARK = 0x1006,

    /* Fault / protection class 0x11xx */
    ESP_FOC_TRACE_FAULT_ILIMIT_HIT = 0x1101,
    ESP_FOC_TRACE_FAULT_SOFT_REQ   = 0x1102,
    ESP_FOC_TRACE_FAULT_GPIO_IRQ   = 0x1103,
    ESP_FOC_TRACE_FAULT_TRIP       = 0x1104,
    ESP_FOC_TRACE_FAULT_TRIP_IGN   = 0x1105,
    ESP_FOC_TRACE_FAULT_OST        = 0x1106,
    ESP_FOC_TRACE_FAULT_EN_OFF     = 0x1107,
    ESP_FOC_TRACE_FAULT_CB         = 0x1108,
    ESP_FOC_TRACE_FAULT_DUTY_IGN   = 0x1109,
    ESP_FOC_TRACE_FAULT_CLEAR_REQ  = 0x110A,
    ESP_FOC_TRACE_FAULT_CLEAR_OK   = 0x110B,
    ESP_FOC_TRACE_FAULT_CLEAR_REJ  = 0x110C,
    ESP_FOC_TRACE_FAULT_ENABLE_REJ = 0x110D,
} esp_foc_trace_type_t;

typedef struct {
    uint16_t magic;     /* 0xE5F0 */
    uint16_t type;      /* esp_foc_trace_type_t */
    uint32_t seq;
    int32_t  a;
    int32_t  b;
} esp_foc_trace_rec_t;

#define ESP_FOC_TRACE_MAGIC ((uint16_t)0xE5F0)

esp_err_t esp_foc_trace_init(void);
void esp_foc_trace_reset(void);

/** ISR-safe O(1) push. No-op if trace disabled or not initialized. */
void esp_foc_trace_push(uint16_t type, int32_t a, int32_t b);

/**
 * Copy up to max_out records into out (oldest first among available).
 * Returns number of records copied. Safe from task context.
 */
size_t esp_foc_trace_snapshot(esp_foc_trace_rec_t *out, size_t max_out);

uint32_t esp_foc_trace_dropped(void);
uint32_t esp_foc_trace_count(void);

#ifdef __cplusplus
}
#endif
