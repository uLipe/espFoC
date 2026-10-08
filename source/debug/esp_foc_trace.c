/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#include "sdkconfig.h"
#include "espFoC/debug/esp_foc_trace.h"

#include <stdbool.h>
#include <string.h>

#if CONFIG_ESP_FOC_TRACE_ENABLE

#ifndef CONFIG_ESP_FOC_TRACE_DEPTH
#define CONFIG_ESP_FOC_TRACE_DEPTH 256
#endif

#define TRACE_DEPTH ((size_t)CONFIG_ESP_FOC_TRACE_DEPTH)
#define TRACE_MASK  (TRACE_DEPTH - 1u)

#if (CONFIG_ESP_FOC_TRACE_DEPTH & (CONFIG_ESP_FOC_TRACE_DEPTH - 1)) != 0
#error "CONFIG_ESP_FOC_TRACE_DEPTH must be a power of two"
#endif

static esp_foc_trace_rec_t s_ring[TRACE_DEPTH];
static volatile uint32_t s_head;
static volatile uint32_t s_seq;
static volatile uint32_t s_dropped;
static volatile uint32_t s_count;
static bool s_inited;

esp_err_t esp_foc_trace_init(void)
{
    memset(s_ring, 0, sizeof(s_ring));
    s_head = 0;
    s_seq = 0;
    s_dropped = 0;
    s_count = 0;
    s_inited = true;
    return ESP_OK;
}

void esp_foc_trace_reset(void)
{
    s_head = 0;
    s_seq = 0;
    s_dropped = 0;
    s_count = 0;
}

void esp_foc_trace_push(uint16_t type, int32_t a, int32_t b)
{
    if (!s_inited) {
        return;
    }
    uint32_t idx = s_head;
    s_head = (idx + 1u) & TRACE_MASK;
    if (s_count < TRACE_DEPTH) {
        s_count++;
    } else {
        s_dropped++;
    }
    esp_foc_trace_rec_t *r = &s_ring[idx & TRACE_MASK];
    r->magic = ESP_FOC_TRACE_MAGIC;
    r->type = type;
    r->seq = ++s_seq;
    r->a = a;
    r->b = b;
}

size_t esp_foc_trace_snapshot(esp_foc_trace_rec_t *out, size_t max_out)
{
    if (out == NULL || max_out == 0 || !s_inited) {
        return 0;
    }
    uint32_t n = s_count;
    if (n > max_out) {
        n = (uint32_t)max_out;
    }
    uint32_t head = s_head;
    uint32_t start = (head - n) & TRACE_MASK;
    for (uint32_t i = 0; i < n; i++) {
        out[i] = s_ring[(start + i) & TRACE_MASK];
    }
    return (size_t)n;
}

uint32_t esp_foc_trace_dropped(void)
{
    return s_dropped;
}

uint32_t esp_foc_trace_count(void)
{
    return s_count;
}

#else /* !CONFIG_ESP_FOC_TRACE_ENABLE */

esp_err_t esp_foc_trace_init(void)
{
    return ESP_OK;
}

void esp_foc_trace_reset(void)
{
}

void esp_foc_trace_push(uint16_t type, int32_t a, int32_t b)
{
    (void)type;
    (void)a;
    (void)b;
}

size_t esp_foc_trace_snapshot(esp_foc_trace_rec_t *out, size_t max_out)
{
    (void)out;
    (void)max_out;
    return 0;
}

uint32_t esp_foc_trace_dropped(void)
{
    return 0;
}

uint32_t esp_foc_trace_count(void)
{
    return 0;
}

#endif
