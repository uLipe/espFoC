/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "espFoC/drivers/esp_foc_rotor_hall.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Where a hall edge's timestamp comes from, isolated behind three calls so
 * that the estimator and the rotor-sensor interface never learn which strategy
 * is in use.
 *
 * ETM_TIMG latches the instant in hardware, so ISR jitter cannot reach it, and
 * its hot path is two register reads with zero writes. GPIO_IRQ reads a
 * microsecond clock at ISR entry instead, which is cheaper to bring up and
 * measures the edge against the wrong reference. The boundary exists so that
 * swapping them costs one config field.
 */
typedef struct {
    esp_foc_hall_ts_kind_t kind;
    int gpio[3];
    /* ETM_TIMG only: channel indices come from the app, as etm_link's do. */
    int etm_channel[3];
    int timer_group;
    /*
     * ETM_TIMG only. See the init-order contract in esp_foc_rotor_hall.h:
     * true refuses to come up when nothing has enabled the ETM bus clock yet.
     */
    bool require_etm_ready;
    /* GPIO_IRQ only. Must stay below the PWM ISR's level. */
    int irq_level;
} esp_foc_hall_ts_cfg_t;

typedef struct esp_foc_hall_ts_s esp_foc_hall_ts_t;

typedef struct {
    esp_err_t (*init)(esp_foc_hall_ts_t *ts);
    /** True when a new edge was captured; *ticks is its timestamp. */
    bool (*poll)(esp_foc_hall_ts_t *ts, uint64_t *ticks);
    /** Reprogram whatever the strategy owns, after someone else clobbered it. */
    esp_err_t (*rearm)(esp_foc_hall_ts_t *ts);
    /** Still programmed and responding? */
    bool (*healthy)(const esp_foc_hall_ts_t *ts);
    void (*deinit)(esp_foc_hall_ts_t *ts);
} esp_foc_hall_ts_ops_t;

struct esp_foc_hall_ts_s {
    const esp_foc_hall_ts_ops_t *ops;
    esp_foc_hall_ts_cfg_t cfg;
    uint32_t tick_hz;
    bool inited;
    /* ETM_TIMG. */
    void *timg;
    uint64_t last_cap;
    bool etm_cold_start;
    /* GPIO_IRQ: the ISR publishes, poll() consumes by sequence number. */
    volatile uint64_t irq_ticks;
    volatile uint32_t irq_seq;
    uint32_t seen_seq;
    void *isr_handle;
};

esp_err_t esp_foc_hall_ts_init(esp_foc_hall_ts_t *ts, const esp_foc_hall_ts_cfg_t *cfg);
void esp_foc_hall_ts_deinit(esp_foc_hall_ts_t *ts);

/** Hot path. */
static inline bool esp_foc_hall_ts_poll(esp_foc_hall_ts_t *ts, uint64_t *ticks)
{
    return ts->ops->poll(ts, ticks);
}

static inline uint32_t esp_foc_hall_ts_tick_hz(const esp_foc_hall_ts_t *ts)
{
    return ts->tick_hz;
}

static inline esp_err_t esp_foc_hall_ts_rearm(esp_foc_hall_ts_t *ts)
{
    if (ts == NULL || !ts->inited) {
        return ESP_ERR_INVALID_STATE;
    }
    return ts->ops->rearm(ts);
}

static inline bool esp_foc_hall_ts_healthy(const esp_foc_hall_ts_t *ts)
{
    return ts != NULL && ts->inited && ts->ops->healthy(ts);
}

static inline bool esp_foc_hall_ts_etm_cold_start(const esp_foc_hall_ts_t *ts)
{
    return ts != NULL && ts->etm_cold_start;
}

/* Strategy tables, defined by their own translation units. */
extern const esp_foc_hall_ts_ops_t esp_foc_hall_ts_etm_timg_ops;
extern const esp_foc_hall_ts_ops_t esp_foc_hall_ts_gpio_irq_ops;

/** Shared by both strategies: open-drain released + 3.3 V pull-up. */
void esp_foc_hall_ts_gpio_as_input(int gpio);

#ifdef __cplusplus
}
#endif
