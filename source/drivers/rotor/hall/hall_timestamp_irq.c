/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Fallback timestamp strategy: a GPIO interrupt below the PWM ISR's level
 * stamps the edge with the OSAL microsecond clock.
 *
 * Same measurement, worse clock. What ETM latches in hardware this reads at
 * ISR entry, so the error is the entry jitter — with a ~20 µs PWM ISR every
 * 50 µs that can reach ~25 µs, i.e. 4% of a 625 µs sector. λ_ω attenuates it,
 * but it is bias from the wrong reference and not noise. It exists so that a
 * surprise in ETM or the timer group on silicon costs one config field instead
 * of a redesign: the estimator and the rotor-sensor interface never find out.
 */
#include "hall_timestamp.h"

#include "esp_check.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"
#include "soc/interrupts.h"

#include "espFoC/osal/esp_foc_osal.h"

static const char *TAG = "foc_hall_irq";

#define IRQ_TICK_HZ 1000000u

static void hall_gpio_isr(void *arg)
{
    esp_foc_hall_ts_t *ts = (esp_foc_hall_ts_t *)arg;

    uint32_t status = 0;
    gpio_ll_get_intr_status(&GPIO, 0, &status);

    uint32_t mine = 0;
    for (int i = 0; i < 3; i++) {
        int g = ts->cfg.gpio[i];
        if (g < 32) {
            mine |= (1u << g);
        }
    }
    uint32_t hit = status & mine;
    if (hit == 0u) {
        return;
    }
    gpio_ll_clear_intr_status(&GPIO, hit);

    /*
     * Stamp first, publish second. poll() reads the sequence number to decide
     * there is a new edge, so the timestamp has to be in place before the
     * sequence advertises it.
     */
    ts->irq_ticks = esp_foc_now_us();
    ts->irq_seq++;
}

static esp_err_t irq_init(esp_foc_hall_ts_t *ts)
{
    const esp_foc_hall_ts_cfg_t *cfg = &ts->cfg;

    int level = (cfg->irq_level > 0) ? cfg->irq_level : 1;
    ESP_RETURN_ON_FALSE(level >= 1 && level <= 3, ESP_ERR_INVALID_ARG, TAG, "level");

    ts->tick_hz = IRQ_TICK_HZ;
    ts->irq_seq = 0;
    ts->seen_seq = 0;
    ts->irq_ticks = 0;

    for (int i = 0; i < 3; i++) {
        gpio_ll_set_intr_type(&GPIO, (uint32_t)cfg->gpio[i], GPIO_INTR_ANYEDGE);
        gpio_ll_clear_intr_status(&GPIO, 1u << cfg->gpio[i]);
    }

    int flags = ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_SHARED;
    flags |= (level == 3) ? ESP_INTR_FLAG_LEVEL3
                          : ((level == 2) ? ESP_INTR_FLAG_LEVEL2 : ESP_INTR_FLAG_LEVEL1);

    intr_handle_t handle = NULL;
    esp_err_t err = esp_intr_alloc(ETS_GPIO_INTR_SOURCE, flags, hall_gpio_isr, ts, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_intr_alloc: %s", esp_err_to_name(err));
        return err;
    }
    ts->isr_handle = handle;

    for (int i = 0; i < 3; i++) {
        gpio_ll_intr_enable_on_core(&GPIO, 0, (uint32_t)cfg->gpio[i]);
    }

    ESP_LOGI(TAG, "hall ts GPIO IRQ: gpio=%d/%d/%d level=%d tick=%luHz",
             cfg->gpio[0], cfg->gpio[1], cfg->gpio[2], level,
             (unsigned long)ts->tick_hz);
    return ESP_OK;
}

static bool irq_poll(esp_foc_hall_ts_t *ts, uint64_t *ticks)
{
    uint32_t seq = ts->irq_seq;
    if (seq == ts->seen_seq) {
        return false;
    }
    ts->seen_seq = seq;
    *ticks = ts->irq_ticks;
    return true;
}

static esp_err_t irq_rearm(esp_foc_hall_ts_t *ts)
{
    for (int i = 0; i < 3; i++) {
        gpio_ll_set_intr_type(&GPIO, (uint32_t)ts->cfg.gpio[i], GPIO_INTR_ANYEDGE);
        gpio_ll_intr_enable_on_core(&GPIO, 0, (uint32_t)ts->cfg.gpio[i]);
    }
    return ESP_OK;
}

static bool irq_healthy(const esp_foc_hall_ts_t *ts)
{
    return ts->isr_handle != NULL;
}

static void irq_deinit(esp_foc_hall_ts_t *ts)
{
    for (int i = 0; i < 3; i++) {
        gpio_ll_intr_disable(&GPIO, (uint32_t)ts->cfg.gpio[i]);
    }
    if (ts->isr_handle != NULL) {
        (void)esp_intr_free((intr_handle_t)ts->isr_handle);
        ts->isr_handle = NULL;
    }
}

const esp_foc_hall_ts_ops_t esp_foc_hall_ts_gpio_irq_ops = {
    .init = irq_init,
    .poll = irq_poll,
    .rearm = irq_rearm,
    .healthy = irq_healthy,
    .deinit = irq_deinit,
};
