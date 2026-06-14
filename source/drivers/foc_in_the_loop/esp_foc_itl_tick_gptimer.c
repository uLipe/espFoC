/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#include "sdkconfig.h"
#include "esp_foc_itl_tick.h"

#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_log.h"

static const char *TAG = "foc_itl_tick";

static gptimer_handle_t s_timer;
static esp_foc_itl_tick_cb_t s_cb;
static void *s_cb_arg;

static bool IRAM_ATTR tick_isr(gptimer_handle_t timer,
                               const gptimer_alarm_event_data_t *edata,
                               void *user_data)
{
    (void)timer;
    (void)edata;
    (void)user_data;
    /* GPTimer driver clears ALARM and re-arms auto-reload before this runs. */
    if (s_cb != NULL) {
        s_cb(s_cb_arg);
    }
    return false;
}

esp_foc_err_t esp_foc_itl_tick_start(esp_foc_itl_tick_cb_t cb, void *arg, uint32_t rate_hz)
{
    if (cb == NULL || rate_hz == 0u) {
        return ESP_FOC_ERR_INVALID_ARG;
    }

    esp_foc_itl_tick_stop();

    s_cb = cb;
    s_cb_arg = arg;

    gptimer_config_t timer_cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
        .intr_priority = 1,
    };

    esp_err_t err = gptimer_new_timer(&timer_cfg, &s_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gptimer_new_timer failed: %s", esp_err_to_name(err));
        return ESP_FOC_ERR_UNKNOWN;
    }

    gptimer_event_callbacks_t cbs = {
        .on_alarm = tick_isr,
    };
    err = gptimer_register_event_callbacks(s_timer, &cbs, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register callbacks failed: %s", esp_err_to_name(err));
        gptimer_del_timer(s_timer);
        s_timer = NULL;
        return ESP_FOC_ERR_UNKNOWN;
    }

    err = gptimer_enable(s_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "timer enable failed: %s", esp_err_to_name(err));
        gptimer_del_timer(s_timer);
        s_timer = NULL;
        return ESP_FOC_ERR_UNKNOWN;
    }

    const uint64_t alarm_us = 1000000ull / (uint64_t)rate_hz;
    static gptimer_alarm_config_t alarm_cfg;
    alarm_cfg.alarm_count = alarm_us;
    alarm_cfg.reload_count = 0;
    alarm_cfg.flags.auto_reload_on_alarm = true;
    err = gptimer_set_alarm_action(s_timer, &alarm_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set alarm failed: %s", esp_err_to_name(err));
        gptimer_disable(s_timer);
        gptimer_del_timer(s_timer);
        s_timer = NULL;
        return ESP_FOC_ERR_UNKNOWN;
    }

    err = gptimer_start(s_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "timer start failed: %s", esp_err_to_name(err));
        gptimer_disable(s_timer);
        gptimer_del_timer(s_timer);
        s_timer = NULL;
        return ESP_FOC_ERR_UNKNOWN;
    }

    ESP_LOGI(TAG, "GPTimer tick @ %lu Hz", (unsigned long)rate_hz);
    return ESP_FOC_OK;
}

void esp_foc_itl_tick_stop(void)
{
    s_cb = NULL;
    s_cb_arg = NULL;
    if (s_timer != NULL) {
        gptimer_stop(s_timer);
        gptimer_disable(s_timer);
        gptimer_del_timer(s_timer);
        s_timer = NULL;
    }
}
