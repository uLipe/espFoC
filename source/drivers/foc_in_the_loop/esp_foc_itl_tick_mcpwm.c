/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#include "sdkconfig.h"
#include "esp_foc_itl_tick.h"

#include "driver/mcpwm_prelude.h"
#include "esp_log.h"

static const char *TAG = "foc_itl_tick";

static mcpwm_timer_handle_t s_timer;
static esp_foc_itl_tick_cb_t s_cb;
static void *s_cb_arg;

static bool tick_isr(mcpwm_timer_handle_t timer,
                     const mcpwm_timer_event_data_t *edata,
                     void *user_data)
{
    (void)timer;
    (void)edata;
    (void)user_data;
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

    s_cb = cb;
    s_cb_arg = arg;

    const uint32_t resolution_hz = 160000000u;
    const uint32_t period_ticks = resolution_hz / rate_hz;

    mcpwm_timer_config_t timer_cfg = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = resolution_hz,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
        .period_ticks = period_ticks,
    };

    esp_err_t err = mcpwm_new_timer(&timer_cfg, &s_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_new_timer failed: %s", esp_err_to_name(err));
        return ESP_FOC_ERR_UNKNOWN;
    }

    mcpwm_timer_event_callbacks_t cbs = {
        .on_full = tick_isr,
    };
    err = mcpwm_timer_register_event_callbacks(s_timer, &cbs, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register callbacks failed: %s", esp_err_to_name(err));
        mcpwm_del_timer(s_timer);
        s_timer = NULL;
        return ESP_FOC_ERR_UNKNOWN;
    }

    err = mcpwm_timer_enable(s_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "timer enable failed: %s", esp_err_to_name(err));
        mcpwm_del_timer(s_timer);
        s_timer = NULL;
        return ESP_FOC_ERR_UNKNOWN;
    }

    err = mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_START_NO_STOP);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "timer start failed: %s", esp_err_to_name(err));
        mcpwm_timer_disable(s_timer);
        mcpwm_del_timer(s_timer);
        s_timer = NULL;
        return ESP_FOC_ERR_UNKNOWN;
    }

    ESP_LOGI(TAG, "MCPWM tick @ %lu Hz", (unsigned long)rate_hz);
    return ESP_FOC_OK;
}

void esp_foc_itl_tick_stop(void)
{
    if (s_timer != NULL) {
        mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_STOP_EMPTY);
        mcpwm_timer_disable(s_timer);
        mcpwm_del_timer(s_timer);
        s_timer = NULL;
    }
    s_cb = NULL;
    s_cb_arg = NULL;
}
