/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * ETM channel: MCPWM driver event -> ADC digi START task.
 */

#include <stdlib.h>
#include "sdkconfig.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_etm.h"
#include "esp_heap_caps.h"
#include "esp_private/etm_interface.h"
#include "soc/soc_etm_source.h"
#include "esp_foc_inverter_internal.h"

#if SOC_ETM_SUPPORTED

static const char *TAG = "isensor_adc_etm";

typedef struct {
    esp_etm_task_t base;
} isensor_adc_etm_task_t;

typedef struct {
    esp_etm_channel_handle_t chan;
    esp_etm_task_handle_t task;
    bool connected;
    bool enabled;
} isensor_adc_etm_ctx_t;

static isensor_adc_etm_ctx_t s_etm;

static esp_err_t isensor_adc_etm_del_task(esp_etm_task_t *task)
{
    isensor_adc_etm_task_t *tk = __containerof(task, isensor_adc_etm_task_t, base);
    free(tk);
    return ESP_OK;
}

static esp_err_t isensor_adc_etm_create_adc_start_task(esp_etm_task_handle_t *out)
{
    isensor_adc_etm_task_t *tk = heap_caps_calloc(1, sizeof(*tk), MALLOC_CAP_DEFAULT);
    ESP_RETURN_ON_FALSE(tk, ESP_ERR_NO_MEM, TAG, "no mem for etm task");

    tk->base.task_id = ADC_TASK_START0;
    tk->base.trig_periph = ETM_TRIG_PERIPH_MCPWM;
    tk->base.del = isensor_adc_etm_del_task;
    *out = &tk->base;
    return ESP_OK;
}

static void isensor_adc_etm_teardown(void)
{
    if (s_etm.enabled) {
        esp_etm_channel_disable(s_etm.chan);
        s_etm.enabled = false;
    }
    if (s_etm.connected) {
        esp_etm_channel_connect(s_etm.chan, NULL, NULL);
        s_etm.connected = false;
    }
    if (s_etm.task) {
        esp_etm_del_task(s_etm.task);
        s_etm.task = NULL;
    }
    if (s_etm.chan) {
        esp_etm_del_channel(s_etm.chan);
        s_etm.chan = NULL;
    }
}

esp_err_t isensor_adc_etm_connect(esp_etm_event_handle_t mcpwm_event)
{
    ESP_RETURN_ON_FALSE(mcpwm_event, ESP_ERR_INVALID_ARG, TAG, "mcpwm_event is NULL");

    isensor_adc_etm_teardown();

    ESP_RETURN_ON_ERROR(isensor_adc_etm_create_adc_start_task(&s_etm.task), TAG, "adc task create failed");

    esp_etm_channel_config_t chan_cfg = {};
    ESP_RETURN_ON_ERROR(esp_etm_new_channel(&chan_cfg, &s_etm.chan), TAG, "channel alloc failed");
    ESP_RETURN_ON_ERROR(esp_etm_channel_connect(s_etm.chan, mcpwm_event, s_etm.task), TAG, "connect failed");
    s_etm.connected = true;

    ESP_LOGI(TAG, "ETM MCPWM comparator event -> ADC_TASK_START0");
    return ESP_OK;
}

esp_err_t isensor_adc_etm_enable(bool enable)
{
    if (!s_etm.chan || !s_etm.connected) {
        return ESP_ERR_INVALID_STATE;
    }
    if (enable == s_etm.enabled) {
        return ESP_OK;
    }
    esp_err_t err = enable ? esp_etm_channel_enable(s_etm.chan) : esp_etm_channel_disable(s_etm.chan);
    if (err == ESP_OK) {
        s_etm.enabled = enable;
    }
    return err;
}

void isensor_adc_etm_disconnect(void)
{
    isensor_adc_etm_teardown();
}

#endif /* SOC_ETM_SUPPORTED */
