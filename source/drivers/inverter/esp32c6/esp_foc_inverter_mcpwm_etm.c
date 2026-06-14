/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include "sdkconfig.h"
#include "soc/soc_caps.h"

#if SOC_ETM_SUPPORTED

#include "esp_foc_inverter_mcpwm_etm.h"

#include "driver/mcpwm_etm.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_foc_inverter_internal.h"

static const char *TAG = "inverter_mcpwm_etm";

#define INVERTER_MCPWM_ETM_SLOTS  (CONFIG_NOOF_AXIS * 2)

typedef struct {
    esp_foc_inverter_t *iface;
    inverter_mcpwm_etm_t *etm;
} inverter_mcpwm_etm_slot_t;

static inverter_mcpwm_etm_slot_t s_slots[INVERTER_MCPWM_ETM_SLOTS];

esp_err_t inverter_mcpwm_etm_init(inverter_mcpwm_etm_t *etm,
                                  mcpwm_oper_handle_t oper,
                                  uint32_t period_ticks)
{
    ESP_RETURN_ON_FALSE(etm && oper, ESP_ERR_INVALID_ARG, TAG, "invalid arg");

    mcpwm_comparator_config_t cfg_center = {
        .flags.update_cmp_on_tez = true,
    };
    mcpwm_comparator_config_t cfg_peak = {
        .flags.update_cmp_on_tep = true,
    };

    ESP_RETURN_ON_ERROR(mcpwm_new_comparator(oper, &cfg_center, &etm->cmpr_center), TAG,
                        "center trigger comparator failed");
    ESP_RETURN_ON_ERROR(mcpwm_new_comparator(oper, &cfg_peak, &etm->cmpr_peak), TAG,
                        "peak trigger comparator failed");
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(etm->cmpr_center, 0), TAG,
                        "center compare value failed");
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(etm->cmpr_peak, period_ticks), TAG,
                        "peak compare value failed");

    const mcpwm_cmpr_etm_event_config_t ev_cfg = {
        .event_type = MCPWM_CMPR_ETM_EVENT_EQUAL,
    };
    ESP_RETURN_ON_ERROR(mcpwm_comparator_new_etm_event(etm->cmpr_center, &ev_cfg, &etm->event_center),
                        TAG, "center ETM event failed");
    ESP_RETURN_ON_ERROR(mcpwm_comparator_new_etm_event(etm->cmpr_peak, &ev_cfg, &etm->event_peak),
                        TAG, "peak ETM event failed");

    etm->inited = true;
    ESP_LOGI(TAG, "ETM trigger comparators ready (center=0, peak=%lu)", (unsigned long)period_ticks);
    return ESP_OK;
}

void inverter_mcpwm_etm_register(esp_foc_inverter_t *iface, inverter_mcpwm_etm_t *etm)
{
    if (iface == NULL || etm == NULL || !etm->inited) {
        return;
    }
    for (int i = 0; i < INVERTER_MCPWM_ETM_SLOTS; ++i) {
        if (s_slots[i].iface == iface) {
            s_slots[i].etm = etm;
            return;
        }
        if (s_slots[i].iface == NULL) {
            s_slots[i].iface = iface;
            s_slots[i].etm = etm;
            return;
        }
    }
    ESP_LOGW(TAG, "ETM slot table full; inverter %p not registered", (void *)iface);
}

static inverter_mcpwm_etm_t *inverter_mcpwm_etm_lookup(esp_foc_inverter_t *iface)
{
    if (iface == NULL) {
        return NULL;
    }
    for (int i = 0; i < INVERTER_MCPWM_ETM_SLOTS; ++i) {
        if (s_slots[i].iface == iface) {
            return s_slots[i].etm;
        }
    }
    return NULL;
}

static esp_etm_event_handle_t inverter_mcpwm_etm_event(const inverter_mcpwm_etm_t *etm,
                                                       inverter_mcpwm_adc_trigger_t trigger)
{
    if (etm == NULL || !etm->inited) {
        return NULL;
    }
    switch (trigger) {
    case INVERTER_MCPWM_ADC_TRIGGER_CENTER:
        return etm->event_center;
    case INVERTER_MCPWM_ADC_TRIGGER_PEAK:
        return etm->event_peak;
    default:
        return NULL;
    }
}

esp_foc_err_t inverter_mcpwm_connect_adc_etm(esp_foc_inverter_t *inverter,
                                             inverter_mcpwm_adc_trigger_t trigger)
{
    inverter_mcpwm_etm_t *etm = inverter_mcpwm_etm_lookup(inverter);
    esp_etm_event_handle_t event = inverter_mcpwm_etm_event(etm, trigger);
    if (event == NULL) {
        return ESP_FOC_ERR_NOT_SUPPORTED;
    }
    if (isensor_adc_etm_connect(event) != ESP_OK) {
        return ESP_FOC_ERR_UNKNOWN;
    }
    if (isensor_adc_etm_enable(true) != ESP_OK) {
        return ESP_FOC_ERR_UNKNOWN;
    }
    return ESP_FOC_OK;
}

#endif /* SOC_ETM_SUPPORTED */
