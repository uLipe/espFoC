/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Strategy selection and the GPIO setup both strategies share.
 */
#include "hall_timestamp.h"

#include <string.h>

#include "esp_check.h"
#include "esp_rom_gpio.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"

static const char *TAG = "foc_hall_ts";

void esp_foc_hall_ts_gpio_as_input(int gpio)
{
    if (gpio < 0) {
        return;
    }
    esp_rom_gpio_pad_select_gpio((uint32_t)gpio);
    gpio_ll_func_sel(&GPIO, (uint8_t)gpio, PIN_FUNC_GPIO);
    gpio_ll_matrix_out_default(&GPIO, (uint32_t)gpio);
    /*
     * 5 V hall, open-collector. The pad must never push 3.3 V: that fights
     * the sensor, and a 5 V pull-up into a push-pull C6 pad cooks the pin.
     * Open-drain with GPIO=1 is released — the hall sinks, our 3.3 V pull-up
     * defines the high. These pins are never written to 0.
     */
    gpio_ll_od_enable(&GPIO, (uint32_t)gpio);
    gpio_ll_set_level(&GPIO, (uint32_t)gpio, 1);
    gpio_ll_input_enable(&GPIO, gpio);
    gpio_ll_output_enable(&GPIO, gpio);
    gpio_ll_pullup_en(&GPIO, gpio);
    gpio_ll_pulldown_dis(&GPIO, gpio);
}

esp_err_t esp_foc_hall_ts_init(esp_foc_hall_ts_t *ts, const esp_foc_hall_ts_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(ts != NULL && cfg != NULL, ESP_ERR_INVALID_ARG, TAG, "null");
    ESP_RETURN_ON_FALSE(!ts->inited, ESP_ERR_INVALID_STATE, TAG, "inited");
    for (int i = 0; i < 3; i++) {
        ESP_RETURN_ON_FALSE(cfg->gpio[i] >= 0, ESP_ERR_INVALID_ARG, TAG, "gpio");
    }

    memset(ts, 0, sizeof(*ts));
    ts->cfg = *cfg;

    switch (cfg->kind) {
    case ESP_FOC_HALL_TS_ETM_TIMG:
        ts->ops = &esp_foc_hall_ts_etm_timg_ops;
        break;
    case ESP_FOC_HALL_TS_GPIO_IRQ:
        ts->ops = &esp_foc_hall_ts_gpio_irq_ops;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    for (int i = 0; i < 3; i++) {
        esp_foc_hall_ts_gpio_as_input(cfg->gpio[i]);
    }

    esp_err_t err = ts->ops->init(ts);
    if (err != ESP_OK) {
        return err;
    }

    ts->inited = true;
    return ESP_OK;
}

void esp_foc_hall_ts_deinit(esp_foc_hall_ts_t *ts)
{
    if (ts == NULL || !ts->inited) {
        return;
    }
    ts->ops->deinit(ts);
    ts->inited = false;
}
