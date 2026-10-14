/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * DC reference on an on-chip DAC pad, e.g. the N pad of a differential
 * current-sense pair. Set up by the application; the inverter does not use
 * it. On chips without the DAC, init returns ESP_ERR_NOT_SUPPORTED.
 */
typedef struct esp_foc_vref_generator esp_foc_vref_generator_t;

#define ESP_FOC_VREF_FULL_SCALE_V 3.3f

typedef struct {
    /* DAC channel. Each one drives a dedicated pad (no GPIO matrix): on the
     * ESP32-S31 channel 0 is GPIO4 and channel 1 is GPIO5. */
    uint8_t channel;
    /* Must be the channel's pad; init rejects any other GPIO. */
    int gpio;
    /* Wait inside set_voltage after the new code, in ms. 0 -> default. */
    uint32_t settle_ms;
} esp_foc_vref_generator_config_t;

esp_err_t esp_foc_vref_generator_init(esp_foc_vref_generator_t **out,
                                      const esp_foc_vref_generator_config_t *cfg);

/* 0 .. ESP_FOC_VREF_FULL_SCALE_V, clamped. Returns after the settle time. */
esp_err_t esp_foc_vref_generator_set_voltage(esp_foc_vref_generator_t *g, float volts);

esp_err_t esp_foc_vref_generator_deinit(esp_foc_vref_generator_t *g);

#ifdef __cplusplus
}
#endif
