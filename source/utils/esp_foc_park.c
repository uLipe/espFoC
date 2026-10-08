/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Public Park API — software by default; optional HW with soft fallback.
 */
#include "espFoC/utils/esp_foc_park.h"

#include <stdbool.h>

#include "sdkconfig.h"
#include "esp_foc_park_soft.h"

#if CONFIG_ESP_FOC_USE_HW_ACCEL_PARK
bool esp_foc_park_hw_available(void);
void esp_foc_park_hw(q16_t s, q16_t c, q16_t alpha, q16_t beta, q16_t *d, q16_t *q);
void esp_foc_inv_park_hw(q16_t s, q16_t c, q16_t d, q16_t q, q16_t *alpha, q16_t *beta);
#endif

void esp_foc_park(q16_t s, q16_t c, q16_t alpha, q16_t beta, q16_t *d, q16_t *q)
{
#if CONFIG_ESP_FOC_USE_HW_ACCEL_PARK
    if (esp_foc_park_hw_available()) {
        esp_foc_park_hw(s, c, alpha, beta, d, q);
        return;
    }
#endif
    esp_foc_park_soft(s, c, alpha, beta, d, q);
}

void esp_foc_inv_park(q16_t s, q16_t c, q16_t d, q16_t q, q16_t *alpha, q16_t *beta)
{
#if CONFIG_ESP_FOC_USE_HW_ACCEL_PARK
    if (esp_foc_park_hw_available()) {
        esp_foc_inv_park_hw(s, c, d, q, alpha, beta);
        return;
    }
#endif
    esp_foc_inv_park_soft(s, c, d, q, alpha, beta);
}
