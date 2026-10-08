/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Public Clarke API — software by default; optional HW with soft fallback.
 */
#include "espFoC/utils/esp_foc_clarke.h"

#include <stdbool.h>

#include "sdkconfig.h"
#include "esp_foc_clarke_soft.h"

#if CONFIG_ESP_FOC_USE_HW_ACCEL_CLARKE
bool esp_foc_clarke_hw_available(void);
void esp_foc_clarke_hw(q16_t u, q16_t v, q16_t w, q16_t *alpha, q16_t *beta);
void esp_foc_inv_clarke_hw(q16_t alpha, q16_t beta, q16_t *u, q16_t *v, q16_t *w);
#endif

void esp_foc_clarke(q16_t u, q16_t v, q16_t w, q16_t *alpha, q16_t *beta)
{
#if CONFIG_ESP_FOC_USE_HW_ACCEL_CLARKE
    if (esp_foc_clarke_hw_available()) {
        esp_foc_clarke_hw(u, v, w, alpha, beta);
        return;
    }
#endif
    esp_foc_clarke_soft(u, v, w, alpha, beta);
}

void esp_foc_inv_clarke(q16_t alpha, q16_t beta, q16_t *u, q16_t *v, q16_t *w)
{
#if CONFIG_ESP_FOC_USE_HW_ACCEL_CLARKE
    if (esp_foc_clarke_hw_available()) {
        esp_foc_inv_clarke_hw(alpha, beta, u, v, w);
        return;
    }
#endif
    esp_foc_inv_clarke_soft(alpha, beta, u, v, w);
}
