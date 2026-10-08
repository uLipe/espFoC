/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Hardware Clarke hook. No SoC in the current support set exposes a
 * Clarke accelerator — always report unavailable.
 */
#include <stdbool.h>

#include "espFoC/utils/esp_foc_clarke.h"
#include "sdkconfig.h"

#if CONFIG_ESP_FOC_USE_HW_ACCEL_CLARKE

bool esp_foc_clarke_hw_available(void)
{
    return false;
}

void esp_foc_clarke_hw(q16_t u, q16_t v, q16_t w, q16_t *alpha, q16_t *beta)
{
    (void)u;
    (void)v;
    (void)w;
    if (alpha != NULL) {
        *alpha = 0;
    }
    if (beta != NULL) {
        *beta = 0;
    }
}

void esp_foc_inv_clarke_hw(q16_t alpha, q16_t beta, q16_t *u, q16_t *v, q16_t *w)
{
    (void)alpha;
    (void)beta;
    if (u != NULL) {
        *u = 0;
    }
    if (v != NULL) {
        *v = 0;
    }
    if (w != NULL) {
        *w = 0;
    }
}

#endif
