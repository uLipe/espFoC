/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Hardware Park hook. No SoC in the current support set exposes a
 * Park accelerator — always report unavailable.
 */
#include <stdbool.h>

#include "espFoC/utils/esp_foc_park.h"
#include "sdkconfig.h"

#if CONFIG_ESP_FOC_USE_HW_ACCEL_PARK

bool esp_foc_park_hw_available(void)
{
    return false;
}

void esp_foc_park_hw(q16_t s, q16_t c, q16_t alpha, q16_t beta, q16_t *d, q16_t *q)
{
    (void)s;
    (void)c;
    (void)alpha;
    (void)beta;
    if (d != NULL) {
        *d = 0;
    }
    if (q != NULL) {
        *q = 0;
    }
}

void esp_foc_inv_park_hw(q16_t s, q16_t c, q16_t d, q16_t q, q16_t *alpha, q16_t *beta)
{
    (void)s;
    (void)c;
    (void)d;
    (void)q;
    if (alpha != NULL) {
        *alpha = 0;
    }
    if (beta != NULL) {
        *beta = 0;
    }
}

#endif
