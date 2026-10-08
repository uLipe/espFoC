/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Hardware vlim hook. No SoC in the current support set exposes a
 * voltage-limit accelerator — always report unavailable.
 */
#include <stdbool.h>

#include "espFoC/utils/esp_foc_vlim.h"
#include "sdkconfig.h"

#if CONFIG_ESP_FOC_USE_HW_ACCEL_VLIM

bool esp_foc_vlim_hw_available(void)
{
    return false;
}

void esp_foc_vlim_dq_hw(q16_t *vd, q16_t *vq, q16_t vmax)
{
    (void)vmax;
    if (vd != NULL) {
        *vd = 0;
    }
    if (vq != NULL) {
        *vq = 0;
    }
}

#endif
