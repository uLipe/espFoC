/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Hardware IIR hook. No SoC in the current support set exposes an IIR
 * accelerator — always report unavailable.
 */
#include <stdbool.h>

#include "espFoC/utils/esp_foc_iir.h"
#include "sdkconfig.h"

#if CONFIG_ESP_FOC_USE_HW_ACCEL_IIR

bool esp_foc_iir_hw_available(void)
{
    return false;
}

q16_t esp_foc_iir_hw_update(esp_foc_iir_t *f, q16_t x)
{
    (void)f;
    (void)x;
    return 0;
}

#endif
