/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Hardware CORDIC hook. No SoC in the current support set exposes a CORDIC
 * accelerator — always report unavailable and let callers use software.
 */
#include <stdbool.h>

#include "espFoC/utils/esp_foc_q16.h"
#include "sdkconfig.h"

#if CONFIG_ESP_FOC_USE_HW_ACCEL_TRIGO

bool esp_foc_trig_hw_available(void)
{
    /* Future: return true when SOC_CAPS reports a CORDIC unit. */
    return false;
}

void esp_foc_trig_hw_sincos(q16_t angle, q16_t *s_out, q16_t *c_out)
{
    (void)angle;
    (void)s_out;
    (void)c_out;
}

q16_t esp_foc_trig_hw_atan2(q16_t y, q16_t x)
{
    (void)y;
    (void)x;
    return 0;
}

q16_t esp_foc_trig_hw_sqrt(q16_t x)
{
    (void)x;
    return 0;
}

#endif /* CONFIG_ESP_FOC_USE_HW_ACCEL_TRIGO */
