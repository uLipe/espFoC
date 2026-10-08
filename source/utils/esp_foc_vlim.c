/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Public vlim API — software by default; optional HW with soft fallback.
 */
#include "espFoC/utils/esp_foc_vlim.h"

#include <stdbool.h>

#include "sdkconfig.h"
#include "esp_foc_vlim_soft.h"

#if CONFIG_ESP_FOC_USE_HW_ACCEL_VLIM
bool esp_foc_vlim_hw_available(void);
void esp_foc_vlim_dq_hw(q16_t *vd, q16_t *vq, q16_t vmax);
#endif

void esp_foc_vlim_dq(q16_t *vd, q16_t *vq, q16_t vmax)
{
#if CONFIG_ESP_FOC_USE_HW_ACCEL_VLIM
    if (esp_foc_vlim_hw_available()) {
        esp_foc_vlim_dq_hw(vd, vq, vmax);
        return;
    }
#endif
    esp_foc_vlim_dq_soft(vd, vq, vmax);
}
