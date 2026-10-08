/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Public IIR API — software TDF-II by default; optional HW with soft fallback.
 */
#include "espFoC/utils/esp_foc_iir.h"

#include <stdbool.h>

#include "sdkconfig.h"
#include "esp_foc_iir_soft.h"

#if CONFIG_ESP_FOC_USE_HW_ACCEL_IIR
bool esp_foc_iir_hw_available(void);
q16_t esp_foc_iir_hw_update(esp_foc_iir_t *f, q16_t x);
#endif

q16_t esp_foc_iir_update(esp_foc_iir_t *f, q16_t x)
{
#if CONFIG_ESP_FOC_USE_HW_ACCEL_IIR
    if (esp_foc_iir_hw_available()) {
        return esp_foc_iir_hw_update(f, x);
    }
#endif
    return esp_foc_iir_soft_update(f, x);
}

void esp_foc_iir_reset(esp_foc_iir_t *f)
{
    esp_foc_iir_soft_reset(f);
}

void esp_foc_iir_set_coeffs(esp_foc_iir_t *f, q16_t b0, q16_t b1, q16_t b2, q16_t a1, q16_t a2)
{
    esp_foc_iir_soft_set_coeffs(f, b0, b1, b2, a1, a2);
}

esp_err_t esp_foc_iir_design_lpf(esp_foc_iir_t *f, float fs_hz, float fc_hz)
{
    return esp_foc_iir_soft_design_lpf(f, fs_hz, fc_hz);
}
