/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Public SVM API — software by default; optional HW with soft fallback.
 */
#include "espFoC/utils/esp_foc_svm.h"

#include <stdbool.h>

#include "sdkconfig.h"
#include "esp_foc_svm_soft.h"

#if CONFIG_ESP_FOC_USE_HW_ACCEL_SVM
bool esp_foc_svm_hw_available(void);
void esp_foc_svm_hw(q16_t v_alpha, q16_t v_beta, q16_t *du, q16_t *dv, q16_t *dw);
#endif

void esp_foc_svm(q16_t v_alpha, q16_t v_beta, q16_t *du, q16_t *dv, q16_t *dw)
{
#if CONFIG_ESP_FOC_USE_HW_ACCEL_SVM
    if (esp_foc_svm_hw_available()) {
        esp_foc_svm_hw(v_alpha, v_beta, du, dv, dw);
        return;
    }
#endif
    esp_foc_svm_soft(v_alpha, v_beta, du, dv, dw);
}
