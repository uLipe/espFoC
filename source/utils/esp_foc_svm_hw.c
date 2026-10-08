/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Hardware SVM hook. No SoC in the current support set exposes an SVM
 * accelerator — always report unavailable.
 */
#include <stdbool.h>

#include "espFoC/utils/esp_foc_svm.h"
#include "sdkconfig.h"

#if CONFIG_ESP_FOC_USE_HW_ACCEL_SVM

bool esp_foc_svm_hw_available(void)
{
    return false;
}

void esp_foc_svm_hw(q16_t v_alpha, q16_t v_beta, q16_t *du, q16_t *dv, q16_t *dw)
{
    (void)v_alpha;
    (void)v_beta;
    if (du != NULL) {
        *du = Q16_HALF;
    }
    if (dv != NULL) {
        *dv = Q16_HALF;
    }
    if (dw != NULL) {
        *dw = Q16_HALF;
    }
}

#endif
