/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Internal software SVM entry point.
 */
#pragma once

#include "espFoC/utils/esp_foc_svm.h"

void esp_foc_svm_soft(q16_t v_alpha, q16_t v_beta, q16_t *du, q16_t *dv, q16_t *dw);
