/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Internal software Clarke entry points.
 */
#pragma once

#include "espFoC/utils/esp_foc_clarke.h"

void esp_foc_clarke_soft(q16_t u, q16_t v, q16_t w, q16_t *alpha, q16_t *beta);
void esp_foc_inv_clarke_soft(q16_t alpha, q16_t beta, q16_t *u, q16_t *v, q16_t *w);
