/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Internal software Park entry points.
 */
#pragma once

#include "espFoC/utils/esp_foc_park.h"

void esp_foc_park_soft(q16_t s, q16_t c, q16_t alpha, q16_t beta, q16_t *d, q16_t *q);
void esp_foc_inv_park_soft(q16_t s, q16_t c, q16_t d, q16_t q, q16_t *alpha, q16_t *beta);
