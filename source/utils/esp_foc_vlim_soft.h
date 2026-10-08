/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Internal software voltage-limiter entry point.
 */
#pragma once

#include "espFoC/utils/esp_foc_vlim.h"

void esp_foc_vlim_dq_soft(q16_t *vd, q16_t *vq, q16_t vmax);
