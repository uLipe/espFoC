/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Internal soft-CORDIC entry points (used by public API and HW fallback).
 */
#pragma once

#include "espFoC/utils/esp_foc_q16.h"

void esp_foc_trig_soft_sincos(q16_t angle, q16_t *s_out, q16_t *c_out);
q16_t esp_foc_trig_soft_atan2(q16_t y, q16_t x);
q16_t esp_foc_trig_soft_sqrt(q16_t x);
