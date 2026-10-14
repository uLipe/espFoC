/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Trig for the TEZ hot path on the chip's CORDIC unit. On chips without one,
 * and before init, every call runs the software trig
 * (espFoC/utils/esp_foc_trig.h). Inputs and outputs are Q16.16.
 *
 * Single owner: call the compute functions only from the TEZ ISR. The unit
 * runs one operation at a time and there is no lock, so task context keeps
 * using esp_foc_sincos / esp_foc_atan2 / esp_foc_sqrt.
 */

/* Clocks the unit. Task context, before the TEZ callback starts. Idempotent. */
esp_err_t esp_foc_trigo_hw_init(void);

/* True once init has brought up a CORDIC unit. */
bool esp_foc_trigo_hw_accelerated(void);

/* Any angle in radians (no wrap needed). Max error 2 LSB on the CORDIC. */
void esp_foc_trigo_hw_sincos(q16_t angle, q16_t *s_out, q16_t *c_out);

/* Result in (-pi, +pi]; atan2(0, 0) -> 0. */
q16_t esp_foc_trigo_hw_atan2(q16_t y, q16_t x);

/* sqrt(x^2 + y^2) without forming the squares on the CORDIC; the software
 * fallback saturates them for |x|, |y| above ~181. */
q16_t esp_foc_trigo_hw_magnitude(q16_t x, q16_t y);

/* Operations that fell back to software because the unit did not report
 * ready in time. */
uint32_t esp_foc_trigo_hw_get_timeouts(void);

#ifdef __cplusplus
}
#endif
