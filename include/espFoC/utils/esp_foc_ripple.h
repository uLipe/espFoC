/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdint.h>

#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Standard deviation of a signal about its own least-squares line, from
 * running sums: a slow drift or ramp during the window is not counted as
 * ripple. Samples are taken relative to the first one and scaled down by
 * ESP_FOC_RIPPLE_SHIFT so the squared sums of a long window fit in 64 bits.
 *
 * add() runs in the producer (interrupt); arm(), stop() and sigma() in a
 * task, with sigma() read after stop().
 */
#define ESP_FOC_RIPPLE_SHIFT 8

typedef struct {
    volatile uint32_t n;
    volatile uint32_t budget;
    q16_t seed;
    int64_t sum;
    int64_t sq;
    int64_t ix;
} esp_foc_ripple_t;

static inline void esp_foc_ripple_arm(esp_foc_ripple_t *r, uint32_t budget)
{
    r->sum = 0;
    r->sq = 0;
    r->ix = 0;
    r->n = 0u;
    r->budget = budget;
}

static inline void esp_foc_ripple_stop(esp_foc_ripple_t *r)
{
    r->budget = 0u;
}

/** Takes x while fewer than budget samples have been taken. O(1). */
static inline void esp_foc_ripple_add(esp_foc_ripple_t *r, q16_t x)
{
    const uint32_t n = r->n;
    if (n >= r->budget) {
        return;
    }
    if (n == 0u) {
        r->seed = x;
    }
    const int64_t v = (int64_t)q16_sub(x, r->seed) >> ESP_FOC_RIPPLE_SHIFT;
    r->sum += v;
    r->sq += v * v;
    r->ix += (int64_t)n * v;
    r->n = n + 1u;
}

/** σ of the detrended samples in the units of x; 0 under min_n (>= 3) samples. */
float esp_foc_ripple_sigma(const esp_foc_ripple_t *r, uint32_t min_n);

#ifdef __cplusplus
}
#endif
