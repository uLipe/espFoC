/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Software CORDIC: circular (sin/cos/atan2) and hyperbolic (sqrt).
 * Circular: 16 useful iters for Q16.16 (atan table LSB-limited); hyperbolic 32-style.
 * Circular core on [0, π/2] with quadrant extension to (−π, +π].
 */
#include "esp_foc_trig_soft.h"

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>

#include "espFoC/utils/esp_foc_angle.h"
#include "espFoC/utils/esp_foc_q16.h"

/* Q16.16 atan(2^-i) becomes <1 LSB after i≈16; keep 16 hot-path iters. */
#define CORDIC_CIRC_ITERS 16
#define CORDIC_HYP_ITERS 32

/* atan(2^-i) in Q16.16 */
static const q16_t s_atan_table[CORDIC_CIRC_ITERS] = {
    51472, 30386, 16055, 8150, 4091, 2047, 1024, 512,
    256, 128, 64, 32, 16, 8, 4, 2,
};

/* Circular gain K = Π cos(atan(2^-i)) ≈ 0.607252935 → Q16.16 */
#define CORDIC_K_Q16 ((q16_t)39797)

/* atanh(2^-i) for i = 1..31 (index 0 → i=1) — reserved / documentation */
static const q16_t s_atanh_table[31] = {
    35999, 16739, 8235, 4101, 2049, 1024, 512, 256,
    128, 64, 32, 16, 8, 4, 2, 1,
    1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0,
};

/* Hyperbolic gain Ah for iters 1..31 with repeats at 4 and 13 ≈ 0.828159 */
#define CORDIC_AH_Q16 ((q16_t)54274)

static q16_t sat_i32_to_q16(int32_t v)
{
    return (q16_t)v;
}

static q16_t sat_i64_to_q16(int64_t v)
{
    if (v > INT32_MAX) {
        return (q16_t)INT32_MAX;
    }
    if (v < INT32_MIN) {
        return (q16_t)INT32_MIN;
    }
    return (q16_t)v;
}

/* Unit-circle CORDIC stays near |x|,|y| ≤ ~Q16_ONE — int32 is enough and ISR-cheap. */
static void cordic_circular_rotate(int32_t *x, int32_t *y, int32_t *z)
{
    for (int i = 0; i < CORDIC_CIRC_ITERS; i++) {
        int32_t x0 = *x;
        int32_t y0 = *y;
        int32_t z0 = *z;
        if (z0 >= 0) {
            *x = x0 - (y0 >> i);
            *y = y0 + (x0 >> i);
            *z = z0 - s_atan_table[i];
        } else {
            *x = x0 + (y0 >> i);
            *y = y0 - (x0 >> i);
            *z = z0 + s_atan_table[i];
        }
    }
}

static void cordic_circular_vector(int32_t *x, int32_t *y, int32_t *z)
{
    for (int i = 0; i < CORDIC_CIRC_ITERS; i++) {
        int32_t x0 = *x;
        int32_t y0 = *y;
        int32_t z0 = *z;
        if (y0 >= 0) {
            *x = x0 + (y0 >> i);
            *y = y0 - (x0 >> i);
            *z = z0 + s_atan_table[i];
        } else {
            *x = x0 - (y0 >> i);
            *y = y0 + (x0 >> i);
            *z = z0 - s_atan_table[i];
        }
    }
}

static void cordic_sincos_quad(q16_t a_0_pi2, q16_t *s_out, q16_t *c_out)
{
    int32_t x = CORDIC_K_Q16;
    int32_t y = 0;
    int32_t z = a_0_pi2;
    cordic_circular_rotate(&x, &y, &z);
    *c_out = sat_i32_to_q16(x);
    *s_out = sat_i32_to_q16(y);
}

void esp_foc_trig_soft_sincos(q16_t angle, q16_t *s_out, q16_t *c_out)
{
    q16_t s;
    q16_t c;
    bool neg_s = false;
    bool neg_c = false;

    q16_t z = q16_wrap_pi(angle);
    if (z < 0) {
        z = q16_neg(z);
        neg_s = true;
    }
    if (z > Q16_PI_2) {
        z = q16_sub(Q16_PI, z);
        neg_c = true;
    }
    if (z < 0) {
        z = 0;
    }
    if (z > Q16_PI_2) {
        z = Q16_PI_2;
    }

    cordic_sincos_quad(z, &s, &c);
    if (neg_s) {
        s = q16_neg(s);
    }
    if (neg_c) {
        c = q16_neg(c);
    }
    if (s_out) {
        *s_out = s;
    }
    if (c_out) {
        *c_out = c;
    }
}

q16_t esp_foc_trig_soft_atan2(q16_t y, q16_t x)
{
    if (x == 0 && y == 0) {
        return 0;
    }
    if (x == 0) {
        return (y > 0) ? Q16_PI_2 : Q16_MINUS_PI_2;
    }

    int32_t xi = x;
    int32_t yi = y;
    int32_t zi = 0;
    q16_t offset = 0;

    if (xi < 0) {
        xi = -xi;
        yi = -yi;
        offset = (y >= 0) ? Q16_PI : Q16_MINUS_PI;
    }

    cordic_circular_vector(&xi, &yi, &zi);
    return q16_wrap_pi(q16_add(sat_i32_to_q16(zi), offset));
}

static void hyp_vector_iter(int64_t *x, int64_t *y, int i)
{
    int64_t x0 = *x;
    int64_t y0 = *y;
    int64_t d = (y0 >= 0) ? (int64_t)-1 : (int64_t)1;
    *x = x0 + ((d * y0) >> i);
    *y = y0 + ((d * x0) >> i);
    (void)s_atanh_table;
}

static q16_t hyp_sqrt_normalized(q16_t v)
{
    int64_t x = (int64_t)v + (int64_t)(Q16_ONE / 4);
    int64_t y = (int64_t)v - (int64_t)(Q16_ONE / 4);

    for (int i = 1; i < CORDIC_HYP_ITERS; i++) {
        hyp_vector_iter(&x, &y, i);
        if (i == 4 || i == 13) {
            hyp_vector_iter(&x, &y, i);
        }
    }

    int64_t r = (x << 16) / (int64_t)CORDIC_AH_Q16;
    return sat_i64_to_q16(r);
}

q16_t esp_foc_trig_soft_sqrt(q16_t x)
{
    if (x <= 0) {
        return 0;
    }

    q16_t v = x;
    int shift = 0;

    while (v >= Q16_ONE) {
        v = (q16_t)((uint32_t)v >> 2);
        shift++;
    }
    while (v > 0 && v < (Q16_ONE / 4)) {
        if (v > (q16_t)(INT32_MAX >> 2)) {
            break;
        }
        v = (q16_t)((uint32_t)v << 2);
        shift--;
    }

    q16_t r = hyp_sqrt_normalized(v);
    if (shift > 0) {
        if (shift >= 31) {
            return (q16_t)INT32_MAX;
        }
        return sat_i64_to_q16((int64_t)r << shift);
    }
    if (shift < 0) {
        int sh = -shift;
        if (sh >= 31) {
            return 0;
        }
        return (q16_t)(r >> sh);
    }
    return r;
}
