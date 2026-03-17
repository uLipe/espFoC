/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "espFoC/utils/esp_foc_iq31.h"
#include <stdint.h>

#define IQ31_SIN_LUT_SIZE  512
#define IQ31_SIN_LUT_MASK  (IQ31_SIN_LUT_SIZE - 1)

/* Sin LUT: sin(2*pi * i / IQ31_SIN_LUT_SIZE) in Q1.31, i = 0 .. IQ31_SIN_LUT_SIZE-1 */
static const iq31_t iq31_sin_lut[IQ31_SIN_LUT_SIZE] = {
#include "esp_foc_iq31_sin_lut.inc"
};

/**
 * Reciprocal sqrt: input x in Q1.31 (positive), output in Q2.30.
 * Integer-only: Newton isqrt then scale. Returns 0 if x <= 0.
 */
static inline uint32_t isqrt32(uint32_t x)
{
    if (x == 0) {
        return 0;
    }
    uint32_t y = x;
    uint32_t z = (x + 1) >> 1;
    while (z < y) {
        y = z;
        z = (x / z + z) >> 1;
    }
    return y;
}

iq31_t iq31_rsqrt_fast(iq31_t x)
{
    if (x <= 0) {
        return 0;
    }
    uint32_t u = (uint32_t)x;
    uint32_t s = isqrt32(u);
    if (s == 0) {
        return (iq31_t)0x7FFFFFFF; /* saturate to max Q2.30 representable */
    }
    /* result in Q2.30: (1<<30) * sqrt(2^31/x) = 2^45.5 / s; compute in order to avoid 64-bit overflow */
    uint64_t half = (1ULL << 45) / (uint64_t)s;
    uint32_t r = (uint32_t)(half * 1414213562ULL / 1000000000ULL);
    if (r > 0x7FFFFFFFU) {
        r = 0x7FFFFFFF;
    }
    return (iq31_t)r;
}

/**
 * Angle in [0, 2*pi) as Q1.31: 0 .. IQ31_ONE (exclusive).
 * Index = (angle_q31 * LUT_SIZE) / (IQ31_ONE+1) -> 0 .. LUT_SIZE-1.
 */
iq31_t iq31_sin(iq31_t angle_q31)
{
    uint32_t u = (uint32_t)angle_q31 & 0x7FFFFFFFU;
    uint32_t idx = ((uint64_t)u * (uint64_t)IQ31_SIN_LUT_SIZE) >> 31;
    idx &= IQ31_SIN_LUT_MASK;
    return iq31_sin_lut[idx];
}

/* cos(x) = sin(x + pi/2). Wrap angle + pi/2 to [0, 2*pi) in Q1.31 */
iq31_t iq31_cos(iq31_t angle_q31)
{
    const uint32_t quarter = (uint32_t)IQ31_ONE >> 2;
    uint32_t u = (uint32_t)((int64_t)angle_q31 + (int64_t)quarter);
    u = u % ((uint32_t)IQ31_ONE + 1);
    return iq31_sin((iq31_t)u);
}
