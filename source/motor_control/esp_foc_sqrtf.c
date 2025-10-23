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

#include <stdint.h>
#include <math.h>
#include "esp_attr.h"

#define SQRT_TABLE_BITS 12
#define SQRT_TABLE_SIZE (1 << SQRT_TABLE_BITS)

static float sqrt_table[SQRT_TABLE_SIZE];

void esp_foc_fast_init_sqrt_table(void)
{
    for (int i = 0; i < SQRT_TABLE_SIZE; i++) {
        float x = 1.0f + 3.0f * ((float)i / (SQRT_TABLE_SIZE - 1));
        sqrt_table[i] = sqrtf(x);
    }
}

IRAM_ATTR float esp_foc_fast_sqrt(float x)
{
    if (x <= 0.0f) {
        return 0.0f;
    }

    union {
        float f;
        uint32_t i;
    } u = { x };

    int e = (u.i >> 23) & 0xff;
    int exponent = e - 127;
    float m = 1.0f + (u.i & 0x7fffff) / 8388608.0f;
    int odd = exponent & 1;

    if (odd) {
        m *= 2.0f;
    }

    float t = (m - 1.0f) / 3.0f;
    float pos = t * (SQRT_TABLE_SIZE - 1);
    int index = (int) pos;

    if (index >= SQRT_TABLE_SIZE - 1) {
        index = SQRT_TABLE_SIZE - 2;
    }

    float frac = pos - index;
    float sqrt_m = sqrt_table[index] + frac * (sqrt_table[index + 1] - sqrt_table[index]);

    if (odd) {
        sqrt_m *= 0.707106781186547524f;
    }

    int new_exp = exponent >> 1;
    return ldexpf(sqrt_m, new_exp);
}
