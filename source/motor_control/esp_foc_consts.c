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

#include <math.h>

#ifdef CONFIG_ESP_FOC_CUSTOM_MATH
const float ESP_FOC_FAST_PI = 3.14159265358f;
const float ESP_FOC_FAST_2PI = ESP_FOC_FAST_PI * 2.0f;
const float ESP_FOC_SIN_COS_APPROX_B = 4.0f / ESP_FOC_FAST_PI;
const float ESP_FOC_SIN_COS_APPROX_C = -4.0f / (ESP_FOC_FAST_PI * ESP_FOC_FAST_PI);
const float ESP_FOC_SIN_COS_APPROX_P = 0.225f;
const float ESP_FOC_SIN_COS_APPROX_D = ESP_FOC_FAST_PI/2.0f;
#endif

const float ESP_FOC_CLARKE_K1 = 2.0/3.0f;
const float ESP_FOC_CLARKE_K2 = 1.0/3.0f;
const float ESP_FOC_CLARKE_PARK_SQRT3 = 1.73205080757f;
const float ESP_FOC_CLARKE_K3 = 1.0f / ESP_FOC_CLARKE_PARK_SQRT3;
const float ESP_FOC_SQRT3_TWO = ESP_FOC_CLARKE_PARK_SQRT3 / 2.0f;
