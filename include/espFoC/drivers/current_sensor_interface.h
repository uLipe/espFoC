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

#pragma once

#include <sdkconfig.h>

#ifdef CONFIG_ESP_FOC_USE_FIXED_POINT
#include "espFoC/utils/esp_foc_iq31.h"
#endif

typedef struct {
    float iu_axis_0;
    float iv_axis_0;
    float iw_axis_0;

    float iu_axis_1;
    float iv_axis_1;
    float iw_axis_1;
} isensor_values_t;

#ifdef CONFIG_ESP_FOC_USE_FIXED_POINT
/** Per-axis currents normalized for the FOC loop (Q1.31, typically [-1, 1)). */
typedef struct {
    iq31_t iu_axis_0;
    iq31_t iv_axis_0;
    iq31_t iw_axis_0;

    iq31_t iu_axis_1;
    iq31_t iv_axis_1;
    iq31_t iw_axis_1;
} isensor_values_iq31_t;
#endif

typedef struct esp_foc_isensor_s esp_foc_isensor_t;

typedef void (*isensor_callback_t)(void *arg);

struct esp_foc_isensor_s {
    void (*fetch_isensors)(esp_foc_isensor_t *self, isensor_values_t *values);
    void (*sample_isensors)(esp_foc_isensor_t *self);
    void (*calibrate_isensors)(esp_foc_isensor_t *self, int calibration_rounds);
    void (*set_isensor_callback)(esp_foc_isensor_t *self, isensor_callback_t cb, void *param);
#ifdef CONFIG_ESP_FOC_USE_FIXED_POINT
    /** Same semantics as fetch_isensors; values in Q1.31. */
    void (*fetch_isensors_iq31)(esp_foc_isensor_t *self, isensor_values_iq31_t *values);
#endif
};