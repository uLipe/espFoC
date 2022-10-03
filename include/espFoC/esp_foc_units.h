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

typedef struct {
    float raw;
} esp_foc_q_voltage;

typedef struct {
    float raw;
} esp_foc_d_voltage;

typedef struct {
    float raw;
} esp_foc_alpha_voltage;

typedef struct {
    float raw;
} esp_foc_beta_voltage;

typedef struct {
    float raw;
} esp_foc_u_voltage;

typedef struct {
    float raw;
} esp_foc_v_voltage;

typedef struct {
    float raw;
} esp_foc_w_voltage;

typedef struct {
    float raw;
} esp_foc_q_current;

typedef struct {
    float raw;
} esp_foc_d_current;

typedef struct {
    float raw;
} esp_foc_alpha_current;

typedef struct {
    float raw;
} esp_foc_beta_current;

typedef struct {
    float raw;
} esp_foc_u_current;

typedef struct {
    float raw;
} esp_foc_v_current;

typedef struct {
    float raw;
} esp_foc_w_current;

typedef struct {
    float raw;
} esp_foc_radians_per_second;

typedef struct {
    float raw;
} esp_foc_radians;

typedef struct {
    float raw;
} esp_foc_seconds;