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

struct esp_foc_pid {
    float k[3];
    float x[3];
    float y;
    float b[3];
    float limit_high;
    float limit_low;
    float sample_time;
};

int esp_foc_pid_init(struct esp_foc_pid *p, float sample_time_seconds, float integral_limit);
int esp_foc_pid_set_kp(struct esp_foc_pid *p, float kp);
int esp_foc_pid_set_ki(struct esp_foc_pid *p, float ki);
int esp_foc_pid_set_kd(struct esp_foc_pid *p, float kd);
int esp_foc_pid_update(struct esp_foc_pid *p, float ref, float mes, float *command);