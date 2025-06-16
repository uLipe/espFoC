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

typedef void (*foc_loop_runner) (void *axis);
typedef void*  esp_foc_event_handle_t;

int esp_foc_create_runner(foc_loop_runner runner, void *argument, int priority);
void esp_foc_sleep_ms(int sleep_ms);
void esp_foc_runner_yield(void);
float esp_foc_now_seconds(void);
void esp_foc_fpu_isr_enter(void);
void esp_foc_fpu_isr_leave(void);
void esp_foc_critical_enter(void);
void esp_foc_critical_leave(void);
esp_foc_event_handle_t esp_foc_get_event_handle(void);
void esp_foc_wait_notifier(void);
void esp_foc_send_notification(esp_foc_event_handle_t handle);
