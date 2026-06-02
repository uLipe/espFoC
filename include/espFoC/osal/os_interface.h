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

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef void (*foc_loop_runner) (void *axis);
typedef void*  esp_foc_event_handle_t;

int esp_foc_create_runner(foc_loop_runner runner, void *argument, int priority,
                          void **out_task_handle);
bool esp_foc_runner_is_alive(void *task_handle);
void esp_foc_runner_wake(void *task_handle);
bool esp_foc_in_task_context(void);
void esp_foc_sleep_ms(int sleep_ms);
void esp_foc_runner_yield(void);
uint64_t esp_foc_now_useconds(void);
void esp_foc_critical_enter(void);
void esp_foc_critical_leave(void);
esp_foc_event_handle_t esp_foc_get_event_handle(void);
void esp_foc_wait_notifier(void);
void esp_foc_send_notification_from_isr(esp_foc_event_handle_t handle);
void esp_foc_send_notification(esp_foc_event_handle_t handle);
int esp_foc_debug_pin_init(int debug_pin);
int esp_foc_debug_pin_set(void);
int esp_foc_debug_pin_clear(void);

void esp_foc_runner_delete_self(void);

uint32_t esp_foc_ms_to_wait_ticks(unsigned ms);

int esp_foc_task_spawn(foc_loop_runner fn, void *arg, size_t stack_bytes,
                       int freertos_priority, void **out_task_handle_opt);

typedef struct esp_foc_mutex esp_foc_mutex_t;

/** @return 0 on success, negative on failure. */
int esp_foc_mutex_create(esp_foc_mutex_t **out);
void esp_foc_mutex_destroy(esp_foc_mutex_t *mutex);
void esp_foc_mutex_lock(esp_foc_mutex_t *mutex);
void esp_foc_mutex_unlock(esp_foc_mutex_t *mutex);

typedef void (*esp_foc_timer_callback_t)(void *arg);
typedef struct esp_foc_timer esp_foc_timer_t;

int esp_foc_timer_create(esp_foc_timer_callback_t cb, void *arg, esp_foc_timer_t **out);
void esp_foc_timer_destroy(esp_foc_timer_t *t);
int esp_foc_timer_arm_oneshot_us(esp_foc_timer_t *t, uint64_t period_us);
int esp_foc_timer_arm_periodic_us(esp_foc_timer_t *t, uint64_t period_us);
void esp_foc_timer_cancel(esp_foc_timer_t *t);

/** Console (platform REPL port). Used by espfoc_shell — no driver headers in shell. */

/** One-time setup (blocking stdin on the IDF console VFS). Safe to call repeatedly. */
void esp_foc_console_init(void);

/** Block until one byte or timeout. @p timeout_ms < 0 waits forever. Returns 0–255 or -1. */
int esp_foc_console_read_byte(int timeout_ms);

/** Write bytes to the console port (no implicit newline). Returns bytes sent or -1. */
int esp_foc_console_write(const char *data, size_t len);

/** Does not return on success. */
void esp_foc_reboot(void);
