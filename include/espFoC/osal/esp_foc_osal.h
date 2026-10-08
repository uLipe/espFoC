/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*esp_foc_task_fn_t)(void *arg);
typedef void *esp_foc_task_handle_t;
typedef void *esp_foc_event_handle_t;

typedef struct esp_foc_mutex esp_foc_mutex_t;

/** Highest FreeRTOS task priority value usable on this port (configMAX_PRIORITIES - 1). */
int esp_foc_task_max_priority(void);

/**
 * Spawn a task. @p priority is a FreeRTOS priority (0 .. max).
 * @return 0 on success, negative on failure.
 */
int esp_foc_task_spawn(esp_foc_task_fn_t fn,
                       void *arg,
                       const char *name,
                       size_t stack_bytes,
                       int priority,
                       esp_foc_task_handle_t *out_handle_opt);

bool esp_foc_task_is_alive(esp_foc_task_handle_t handle);
void esp_foc_task_yield(void);
void esp_foc_task_delete_self(void);

void esp_foc_sleep_ms(uint32_t ms);
uint32_t esp_foc_ms_to_ticks(uint32_t ms);
uint64_t esp_foc_now_us(void);

/** Chip reset (IDF `esp_restart`). Does not return. */
void esp_foc_reboot(void);

bool esp_foc_in_task_context(void);
bool esp_foc_in_isr_context(void);

void esp_foc_critical_enter(void);
void esp_foc_critical_leave(void);

/** Opaque event = current task handle (FreeRTOS task notification). */
esp_foc_event_handle_t esp_foc_event_handle_self(void);

/** Block until a matching post (forever). */
void esp_foc_event_wait(void);

/**
 * Block until post or timeout.
 * @return true if notified, false on timeout.
 */
bool esp_foc_event_wait_ms(uint32_t timeout_ms);

/** Drain pending notifications without blocking. */
void esp_foc_event_clear(void);

void esp_foc_event_post(esp_foc_event_handle_t handle);
void esp_foc_event_post_from_isr(esp_foc_event_handle_t handle);
/** Chooses task vs ISR post from current context. */
void esp_foc_event_post_auto(esp_foc_event_handle_t handle);

/** Static-pool mutex. @return 0 on success. */
int esp_foc_mutex_create(esp_foc_mutex_t **out);
void esp_foc_mutex_destroy(esp_foc_mutex_t *mutex);
void esp_foc_mutex_lock(esp_foc_mutex_t *mutex);
void esp_foc_mutex_unlock(esp_foc_mutex_t *mutex);

#ifdef __cplusplus
}
#endif
