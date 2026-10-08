/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * IDF / FreeRTOS port of espFoC OSAL. All FreeRTOS usage for the component
 * lives here — other layers call esp_foc_* only.
 */
#include "espFoC/osal/esp_foc_osal.h"

#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#ifndef CONFIG_ESP_FOC_MAX_MUTEXES
#define CONFIG_ESP_FOC_MAX_MUTEXES 2
#endif

static portMUX_TYPE s_crit = portMUX_INITIALIZER_UNLOCKED;

struct esp_foc_mutex {
    SemaphoreHandle_t h;
    bool used;
};

static esp_foc_mutex_t s_mutex_pool[CONFIG_ESP_FOC_MAX_MUTEXES];

int esp_foc_task_max_priority(void)
{
    return (int)configMAX_PRIORITIES - 1;
}

int esp_foc_task_spawn(esp_foc_task_fn_t fn,
                       void *arg,
                       const char *name,
                       size_t stack_bytes,
                       int priority,
                       esp_foc_task_handle_t *out_handle_opt)
{
    if (fn == NULL || stack_bytes == 0U) {
        return -1;
    }
    int prio_max = esp_foc_task_max_priority();
    if (priority < 0) {
        priority = 0;
    }
    if (priority > prio_max) {
        priority = prio_max;
    }
    const char *nm = (name != NULL && name[0] != '\0') ? name : "espfoc";
    TaskHandle_t h = NULL;
    BaseType_t ok = xTaskCreate(fn,
                                nm,
                                (configSTACK_DEPTH_TYPE)stack_bytes,
                                arg,
                                (UBaseType_t)priority,
                                &h);
    if (ok != pdPASS) {
        return -2;
    }
    if (out_handle_opt != NULL) {
        *out_handle_opt = (esp_foc_task_handle_t)h;
    }
    return 0;
}

bool esp_foc_task_is_alive(esp_foc_task_handle_t handle)
{
    if (handle == NULL) {
        return false;
    }
    eTaskState st = eTaskGetState((TaskHandle_t)handle);
    return st != eDeleted && st != eInvalid;
}

void esp_foc_task_yield(void)
{
    taskYIELD();
}

void esp_foc_task_delete_self(void)
{
    vTaskDelete(NULL);
}

void esp_foc_sleep_ms(uint32_t ms)
{
    if (ms == 0U) {
        return;
    }
    TickType_t ticks = pdMS_TO_TICKS(ms);
    if (ticks == 0) {
        ticks = 1;
    }
    vTaskDelay(ticks);
}

uint32_t esp_foc_ms_to_ticks(uint32_t ms)
{
    return (uint32_t)pdMS_TO_TICKS(ms);
}

uint64_t esp_foc_now_us(void)
{
    return (uint64_t)esp_timer_get_time();
}

void esp_foc_reboot(void)
{
    esp_restart();
}

bool esp_foc_in_task_context(void)
{
    return xTaskGetSchedulerState() == taskSCHEDULER_RUNNING &&
           xTaskGetCurrentTaskHandle() != NULL &&
           !xPortInIsrContext();
}

bool esp_foc_in_isr_context(void)
{
    return xPortInIsrContext();
}

void esp_foc_critical_enter(void)
{
    portENTER_CRITICAL(&s_crit);
}

void esp_foc_critical_leave(void)
{
    portEXIT_CRITICAL(&s_crit);
}

esp_foc_event_handle_t esp_foc_event_handle_self(void)
{
    return (esp_foc_event_handle_t)xTaskGetCurrentTaskHandle();
}

void esp_foc_event_wait(void)
{
    (void)ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
}

bool esp_foc_event_wait_ms(uint32_t timeout_ms)
{
    TickType_t ticks = (timeout_ms == UINT32_MAX)
                           ? portMAX_DELAY
                           : pdMS_TO_TICKS(timeout_ms);
    if (timeout_ms != UINT32_MAX && ticks == 0) {
        ticks = 1;
    }
    return ulTaskNotifyTake(pdFALSE, ticks) != 0;
}

void esp_foc_event_clear(void)
{
    while (ulTaskNotifyTake(pdTRUE, 0) != 0) {
    }
}

void esp_foc_event_post(esp_foc_event_handle_t handle)
{
    if (handle == NULL) {
        return;
    }
    (void)xTaskNotifyGive((TaskHandle_t)handle);
}

void esp_foc_event_post_from_isr(esp_foc_event_handle_t handle)
{
    if (handle == NULL) {
        return;
    }
    BaseType_t wake = pdFALSE;
    vTaskNotifyGiveFromISR((TaskHandle_t)handle, &wake);
    if (wake == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void esp_foc_event_post_auto(esp_foc_event_handle_t handle)
{
    if (esp_foc_in_isr_context()) {
        esp_foc_event_post_from_isr(handle);
    } else {
        esp_foc_event_post(handle);
    }
}

int esp_foc_mutex_create(esp_foc_mutex_t **out)
{
    if (out == NULL) {
        return -1;
    }
    for (int i = 0; i < CONFIG_ESP_FOC_MAX_MUTEXES; i++) {
        if (!s_mutex_pool[i].used) {
            if (s_mutex_pool[i].h == NULL) {
                s_mutex_pool[i].h = xSemaphoreCreateMutex();
                if (s_mutex_pool[i].h == NULL) {
                    return -2;
                }
            }
            s_mutex_pool[i].used = true;
            *out = &s_mutex_pool[i];
            return 0;
        }
    }
    return -3;
}

void esp_foc_mutex_destroy(esp_foc_mutex_t *mutex)
{
    if (mutex == NULL) {
        return;
    }
    mutex->used = false;
}

void esp_foc_mutex_lock(esp_foc_mutex_t *mutex)
{
    if (mutex != NULL && mutex->h != NULL) {
        (void)xSemaphoreTake(mutex->h, portMAX_DELAY);
    }
}

void esp_foc_mutex_unlock(esp_foc_mutex_t *mutex)
{
    if (mutex != NULL && mutex->h != NULL) {
        (void)xSemaphoreGive(mutex->h);
    }
}
