/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include <sys/cdefs.h>
#include <stdlib.h>
#include <unistd.h>
#include "espFoC/esp_foc.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_cpu.h"
#include "driver/gpio.h"

static portMUX_TYPE spinlock =  portMUX_INITIALIZER_UNLOCKED;
static int debug_pin_internal = -1;

int esp_foc_create_runner(foc_loop_runner runner, void *argument, int priority,
                          void **out_task_handle)
{
#if CONFIG_FREERTOS_NUMBER_OF_CORES > 1
    int cpu_num = 1;
#else
    int cpu_num = 0;
#endif
    if (priority < 0) {
        priority = (configMAX_PRIORITIES - 8);
        cpu_num = PRO_CPU_NUM;
    }

    TaskHandle_t created = NULL;
    int ret = xTaskCreatePinnedToCore(runner, "", CONFIG_FOC_TASK_STACK_SIZE,
                                      argument, configMAX_PRIORITIES - priority,
                                      &created, cpu_num);
    if (ret != pdPASS) {
        return -ESP_ERR_NO_MEM;
    }
    if (out_task_handle != NULL) {
        *out_task_handle = (void *)created;
    }
    return 0;
}

bool esp_foc_runner_is_alive(void *task_handle)
{
    if (task_handle == NULL) {
        return false;
    }
    eTaskState st = eTaskGetState((TaskHandle_t)task_handle);
    return st != eDeleted && st != eInvalid;
}

void esp_foc_runner_wake(void *task_handle)
{
    if (task_handle != NULL) {
        xTaskNotifyGive((TaskHandle_t)task_handle);
    }
}

bool esp_foc_in_task_context(void)
{
    return xTaskGetSchedulerState() == taskSCHEDULER_RUNNING &&
           xTaskGetCurrentTaskHandle() != NULL && !xPortInIsrContext();
}

void esp_foc_sleep_ms(int sleep_ms)
{
    usleep(sleep_ms * 1000);
}

void esp_foc_runner_yield(void)
{
    taskYIELD();
}

uint64_t esp_foc_now_useconds(void)
{
    return esp_timer_get_time();
}

void esp_foc_critical_enter(void)
{
    portENTER_CRITICAL(&spinlock);
}

void esp_foc_critical_leave(void)
{
    portEXIT_CRITICAL(&spinlock);
}

esp_foc_event_handle_t esp_foc_get_event_handle(void)
{
    return ((esp_foc_event_handle_t) xTaskGetCurrentTaskHandle());
}

void esp_foc_wait_notifier(void)
{
    ulTaskNotifyTake(pdFALSE ,portMAX_DELAY);
}

void esp_foc_send_notification_from_isr(esp_foc_event_handle_t handle)
{
    BaseType_t wake;
    vTaskNotifyGiveFromISR((TaskHandle_t)handle, &wake);
    if (wake == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void esp_foc_send_notification(esp_foc_event_handle_t handle)
{
    xTaskNotifyGive((TaskHandle_t)handle);
}

int esp_foc_debug_pin_init(int debug_pin)
{
    gpio_config_t drv_en_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << debug_pin,
    };
    gpio_config(&drv_en_config);
    gpio_set_level(debug_pin, false);

    debug_pin_internal = debug_pin;

    return 0;
}

int esp_foc_debug_pin_set(void)
{
    gpio_set_level(debug_pin_internal, true);

    return 0;
}

int esp_foc_debug_pin_clear(void)
{
    gpio_set_level(debug_pin_internal, false);

    return 0;
}

void esp_foc_runner_delete_self(void)
{
    vTaskDelete(NULL);
}

uint32_t esp_foc_ms_to_wait_ticks(unsigned ms)
{
    return (uint32_t)pdMS_TO_TICKS(ms);
}

int esp_foc_task_spawn(foc_loop_runner fn, void *arg, size_t stack_bytes,
                       int freertos_priority, void **out_task_handle_opt)
{
    if (fn == NULL) {
        return -1;
    }
    TaskHandle_t h = NULL;
    BaseType_t ok = xTaskCreate(fn, "espfoc", (configSTACK_DEPTH_TYPE)stack_bytes, arg,
                                (UBaseType_t)freertos_priority, &h);
    if (ok != pdPASS) {
        return -2;
    }
    if (out_task_handle_opt != NULL) {
        *out_task_handle_opt = (void *)h;
    }
    return 0;
}

struct esp_foc_timer {
    esp_timer_handle_t h;
    esp_foc_timer_callback_t cb;
    void *arg;
};

static void esp_foc_timer_trampoline(void *p)
{
    esp_foc_timer_t *t = (esp_foc_timer_t *)p;
    if (t->cb != NULL) {
        t->cb(t->arg);
    }
}

int esp_foc_timer_create(esp_foc_timer_callback_t cb, void *arg, esp_foc_timer_t **out)
{
    if (cb == NULL || out == NULL) {
        return -1;
    }
    esp_foc_timer_t *t = (esp_foc_timer_t *)calloc(1, sizeof(*t));
    if (t == NULL) {
        return -2;
    }
    t->cb = cb;
    t->arg = arg;
    const esp_timer_create_args_t args = {
        .callback = &esp_foc_timer_trampoline,
        .arg = t,
        .name = "espfoc_t",
        .dispatch_method = ESP_TIMER_TASK,
    };
    esp_err_t e = esp_timer_create(&args, &t->h);
    if (e != ESP_OK) {
        free(t);
        return (int)e;
    }
    *out = t;
    return 0;
}

void esp_foc_timer_destroy(esp_foc_timer_t *t)
{
    if (t == NULL) {
        return;
    }
    (void)esp_timer_stop(t->h);
    (void)esp_timer_delete(t->h);
    free(t);
}

int esp_foc_timer_arm_oneshot_us(esp_foc_timer_t *t, uint64_t period_us)
{
    if (t == NULL || period_us == 0ULL) {
        return -1;
    }
    esp_err_t e = esp_timer_start_once(t->h, period_us);
    return (e == ESP_OK) ? 0 : (int)e;
}

int esp_foc_timer_arm_periodic_us(esp_foc_timer_t *t, uint64_t period_us)
{
    if (t == NULL || period_us == 0ULL) {
        return -1;
    }
    (void)esp_timer_stop(t->h);
    esp_err_t e = esp_timer_start_periodic(t->h, period_us);
    return (e == ESP_OK) ? 0 : (int)e;
}

void esp_foc_timer_cancel(esp_foc_timer_t *t)
{
    if (t == NULL) {
        return;
    }
    (void)esp_timer_stop(t->h);
}
