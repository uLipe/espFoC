/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include <sys/cdefs.h>
#include <unistd.h>
#include "espFoC/esp_foc.h"
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
    int cpu_num = 1;
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
