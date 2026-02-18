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

#include <sys/cdefs.h>
#include <unistd.h>
#include "espFoC/esp_foc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_cpu.h"
#include "driver/gpio.h"

static const float scale = 0.000001f;

static portMUX_TYPE spinlock =  portMUX_INITIALIZER_UNLOCKED;
static int debug_pin_internal = -1;

int esp_foc_create_runner(foc_loop_runner runner, void *argument, int priority)
{
    /* Allows cretion of low-priority tasks */
    int cpu_num = 1;
    if(priority < 0) {
        priority = (configMAX_PRIORITIES - 8);
        cpu_num = PRO_CPU_NUM;
    }

    int ret = xTaskCreatePinnedToCore(runner,"", CONFIG_FOC_TASK_STACK_SIZE, argument, configMAX_PRIORITIES - priority, NULL, cpu_num);
    if (ret != pdPASS) {
        return -ESP_ERR_NO_MEM;
    }

    return 0;
}

void esp_foc_sleep_ms(int sleep_ms)
{
    usleep(sleep_ms * 1000);
}

void esp_foc_runner_yield(void)
{
    taskYIELD();
}

float esp_foc_now_seconds(void)
{
    float now = (float)esp_timer_get_time();
    return(now * scale);
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

void esp_foc_send_notification(esp_foc_event_handle_t handle)
{
    BaseType_t wake;
    vTaskNotifyGiveFromISR((TaskHandle_t)handle, &wake);
    if (wake == pdTRUE) {
        portYIELD_FROM_ISR();
    }
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
