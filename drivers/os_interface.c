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
#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __XTENSA__
#include "xtensa/hal.h"
#endif

#include "esp_cpu.h"

static const float scale = 0.000001f;

#ifdef __XTENSA__
static uint32_t cp0_regs[2][18];
static uint32_t cp_state[2];
#endif

static portMUX_TYPE spinlock =  portMUX_INITIALIZER_UNLOCKED;

int esp_foc_create_runner(foc_loop_runner runner, void *argument, int priority)
{

#ifdef __XTENSA__
    int ret = xTaskCreatePinnedToCore(runner,"", CONFIG_FOC_TASK_STACK_SIZE, argument, priority, NULL, APP_CPU_NUM);
#else
    int ret = xTaskCreate(runner,"", CONFIG_FOC_TASK_STACK_SIZE, argument, priority, NULL);
#endif

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

/* the fpu in isr tricks below are required for xtensa based archs: */
void IRAM_ATTR esp_foc_fpu_isr_enter(void)
{

#ifdef __XTENSA__
    int cpu_id = esp_cpu_get_core_id();
    cp_state[cpu_id] = xthal_get_cpenable();

    if(cp_state[cpu_id]) {
        xthal_save_cp0(&cp0_regs[cpu_id][0]);
    } else {
        xthal_set_cpenable(1);
    }
#endif

}

void IRAM_ATTR esp_foc_fpu_isr_leave(void)
{

#ifdef __XTENSA__
    int cpu_id = esp_cpu_get_core_id();

    if(cp_state[cpu_id]) {
        xthal_restore_cp0(&cp0_regs[cpu_id][0]);
    } else {
        xthal_set_cpenable(0);
    }
#endif

}
IRAM_ATTR void esp_foc_critical_enter(void)
{
    portENTER_CRITICAL(&spinlock);
}

IRAM_ATTR void esp_foc_critical_leave(void)
{
    portEXIT_CRITICAL(&spinlock);
}

IRAM_ATTR esp_foc_event_handle_t esp_foc_get_event_handle(void)
{
    return ((esp_foc_event_handle_t) xTaskGetCurrentTaskHandle());
}

IRAM_ATTR void esp_foc_wait_notifier(void)
{
    ulTaskNotifyTake(pdFALSE ,portMAX_DELAY);
}

IRAM_ATTR void esp_foc_send_notification(esp_foc_event_handle_t handle)
{
    BaseType_t wake;
    vTaskNotifyGiveFromISR((TaskHandle_t)handle, &wake);
    if (wake == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}
