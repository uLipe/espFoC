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
#include "xtensa/hal.h"

static const float scale = 0.000001f;
static uint32_t cp0_regs[18];
static uint32_t cp_state;
static portMUX_TYPE spinlock =  portMUX_INITIALIZER_UNLOCKED;

int esp_foc_create_runner(foc_loop_runner runner, void *argument, int priority)
{
    int ret = xTaskCreatePinnedToCore(runner,"", CONFIG_FOC_TASK_STACK_SIZE, argument, priority, NULL, APP_CPU_NUM);

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
void esp_foc_fpu_isr_enter(void)
{
    cp_state = xthal_get_cpenable();

    if(cp_state) {
        xthal_save_cp0(cp0_regs);   
    } else {
        xthal_set_cpenable(1);
    }
}

void esp_foc_fpu_isr_leave(void)
{
    if(cp_state) {
        xthal_restore_cp0(cp0_regs);
    } else {
        xthal_set_cpenable(0);
    }

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
