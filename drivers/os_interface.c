#include <sys/cdefs.h>
#include <unistd.h>
#include "espFoC/esp_foc.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const float scale = 0.000001f;

int esp_foc_create_runner(foc_loop_runner runner, void *argument)
{
    int ret = xTaskCreatePinnedToCore(runner,"", CONFIG_FOC_TASK_STACK_SIZE, argument, CONFIG_FOC_TASK_PRIORITY, NULL, PRO_CPU_NUM);

    if (ret != pdPASS) {
        return -ESP_ERR_NO_MEM;
    }

    return 0;
}

void esp_foc_sleep_ms(int sleep_ms)
{
    usleep(sleep_ms);
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
