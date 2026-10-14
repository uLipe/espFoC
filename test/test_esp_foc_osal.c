/*
 * OSAL task placement: stack tasks on CONFIG_ESP_FOC_CORE_ID.
 */
#include "unity.h"
#include "esp_cpu.h"
#include "sdkconfig.h"
#include "espFoC/osal/esp_foc_osal.h"

typedef struct {
    esp_foc_event_handle_t waiter;
    volatile int core;
} core_probe_t;

static void probe_task(void *arg)
{
    core_probe_t *p = (core_probe_t *)arg;
    p->core = esp_cpu_get_core_id();
    esp_foc_event_post(p->waiter);
    esp_foc_task_delete_self();
}

TEST_CASE("stack task runs on the configured core", "[espFoC][osal]")
{
    esp_foc_event_clear();
    for (int i = 0; i < 4; i++) {
        core_probe_t p = { .waiter = esp_foc_event_handle_self(), .core = -2 };
        TEST_ASSERT_EQUAL_INT(0, esp_foc_task_spawn_stack(probe_task, &p, "probe", 2048,
                                                          esp_foc_task_max_priority() - 1, NULL));
        TEST_ASSERT_TRUE(esp_foc_event_wait_ms(1000));
#if CONFIG_ESP_FOC_CORE_ID >= 0
        TEST_ASSERT_EQUAL_INT(CONFIG_ESP_FOC_CORE_ID, p.core);
#else
        TEST_ASSERT_TRUE(p.core >= 0);
#endif
        esp_foc_sleep_ms(2);
    }
}

TEST_CASE("plain task spawn still runs", "[espFoC][osal]")
{
    esp_foc_event_clear();
    core_probe_t p = { .waiter = esp_foc_event_handle_self(), .core = -2 };
    TEST_ASSERT_EQUAL_INT(0, esp_foc_task_spawn(probe_task, &p, "probe", 2048, 1, NULL));
    TEST_ASSERT_TRUE(esp_foc_event_wait_ms(1000));
    TEST_ASSERT_TRUE(p.core >= 0);
    esp_foc_sleep_ms(2);
}

TEST_CASE("stack task spawn rejects a null function", "[espFoC][osal]")
{
    TEST_ASSERT_NOT_EQUAL(0, esp_foc_task_spawn_stack(NULL, NULL, "x", 2048, 1, NULL));
    TEST_ASSERT_NOT_EQUAL(0, esp_foc_task_spawn_stack(probe_task, NULL, "x", 0, 1, NULL));
}
