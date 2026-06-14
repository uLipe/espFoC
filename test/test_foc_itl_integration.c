/*
 * Integration tests for esp_foc_in_the_loop factory (FITL build).
 */
#include "sdkconfig.h"
#if CONFIG_ESP_FOC_FITL

#include <unity.h>
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "espFoC/esp_foc_in_the_loop.h"
#include "espFoC/esp_foc.h"
#include "espFoC/esp_foc_err.h"
#include "espFoC/utils/esp_foc_q16.h"

/* At CONFIG_FREERTOS_HZ=100 each vTaskDelay(1) is 10 ms → 100 ticks ≈ 1 s. */
#define FITL_ITL_POLL_TICKS  100

static volatile int s_tick_cb_count;

static void IRAM_ATTR fitl_count_tick(void *arg)
{
    (void)arg;
    s_tick_cb_count++;
}

void tearDown(void)
{
    esp_foc_in_the_loop_destroy();
    vTaskDelay(pdMS_TO_TICKS(10));
}

TEST_CASE("fitl: create returns two interfaces", "[espFoC][foc_itl]")
{
    esp_foc_in_the_loop_config_t cfg = {
        .r_ohm = 1.0f,
        .l_henry = 0.002f,
        .j_kgm2 = 5e-5f,
        .b_nms = 1e-5f,
        .kt_nm_per_a = 1e-4f,
        .pole_pairs = 7,
        .vdc_q16 = q16_from_float(12.0f),
        .i_max_a = 10.0f,
        .locked_rotor = true,
        .connection_delta = false,
        .pwm_hz = 10000,
    };
    esp_foc_in_the_loop_handles_t h = {0};

    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_in_the_loop_create(&cfg, &h));
    TEST_ASSERT_NOT_NULL(h.inverter);
    TEST_ASSERT_NOT_NULL(h.encoder);
    TEST_ASSERT_NOT_NULL(h.inverter->fetch_isensors);
    TEST_ASSERT_EQUAL(10000u, h.inverter->get_inverter_pwm_rate(h.inverter));
}

TEST_CASE("fitl: tick invokes inverter callback", "[espFoC][foc_itl]")
{
    esp_foc_in_the_loop_config_t cfg = {
        .locked_rotor = true,
        .pwm_hz = 10000,
    };
    esp_foc_in_the_loop_handles_t h = {0};
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_in_the_loop_create(&cfg, &h));

    h.inverter->set_inverter_callback(h.inverter, fitl_count_tick, NULL);
    s_tick_cb_count = 0;

    for (int i = 0; i < FITL_ITL_POLL_TICKS && s_tick_cb_count == 0; ++i) {
        vTaskDelay(1);
    }

    TEST_ASSERT_GREATER_THAN_MESSAGE(0, s_tick_cb_count,
                                   "FITL GPTimer tick did not invoke inverter callback");
}

TEST_CASE("fitl: second create without destroy fails", "[espFoC][foc_itl]")
{
    esp_foc_in_the_loop_config_t cfg = { .locked_rotor = true, .pwm_hz = 10000 };
    esp_foc_in_the_loop_handles_t h = {0};
    TEST_ASSERT_EQUAL(ESP_FOC_OK, esp_foc_in_the_loop_create(&cfg, &h));
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG, esp_foc_in_the_loop_create(&cfg, &h));
}

#endif /* CONFIG_ESP_FOC_FITL */
