/*
 * Unit tests for pid_controller.h (Q16 path).
 */
#include <math.h>
#include <stdlib.h>
#include <unity.h>
#include "espFoC/utils/pid_controller.h"
#include "espFoC/utils/esp_foc_q16.h"

static void init_pi(esp_foc_pid_controller_t *p,
                     float kp, float ki, float dt,
                     float out_min, float out_max, float int_lim)
{
    esp_foc_pid_init_from_float(p, kp, ki, 0.0f, 0.0f, 1.0f, dt, out_min, out_max,
                                int_lim);
}

TEST_CASE("esp_foc_pid: zero error steady output", "[espFoC][pid_q16]")
{
    esp_foc_pid_controller_t p;
    init_pi(&p, 1.0f, 0.1f, 0.001f, -10.0f, 10.0f, 5.0f);
    q16_t o = esp_foc_pid_update(&p, 0, 0);
    TEST_ASSERT_INT32_WITHIN(500, 0, (int32_t)o);
}

TEST_CASE("esp_foc_pid: proportional step", "[espFoC][pid_q16]")
{
    esp_foc_pid_controller_t p;
    init_pi(&p, 2.0f, 0.0f, 0.001f, -100.0f, 100.0f, 10.0f);
    q16_t o = esp_foc_pid_update(&p, q16_from_float(0.5f), 0);
    TEST_ASSERT_INT32_WITHIN(2000, q16_from_float(1.0f), (int32_t)o);
}

TEST_CASE("esp_foc_pid: output clamp max", "[espFoC][pid_q16]")
{
    esp_foc_pid_controller_t p;
    init_pi(&p, 1.0f, 0.0f, 0.001f, -0.1f, 0.1f, 5.0f);
    q16_t o = esp_foc_pid_update(&p, q16_from_float(1.0f), 0);
    TEST_ASSERT_EQUAL_INT32(q16_from_float(0.1f), o);
}

TEST_CASE("esp_foc_pid: output clamp min", "[espFoC][pid_q16]")
{
    esp_foc_pid_controller_t p;
    init_pi(&p, 1.0f, 0.0f, 0.001f, -0.1f, 0.1f, 5.0f);
    q16_t o = esp_foc_pid_update(&p, q16_from_float(-1.0f), 0);
    TEST_ASSERT_EQUAL_INT32(q16_from_float(-0.1f), o);
}

TEST_CASE("esp_foc_pid: reset clears integrator", "[espFoC][pid_q16]")
{
    esp_foc_pid_controller_t p;
    init_pi(&p, 0.5f, 0.5f, 0.001f, -100.0f, 100.0f, 100.0f);
    for (int i = 0; i < 10; i++) {
        esp_foc_pid_update(&p, q16_from_float(0.5f), 0);
    }
    esp_foc_pid_reset(&p);
    q16_t o = esp_foc_pid_update(&p, 0, 0);
    TEST_ASSERT_INT32_WITHIN(2000, 0, (int32_t)o);
}

TEST_CASE("esp_foc_pid: integral buildup", "[espFoC][pid_q16]")
{
    esp_foc_pid_controller_t p;
    init_pi(&p, 0.0f, 0.5f, 0.01f, -100.0f, 100.0f, 50.0f);
    q16_t o = 0;
    for (int i = 0; i < 100; i++) {
        o = esp_foc_pid_update(&p, Q16_ONE, 0);
    }
    TEST_ASSERT_TRUE(q16_to_float(o) > 0.3f);
}

TEST_CASE("esp_foc_pid: bypass kff passthrough", "[espFoC][pid_q16]")
{
    esp_foc_pid_controller_t p;
    esp_foc_pid_init_from_float(&p, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.001f,
                                -1.0f, 1.0f, 1.0f);
    q16_t o = esp_foc_pid_update(&p, q16_from_float(0.25f), 0);
    TEST_ASSERT_INT32_WITHIN(500, q16_from_float(0.25f), (int32_t)o);
}

TEST_CASE("esp_foc_pid: ff plus P", "[espFoC][pid_q16]")
{
    esp_foc_pid_controller_t p;
    esp_foc_pid_init_from_float(&p, 1.0f, 0.0f, 0.0f, 0.5f, 1.0f, 0.001f,
                                -2.0f, 2.0f, 2.0f);
    /* ref=1, meas=0 -> err=1, P=1, FF=0.5 -> unsat 1.5 */
    q16_t o = esp_foc_pid_update(&p, Q16_ONE, 0);
    TEST_ASSERT_INT32_WITHIN(3000, q16_from_float(1.5f), (int32_t)o);
}

TEST_CASE("esp_foc_pid: derivative kick", "[espFoC][pid_q16]")
{
    esp_foc_pid_controller_t p;
    esp_foc_pid_init_from_float(&p, 0.0f, 0.0f, 0.01f, 0.0f, 1.0f, 0.1f,
                                -1000.0f, 1000.0f, 100.0f);
    esp_foc_pid_update(&p, 0, 0);
    q16_t o = esp_foc_pid_update(&p, Q16_ONE, 0);
    TEST_ASSERT_TRUE((int32_t)o != 0);
}
