/*
 * Unit tests for PID (pid_controller.h): Q16.16 I/O vs esp_foc_pid_update_float (same path).
 */
#include <math.h>
#include <stdlib.h>
#include <unity.h>
#include "espFoC/utils/pid_controller.h"
#include "espFoC/utils/esp_foc_q16.h"

#define TOL 0.02f
#define TOL_STRICT 0.03f

static void setup_both(esp_foc_pid_controller_t *pf, esp_foc_pid_controller_t *pi,
                       float kp, float ki, float kd, float dt, float out_min, float out_max, float int_lim)
{
    esp_foc_pid_init_from_float(pf, kp, ki, kd, dt, out_min, out_max, int_lim);
    esp_foc_pid_init_from_float(pi, kp, ki, kd, dt, out_min, out_max, int_lim);
}

static void assert_q16_near_float(float expected_float, q16_t got)
{
    TEST_ASSERT_FLOAT_WITHIN(TOL, expected_float, q16_to_float(got));
}

TEST_CASE("esp_foc_pid: matches float zero error after reset", "[espFoC][pid_iq31]")
{
    esp_foc_pid_controller_t pf;
    esp_foc_pid_controller_t pi;
    setup_both(&pf, &pi, 1.0f, 0.1f, 0.0f, 0.001f, -10.0f, 10.0f, 5.0f);
    float of = esp_foc_pid_update_float(&pf, 0.0f, 0.0f);
    q16_t oi = esp_foc_pid_update(&pi, 0, 0);
    assert_q16_near_float(of, oi);
}

TEST_CASE("esp_foc_pid: proportional matches float (per-unit)", "[espFoC][pid_iq31]")
{
    esp_foc_pid_controller_t pf;
    esp_foc_pid_controller_t pi;
    setup_both(&pf, &pi, 2.0f, 0.0f, 0.0f, 0.001f, -100.0f, 100.0f, 10.0f);
    float ref = 0.5f;
    float meas = 0.0f;
    float of = esp_foc_pid_update_float(&pf, ref, meas);
    q16_t oi = esp_foc_pid_update(&pi, q16_from_float(ref), q16_from_float(meas));
    assert_q16_near_float(of, oi);
}

TEST_CASE("esp_foc_pid: output clamp max matches float", "[espFoC][pid_iq31]")
{
    esp_foc_pid_controller_t pf;
    esp_foc_pid_controller_t pi;
    setup_both(&pf, &pi, 1.0f, 0.0f, 0.0f, 0.001f, -0.1f, 0.1f, 5.0f);
    float ref = 1.0f;
    float meas = 0.0f;
    float of = esp_foc_pid_update_float(&pf, ref, meas);
    q16_t oi = esp_foc_pid_update(&pi, q16_from_float(ref), q16_from_float(meas));
    assert_q16_near_float(of, oi);
}

TEST_CASE("esp_foc_pid: output clamp min matches float", "[espFoC][pid_iq31]")
{
    esp_foc_pid_controller_t pf;
    esp_foc_pid_controller_t pi;
    setup_both(&pf, &pi, 1.0f, 0.0f, 0.0f, 0.001f, -0.1f, 0.1f, 5.0f);
    float ref = -1.0f;
    float meas = 0.0f;
    float of = esp_foc_pid_update_float(&pf, ref, meas);
    q16_t oi = esp_foc_pid_update(&pi, q16_from_float(ref), q16_from_float(meas));
    assert_q16_near_float(of, oi);
}

TEST_CASE("esp_foc_pid: reset clears state to match float", "[espFoC][pid_iq31]")
{
    esp_foc_pid_controller_t pf;
    esp_foc_pid_controller_t pi;
    setup_both(&pf, &pi, 0.5f, 0.5f, 0.0f, 0.001f, -100.0f, 100.0f, 100.0f);
    for (int i = 0; i < 10; i++) {
        esp_foc_pid_update_float(&pf, 0.5f, 0.0f);
        esp_foc_pid_update(&pi, q16_from_float(0.5f), 0);
    }
    esp_foc_pid_reset(&pf);
    esp_foc_pid_reset(&pi);
    float of = esp_foc_pid_update_float(&pf, 0.0f, 0.0f);
    q16_t oi = esp_foc_pid_update(&pi, 0, 0);
    assert_q16_near_float(of, oi);
}

TEST_CASE("esp_foc_pid: integral buildup", "[espFoC][pid_iq31]")
{
    esp_foc_pid_controller_t pf;
    esp_foc_pid_controller_t pi;
    setup_both(&pf, &pi, 0.0f, 0.5f, 0.0f, 0.01f, -100.0f, 100.0f, 50.0f);
    float of = 0.0f;
    q16_t oi = 0;
    for (int i = 0; i < 100; i++) {
        of = esp_foc_pid_update_float(&pf, 1.0f, 0.0f);
        oi = esp_foc_pid_update(&pi, Q16_ONE, 0);
    }
    assert_q16_near_float(of, oi);
    TEST_ASSERT_TRUE(q16_to_float(oi) > 0.3f);
}

TEST_CASE("esp_foc_pid: PI first step", "[espFoC][pid_iq31]")
{
    esp_foc_pid_controller_t pf;
    esp_foc_pid_controller_t pi;
    setup_both(&pf, &pi, 0.5f, 2.0f, 0.0f, 0.01f, -100.0f, 100.0f, 100.0f);
    float of = esp_foc_pid_update_float(&pf, 1.0f, 0.0f);
    q16_t oi = esp_foc_pid_update(&pi, Q16_ONE, 0);
    assert_q16_near_float(of, oi);
    TEST_ASSERT_TRUE(llabs((long long)pf.integrator - (long long)pi.integrator) <= 256);
}

TEST_CASE("esp_foc_pid: integrator clamps at limit", "[espFoC][pid_iq31]")
{
    esp_foc_pid_controller_t pf;
    esp_foc_pid_controller_t pi;
    const float int_lim = 0.4f;
    setup_both(&pf, &pi, 0.0f, 30.0f, 0.0f, 0.01f, -100.0f, 100.0f, int_lim);
    for (int i = 0; i < 400; i++) {
        esp_foc_pid_update_float(&pf, 1.0f, 0.0f);
        esp_foc_pid_update(&pi, Q16_ONE, 0);
    }
    TEST_ASSERT_TRUE(llabs((long long)pf.integrator - (long long)pi.integrator) <= 256);
}

TEST_CASE("esp_foc_pid: output saturated integrator still bounded", "[espFoC][pid_iq31]")
{
    esp_foc_pid_controller_t pf;
    esp_foc_pid_controller_t pi;
    setup_both(&pf, &pi, 0.0f, 15.0f, 0.0f, 0.01f, -0.4f, 0.4f, 8.0f);
    for (int i = 0; i < 150; i++) {
        esp_foc_pid_update_float(&pf, 1.0f, 0.0f);
        esp_foc_pid_update(&pi, Q16_ONE, 0);
    }
    TEST_ASSERT_TRUE(llabs((long long)pf.integrator - (long long)pi.integrator) <= 512);
    float of = esp_foc_pid_update_float(&pf, 1.0f, 0.0f);
    q16_t oi = esp_foc_pid_update(&pi, Q16_ONE, 0);
    assert_q16_near_float(of, oi);
}

TEST_CASE("esp_foc_pid: negative error", "[espFoC][pid_iq31]")
{
    esp_foc_pid_controller_t pf;
    esp_foc_pid_controller_t pi;
    setup_both(&pf, &pi, 1.0f, 0.0f, 0.0f, 0.001f, -100.0f, 100.0f, 10.0f);
    float of = esp_foc_pid_update_float(&pf, 0.0f, 0.5f);
    q16_t oi = esp_foc_pid_update(&pi, 0, q16_from_float(0.5f));
    assert_q16_near_float(of, oi);
}

TEST_CASE("esp_foc_pid: derivative step", "[espFoC][pid_iq31]")
{
    esp_foc_pid_controller_t pf;
    esp_foc_pid_controller_t pi;
    setup_both(&pf, &pi, 0.0f, 0.0f, 0.01f, 0.1f, -1000.0f, 1000.0f, 100.0f);
    esp_foc_pid_update_float(&pf, 0.0f, 0.0f);
    esp_foc_pid_update(&pi, 0, 0);
    float of = esp_foc_pid_update_float(&pf, 1.0f, 0.0f);
    q16_t oi = esp_foc_pid_update(&pi, Q16_ONE, 0);
    assert_q16_near_float(of, oi);
}
