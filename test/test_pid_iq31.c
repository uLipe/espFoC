/*
 * Unit tests for PID IQ31 path (pid_controller_iq31.h): same dynamics as float PID
 * for per-unit reference/measure in [-1, 1]. Parallel structs vs float controller.
 */
#include <math.h>
#include <unity.h>
#include "espFoC/utils/foc_math.h"
#include "espFoC/utils/pid_controller.h"
#include "espFoC/utils/pid_controller_iq31.h"

#define TOL 0.02f

static void setup_both(esp_foc_pid_controller_t *pf, esp_foc_pid_iq31_t *pi,
                       float kp, float ki, float kd, float dt, float out_min, float out_max, float int_lim)
{
    pf->kp = kp;
    pf->ki = ki;
    pf->kd = kd;
    pf->dt = dt;
    pf->inv_dt = 1.0f / dt;
    pf->min_output_value = out_min;
    pf->max_output_value = out_max;
    pf->integrator_limit = int_lim;
    esp_foc_pid_reset(pf);

    esp_foc_pid_iq31_init_from_float(pi, kp, ki, kd, dt, out_min, out_max, int_lim);
}

static void assert_iq31_near_float(float expected_float, iq31_t got_iq31)
{
    TEST_ASSERT_FLOAT_WITHIN(TOL, expected_float, iq31_to_float(got_iq31));
}

TEST_CASE("esp_foc_pid_iq31: matches float zero error after reset", "[espFoC][pid_iq31]")
{
    esp_foc_pid_controller_t pf;
    esp_foc_pid_iq31_t pi;
    setup_both(&pf, &pi, 1.0f, 0.1f, 0.0f, 0.001f, -10.0f, 10.0f, 5.0f);
    float of = esp_foc_pid_update(&pf, 0.0f, 0.0f);
    iq31_t oi = esp_foc_pid_update_iq31(&pi, 0, 0);
    assert_iq31_near_float(of, oi);
}

TEST_CASE("esp_foc_pid_iq31: proportional matches float (per-unit)", "[espFoC][pid_iq31]")
{
    esp_foc_pid_controller_t pf;
    esp_foc_pid_iq31_t pi;
    setup_both(&pf, &pi, 2.0f, 0.0f, 0.0f, 0.001f, -100.0f, 100.0f, 10.0f);
    float ref = 0.5f;
    float meas = 0.0f;
    float of = esp_foc_pid_update(&pf, ref, meas);
    iq31_t oi = esp_foc_pid_update_iq31(&pi, iq31_from_float(ref), iq31_from_float(meas));
    assert_iq31_near_float(of, oi);
}

TEST_CASE("esp_foc_pid_iq31: output clamp max matches float", "[espFoC][pid_iq31]")
{
    esp_foc_pid_controller_t pf;
    esp_foc_pid_iq31_t pi;
    setup_both(&pf, &pi, 1.0f, 0.0f, 0.0f, 0.001f, -0.1f, 0.1f, 5.0f);
    float ref = 1.0f;
    float meas = 0.0f;
    float of = esp_foc_pid_update(&pf, ref, meas);
    iq31_t oi = esp_foc_pid_update_iq31(&pi, iq31_from_float(ref), iq31_from_float(meas));
    assert_iq31_near_float(of, oi);
}

TEST_CASE("esp_foc_pid_iq31: output clamp min matches float", "[espFoC][pid_iq31]")
{
    esp_foc_pid_controller_t pf;
    esp_foc_pid_iq31_t pi;
    setup_both(&pf, &pi, 1.0f, 0.0f, 0.0f, 0.001f, -0.1f, 0.1f, 5.0f);
    float ref = -1.0f;
    float meas = 0.0f;
    float of = esp_foc_pid_update(&pf, ref, meas);
    iq31_t oi = esp_foc_pid_update_iq31(&pi, iq31_from_float(ref), iq31_from_float(meas));
    assert_iq31_near_float(of, oi);
}

TEST_CASE("esp_foc_pid_iq31: reset clears state to match float", "[espFoC][pid_iq31]")
{
    esp_foc_pid_controller_t pf;
    esp_foc_pid_iq31_t pi;
    setup_both(&pf, &pi, 0.5f, 0.5f, 0.0f, 0.001f, -100.0f, 100.0f, 100.0f);
    for (int i = 0; i < 10; i++) {
        esp_foc_pid_update(&pf, 0.5f, 0.0f);
        esp_foc_pid_update_iq31(&pi, iq31_from_float(0.5f), 0);
    }
    esp_foc_pid_reset(&pf);
    esp_foc_pid_iq31_reset(&pi);
    float of = esp_foc_pid_update(&pf, 0.0f, 0.0f);
    iq31_t oi = esp_foc_pid_update_iq31(&pi, 0, 0);
    assert_iq31_near_float(of, oi);
}

TEST_CASE("esp_foc_pid_iq31: integral buildup matches float", "[espFoC][pid_iq31]")
{
    esp_foc_pid_controller_t pf;
    esp_foc_pid_iq31_t pi;
    setup_both(&pf, &pi, 0.0f, 0.5f, 0.0f, 0.01f, -100.0f, 100.0f, 50.0f);
    float of = 0.0f;
    iq31_t oi = 0;
    for (int i = 0; i < 100; i++) {
        of = esp_foc_pid_update(&pf, 1.0f, 0.0f);
        oi = esp_foc_pid_update_iq31(&pi, IQ31_ONE, 0);
    }
    assert_iq31_near_float(of, oi);
    TEST_ASSERT_TRUE(iq31_to_float(oi) > 0.3f);
}
