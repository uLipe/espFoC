/*
 * Unit tests for FOC PID controller (inline in pid_controller.h).
 * No hardware; pure control logic with fixed dt.
 */
#include <math.h>
#include <unity.h>
#include "espFoC/utils/foc_math.h"
#include "espFoC/utils/pid_controller.h"

#define FLOAT_TOL 1e-4f

static void setup_pid(esp_foc_pid_controller_t *pid, float kp, float ki, float kd, float dt, float out_min, float out_max, float int_lim)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->inv_dt = 1.0f / dt;
    pid->min_output_value = out_min;
    pid->max_output_value = out_max;
    pid->integrator_limit = int_lim;
    esp_foc_pid_reset(pid);
}

TEST_CASE("esp_foc_pid: zero error gives zero output when reset", "[espFoC][pid]")
{
    esp_foc_pid_controller_t pid;
    setup_pid(&pid, 1.0f, 0.1f, 0.0f, 0.001f, -10.0f, 10.0f, 5.0f);
    float out = esp_foc_pid_update(&pid, 0.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.0f, out);
}

TEST_CASE("esp_foc_pid: proportional only tracks reference", "[espFoC][pid]")
{
    esp_foc_pid_controller_t pid;
    setup_pid(&pid, 2.0f, 0.0f, 0.0f, 0.001f, -100.0f, 100.0f, 10.0f);
    float out = esp_foc_pid_update(&pid, 5.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 10.0f, out);
}

TEST_CASE("esp_foc_pid: output clamped to max", "[espFoC][pid]")
{
    esp_foc_pid_controller_t pid;
    setup_pid(&pid, 100.0f, 0.0f, 0.0f, 0.001f, -10.0f, 10.0f, 5.0f);
    float out = esp_foc_pid_update(&pid, 1.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 10.0f, out);
}

TEST_CASE("esp_foc_pid: output clamped to min", "[espFoC][pid]")
{
    esp_foc_pid_controller_t pid;
    setup_pid(&pid, 100.0f, 0.0f, 0.0f, 0.001f, -10.0f, 10.0f, 5.0f);
    float out = esp_foc_pid_update(&pid, -1.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, -10.0f, out);
}

TEST_CASE("esp_foc_pid: reset clears state", "[espFoC][pid]")
{
    esp_foc_pid_controller_t pid;
    setup_pid(&pid, 1.0f, 10.0f, 0.0f, 0.001f, -100.0f, 100.0f, 100.0f);
    for (int i = 0; i < 10; i++) {
        esp_foc_pid_update(&pid, 1.0f, 0.0f);
    }
    esp_foc_pid_reset(&pid);
    float out = esp_foc_pid_update(&pid, 0.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, out);
}

TEST_CASE("esp_foc_pid: integral builds up over time", "[espFoC][pid]")
{
    esp_foc_pid_controller_t pid;
    setup_pid(&pid, 0.0f, 1.0f, 0.0f, 0.01f, -100.0f, 100.0f, 50.0f);
    float out = 0.0f;
    for (int i = 0; i < 100; i++) {
        out = esp_foc_pid_update(&pid, 1.0f, 0.0f);
    }
    TEST_ASSERT_TRUE(out > 0.5f);
}
