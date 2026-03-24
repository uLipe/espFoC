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

/* --- Behavioral: PI combined, integrator saturation, output vs integrator, sign, D term --- */

TEST_CASE("esp_foc_pid: PI step reference P and I both contribute", "[espFoC][pid]")
{
    esp_foc_pid_controller_t pid;
    setup_pid(&pid, 0.5f, 2.0f, 0.0f, 0.01f, -100.0f, 100.0f, 100.0f);
    /* First sample: error=1, P=0.5, I_prev=0 -> mv = 0.5 */
    float o1 = esp_foc_pid_update(&pid, 1.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.5f, o1);
    /* Integrator after first update: acc += 1*0.01*2 = 0.02 */
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.02f, pid.accumulated_error);
}

TEST_CASE("esp_foc_pid: integrator clamps to limit windup bound", "[espFoC][pid]")
{
    esp_foc_pid_controller_t pid;
    const float int_lim = 3.0f;
    setup_pid(&pid, 0.0f, 50.0f, 0.0f, 0.01f, -100.0f, 100.0f, int_lim);
    for (int i = 0; i < 500; i++) {
        esp_foc_pid_update(&pid, 1.0f, 0.0f);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, int_lim, pid.accumulated_error);
    TEST_ASSERT_TRUE(pid.accumulated_error <= int_lim + 1e-3f);
    TEST_ASSERT_TRUE(pid.accumulated_error >= -int_lim - 1e-3f);
}

TEST_CASE("esp_foc_pid: output saturates while integrator stays bounded", "[espFoC][pid]")
{
    esp_foc_pid_controller_t pid;
    setup_pid(&pid, 0.0f, 20.0f, 0.0f, 0.01f, -0.5f, 0.5f, 10.0f);
    float out = 0.0f;
    for (int i = 0; i < 200; i++) {
        out = esp_foc_pid_update(&pid, 1.0f, 0.0f);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.5f, out);
    TEST_ASSERT_TRUE(pid.accumulated_error <= 10.0f + 1e-2f);
    TEST_ASSERT_TRUE(pid.accumulated_error >= -10.0f - 1e-2f);
}

TEST_CASE("esp_foc_pid: negative error negative output with positive kp", "[espFoC][pid]")
{
    esp_foc_pid_controller_t pid;
    setup_pid(&pid, 2.0f, 0.0f, 0.0f, 0.001f, -100.0f, 100.0f, 10.0f);
    float out = esp_foc_pid_update(&pid, 0.0f, 3.0f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, -6.0f, out);
}

TEST_CASE("esp_foc_pid: derivative term reacts to error step", "[espFoC][pid]")
{
    esp_foc_pid_controller_t pid;
    setup_pid(&pid, 0.0f, 0.0f, 0.01f, 0.1f, -1000.0f, 1000.0f, 100.0f);
    esp_foc_pid_update(&pid, 0.0f, 0.0f);
    /* error jumps 0 -> 1; (error - prev_error)*inv_dt = (1-0)*10 = 10; D = 0.01*10 = 0.1 */
    float out = esp_foc_pid_update(&pid, 1.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.1f, out);
}
