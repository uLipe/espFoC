/*
 * Unit tests for esp_foc_traj: trapezoidal rest-to-rest profile.
 */
#include <math.h>
#include <stdint.h>

#include "unity.h"
#include "esp_err.h"
#include "espFoC/motor_control/esp_foc_traj.h"

#define FS       2500u
#define A_MAX    1800.0f
#define V_MAX    (2.0f * 3.14159265f * 20.0f)
#define Q16(x)   ((int64_t)llround((double)(x) * 65536.0))
#define MAX_STEPS 200000
/* The target Unity is built without 64-bit asserts. */
#define ASSERT_EQ64(e, a) TEST_ASSERT_TRUE((int64_t)(e) == (int64_t)(a))

typedef struct {
    int steps;
    double acc_max;     /* rad/s^2 */
    double w_max;       /* rad/s */
    double overshoot;   /* rad past the target, toward the motion */
    double w_peak_signed;
} traj_run_t;

static double to_rad(int64_t q32)
{
    return (double)q32 / 4294967296.0;
}

static traj_run_t run_to_done(esp_foc_traj_t *t)
{
    traj_run_t r = { 0 };
    const double tgt = to_rad(t->tgt);
    const double s = (t->tgt >= t->th) ? 1.0 : -1.0;
    while (!esp_foc_traj_done(t) && (r.steps < MAX_STEPS)) {
        esp_foc_traj_step(t);
        r.steps++;
        const double acc = fabs(to_rad(t->acc)) * FS * FS;
        const double w = to_rad(t->w) * FS;
        /* The landing step brakes from the last speed in one sample. */
        if (!esp_foc_traj_done(t) && (acc > r.acc_max)) {
            r.acc_max = acc;
        }
        if (fabs(w) > r.w_max) {
            r.w_max = fabs(w);
            r.w_peak_signed = w;
        }
        const double over = s * (to_rad(t->th) - tgt);
        if (over > r.overshoot) {
            r.overshoot = over;
        }
    }
    return r;
}

static void start(esp_foc_traj_t *t, double from, double to)
{
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_traj_init(t, FS, A_MAX, V_MAX));
    esp_foc_traj_seed(t, Q16(from), 0);
    esp_foc_traj_set_target(t, Q16(to));
}

TEST_CASE("traj long move cruises at v_max and lands exactly", "[espFoC][traj]")
{
    esp_foc_traj_t t;
    const double d = 20.0;
    start(&t, 1.0, 1.0 + d);
    const traj_run_t r = run_to_done(&t);

    TEST_ASSERT_TRUE(esp_foc_traj_done(&t));
    ASSERT_EQ64(Q16(1.0 + d), esp_foc_traj_theta_q16(&t));
    ASSERT_EQ64(0, t.w);
    TEST_ASSERT_TRUE(r.acc_max <= A_MAX * 1.001);
    TEST_ASSERT_TRUE(r.w_max <= V_MAX * 1.0001);
    TEST_ASSERT_TRUE(r.w_max >= V_MAX * 0.999);
    TEST_ASSERT_TRUE(r.overshoot <= 0.0);

    const double t_ideal = d / V_MAX + V_MAX / A_MAX;
    TEST_ASSERT_INT_WITHIN(3, (int)lround(t_ideal * FS), r.steps);
}

TEST_CASE("traj short move is triangular at sqrt(a d)", "[espFoC][traj]")
{
    esp_foc_traj_t t;
    const double d = 0.5;
    start(&t, 0.0, -d);
    const traj_run_t r = run_to_done(&t);

    ASSERT_EQ64(Q16(-d), esp_foc_traj_theta_q16(&t));
    const double w_pk = sqrt(A_MAX * d);
    TEST_ASSERT_TRUE(w_pk < V_MAX);
    TEST_ASSERT_DOUBLE_WITHIN(0.02 * w_pk, w_pk, r.w_max);
    TEST_ASSERT_TRUE(r.w_peak_signed < 0.0);
    TEST_ASSERT_TRUE(r.acc_max <= A_MAX * 1.001);
    TEST_ASSERT_TRUE(r.overshoot <= 0.0);
    TEST_ASSERT_INT_WITHIN(3, (int)lround(2.0 * sqrt(d / A_MAX) * FS), r.steps);
}

TEST_CASE("traj tiny moves and zero moves settle at once", "[espFoC][traj]")
{
    esp_foc_traj_t t;
    start(&t, 3.0, 3.0);
    esp_foc_traj_step(&t);
    TEST_ASSERT_TRUE(esp_foc_traj_done(&t));
    ASSERT_EQ64(0, t.acc);

    start(&t, 3.0, 3.0 + 1.0 / 65536.0);
    const traj_run_t r = run_to_done(&t);
    TEST_ASSERT_TRUE(r.steps <= 2);
    ASSERT_EQ64(Q16(3.0) + 1, esp_foc_traj_theta_q16(&t));
}

TEST_CASE("traj redirect while moving reverses within limits", "[espFoC][traj]")
{
    esp_foc_traj_t t;
    start(&t, 0.0, 20.0);
    for (int i = 0; i < 400; i++) {
        esp_foc_traj_step(&t);
    }
    TEST_ASSERT_TRUE(to_rad(t.w) * FS > 0.5 * V_MAX);

    esp_foc_traj_set_target(&t, Q16(-5.0));
    double acc_max = 0.0;
    double w_max = 0.0;
    double th_min = 1e9;
    int steps = 0;
    while (!esp_foc_traj_done(&t) && (steps < MAX_STEPS)) {
        esp_foc_traj_step(&t);
        steps++;
        if (!esp_foc_traj_done(&t)) {
            acc_max = fmax(acc_max, fabs(to_rad(t.acc)) * FS * FS);
        }
        w_max = fmax(w_max, fabs(to_rad(t.w)) * FS);
        th_min = fmin(th_min, to_rad(t.th));
    }
    TEST_ASSERT_TRUE(esp_foc_traj_done(&t));
    ASSERT_EQ64(Q16(-5.0), esp_foc_traj_theta_q16(&t));
    TEST_ASSERT_TRUE(acc_max <= A_MAX * 1.001);
    TEST_ASSERT_TRUE(w_max <= V_MAX * 1.0001);
    TEST_ASSERT_TRUE(th_min >= -5.0 - 1e-9);
}

TEST_CASE("traj seeded with the shaft moving away brakes and returns", "[espFoC][traj]")
{
    esp_foc_traj_t t;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_traj_init(&t, FS, A_MAX, V_MAX));
    esp_foc_traj_seed(&t, Q16(2.0), (q16_t)Q16(100.0));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 100.0, esp_foc_traj_w(&t) / 65536.0);

    esp_foc_traj_set_target(&t, Q16(2.0));
    double acc_max = 0.0;
    double th_max = 0.0;
    int steps = 0;
    while (!esp_foc_traj_done(&t) && (steps < MAX_STEPS)) {
        esp_foc_traj_step(&t);
        steps++;
        if (!esp_foc_traj_done(&t)) {
            acc_max = fmax(acc_max, fabs(to_rad(t.acc)) * FS * FS);
        }
        th_max = fmax(th_max, to_rad(t.th));
    }
    TEST_ASSERT_TRUE(esp_foc_traj_done(&t));
    ASSERT_EQ64(Q16(2.0), esp_foc_traj_theta_q16(&t));
    TEST_ASSERT_TRUE(acc_max <= A_MAX * 1.001);
    /* Coasting 100 rad/s down at a_max carries it v^2 / 2a past the start. */
    TEST_ASSERT_DOUBLE_WITHIN(0.05, 2.0 + 100.0 * 100.0 / (2.0 * A_MAX), th_max);
}

TEST_CASE("traj lower v_max mid-flight slows down at a_max", "[espFoC][traj]")
{
    esp_foc_traj_t t;
    start(&t, 0.0, 200.0);
    for (int i = 0; i < 1000; i++) {
        esp_foc_traj_step(&t);
    }
    TEST_ASSERT_DOUBLE_WITHIN(0.01 * V_MAX, V_MAX, to_rad(t.w) * FS);

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_traj_set_limits(&t, A_MAX, 0.5f * V_MAX));
    const double w0 = to_rad(t.w) * FS;
    for (int i = 0; i < 10; i++) {
        esp_foc_traj_step(&t);
    }
    TEST_ASSERT_DOUBLE_WITHIN(1.0, w0 - 10.0 * A_MAX / FS, to_rad(t.w) * FS);

    const int slow_steps = (int)ceil((w0 - 0.5 * V_MAX) / A_MAX * FS);
    for (int i = 10; i < slow_steps; i++) {
        esp_foc_traj_step(&t);
    }
    const traj_run_t r = run_to_done(&t);
    TEST_ASSERT_TRUE(r.w_max <= 0.5 * V_MAX * 1.0001);
    ASSERT_EQ64(Q16(200.0), esp_foc_traj_theta_q16(&t));
}

TEST_CASE("traj multi-turn move stays exact in int64", "[espFoC][traj]")
{
    esp_foc_traj_t t;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_traj_init(&t, FS, 20000.0f, 1200.0f));
    esp_foc_traj_seed(&t, Q16(-3000.0), 0);
    esp_foc_traj_set_target(&t, Q16(4000.0));
    const traj_run_t r = run_to_done(&t);
    TEST_ASSERT_TRUE(esp_foc_traj_done(&t));
    ASSERT_EQ64(Q16(4000.0), esp_foc_traj_theta_q16(&t));
    TEST_ASSERT_TRUE(r.acc_max <= 20000.0 * 1.001);
    TEST_ASSERT_TRUE(r.overshoot <= 0.0);
}

TEST_CASE("traj acceleration integrates to the speed", "[espFoC][traj]")
{
    esp_foc_traj_t t;
    start(&t, 0.0, 7.0);
    int64_t w_sum = 0;
    int guard = 0;
    while (!esp_foc_traj_done(&t) && (guard++ < MAX_STEPS)) {
        esp_foc_traj_step(&t);
        w_sum += t.acc;
        ASSERT_EQ64(w_sum, t.w);
    }
    ASSERT_EQ64(0, w_sum);
}

TEST_CASE("traj inactive holds and bad limits are refused", "[espFoC][traj]")
{
    esp_foc_traj_t t;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_traj_init(&t, FS, A_MAX, V_MAX));
    esp_foc_traj_seed(&t, Q16(1.5), (q16_t)Q16(10.0));
    const int64_t th0 = t.th;
    esp_foc_traj_step(&t);
    ASSERT_EQ64(th0, t.th);
    ASSERT_EQ64(0, t.acc);

    esp_foc_traj_set_target(&t, Q16(3.0));
    esp_foc_traj_stop(&t);
    esp_foc_traj_step(&t);
    ASSERT_EQ64(th0, t.th);
    ASSERT_EQ64(0, t.w);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_traj_init(NULL, FS, A_MAX, V_MAX));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_traj_init(&t, 0u, A_MAX, V_MAX));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_traj_init(&t, FS, 0.0f, V_MAX));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_traj_init(&t, FS, A_MAX, -1.0f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_traj_init(&t, FS, A_MAX, (float)FS));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_traj_init(&t, FS, 1e-4f, V_MAX));
}
