/*
 * Unit tests for the sparse-measurement angle estimator.
 *
 * The harness models what the hall driver actually does: the rotor turns
 * continuously, boundaries are crossed at exact instants, and the estimator
 * only hears about a crossing on the next hot-path step — with the hardware
 * timestamp of the true instant. That delay is the half-period compensation's
 * whole reason to exist, so it has to be in the stimulus.
 */
#include <math.h>
#include <string.h>

#include "unity.h"
#include "espFoC/motor_control/esp_foc_rotor_est.h"
#include "espFoC/utils/esp_foc_angle.h"
#include "espFoC/utils/esp_foc_q16.h"

#define TICK_HZ      1000000u
#define STEP_HZ      20000u
#define DT_S         (1.0 / (double)STEP_HZ)
#define SECTOR_RAD   ((float)M_PI / 3.0f)
#define STANDSTILL_MS 100.0f

typedef struct {
    esp_foc_rotor_est_t est;
    double theta;        /* true angle, unwrapped [rad] */
    double omega;        /* true speed [rad/s] */
    double t;            /* [s] */
    double next_edge;    /* next boundary in unwrapped angle */
    int dir;
    int edges;
    /* Skip delivering this many upcoming edges (lost-edge case). */
    int drop;
} sim_t;

static float wrap_pi_f(double a)
{
    while (a > M_PI) {
        a -= 2.0 * M_PI;
    }
    while (a <= -M_PI) {
        a += 2.0 * M_PI;
    }
    return (float)a;
}

static void sim_init(sim_t *s, double omega, double theta0)
{
    memset(s, 0, sizeof(*s));
    esp_foc_rotor_est_config_t cfg;
    esp_foc_rotor_est_config_default(&cfg, TICK_HZ, STEP_HZ, SECTOR_RAD, STANDSTILL_MS);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_rotor_est_init(&s->est, &cfg));

    s->theta = theta0;
    s->omega = omega;
    s->dir = (omega >= 0.0) ? 1 : -1;
    s->next_edge = (omega >= 0.0)
                       ? (floor(theta0 / SECTOR_RAD) + 1.0) * SECTOR_RAD
                       : (ceil(theta0 / SECTOR_RAD) - 1.0) * SECTOR_RAD;
}

/** One PWM period: advance the rotor, deliver any crossing, then step(). */
static void sim_step(sim_t *s)
{
    double prev = s->theta;
    s->theta += s->omega * DT_S;
    s->t += DT_S;

    bool crossed = (s->omega >= 0.0) ? (s->theta >= s->next_edge)
                                     : (s->theta <= s->next_edge);
    if (crossed && s->omega != 0.0) {
        /* Exact instant of the crossing, which is what the capture latches. */
        double frac = (s->next_edge - prev) / (s->theta - prev);
        double t_edge = s->t - DT_S + frac * DT_S;
        uint64_t ticks = (uint64_t)(t_edge * (double)TICK_HZ + 0.5);

        if (s->drop > 0) {
            s->drop--;
        } else {
            esp_foc_rotor_est_on_edge(&s->est,
                                      q16_from_float(wrap_pi_f(s->next_edge)),
                                      ticks,
                                      s->dir);
            s->edges++;
        }
        s->next_edge += (s->omega >= 0.0) ? SECTOR_RAD : -SECTOR_RAD;
    }

    esp_foc_rotor_est_step(&s->est);
}

static void sim_run(sim_t *s, int steps)
{
    for (int i = 0; i < steps; i++) {
        sim_step(s);
    }
}

/** Signed angle error of the estimate against truth [rad]. */
static float theta_err(const sim_t *s)
{
    q16_t hat = esp_foc_rotor_est_get_theta(&s->est);
    q16_t truth = q16_from_float(wrap_pi_f(s->theta));
    return q16_to_float(q16_angle_delta(truth, hat));
}

TEST_CASE("rotor_est: rejects malformed config", "[espFoC][rotor_est]")
{
    esp_foc_rotor_est_t e;
    esp_foc_rotor_est_config_t cfg;
    esp_foc_rotor_est_config_default(&cfg, TICK_HZ, STEP_HZ, SECTOR_RAD, STANDSTILL_MS);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_rotor_est_init(NULL, &cfg));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_rotor_est_init(&e, NULL));

    esp_foc_rotor_est_config_t bad = cfg;
    bad.tick_hz = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_rotor_est_init(&e, &bad));

    bad = cfg;
    bad.step_hz = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_rotor_est_init(&e, &bad));

    bad = cfg;
    bad.lambda_theta = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_rotor_est_init(&e, &bad));

    bad = cfg;
    bad.lambda_omega = 2 * Q16_ONE;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_rotor_est_init(&e, &bad));

    bad = cfg;
    bad.dticks_max = bad.dticks_min;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_rotor_est_init(&e, &bad));

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_rotor_est_init(&e, &cfg));
}

TEST_CASE("rotor_est: happy path tracks a constant speed", "[espFoC][rotor_est]")
{
    /* 267 Hz electrical: a 60° sector lasts 625 µs = 12.5 PWM periods. */
    const double w = 2.0 * M_PI * 267.0;
    sim_t s;
    sim_init(&s, w, 0.0);

    sim_run(&s, 4000); /* 200 ms, ~320 edges */

    float w_hat = q16_to_float(esp_foc_rotor_est_get_omega(&s.est));
    TEST_ASSERT_FLOAT_WITHIN(0.01f * (float)w, (float)w, w_hat);
    /*
     * Residual is the detection quantization, not a tuning miss: the edge is
     * seen up to one period late, so ε rides within ±ω·dt/2 = 0.042 rad here.
     * The half-period compensation is what centres it on zero — without it
     * the same run parks at a systematic −ω·dt.
     */
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, theta_err(&s));
    TEST_ASSERT_TRUE(esp_foc_rotor_est_is_moving(&s.est));
    TEST_ASSERT_EQUAL_UINT32(0, s.est.rejected);
}

TEST_CASE("rotor_est: angle error stays bounded over a long run",
          "[espFoC][rotor_est]")
{
    const double w = 2.0 * M_PI * 133.0;
    sim_t s;
    sim_init(&s, w, 0.0);

    sim_run(&s, 400); /* let it acquire */

    float worst = 0.0f;
    for (int i = 0; i < 20000; i++) { /* 1 s */
        sim_step(&s);
        float e = fabsf(theta_err(&s));
        if (e > worst) {
            worst = e;
        }
    }
    /* Well inside the 60° sector the clamp guarantees. */
    TEST_ASSERT_TRUE(worst < 0.05f);
}

TEST_CASE("rotor_est: omega error decays as (1-lambda_w)^k, monotone",
          "[espFoC][rotor_est]")
{
    /*
     * Final-value theorem in closed form. The frequency loop is a scalar
     * contraction sampled once per edge, so with a constant true speed the
     * error after k edges is exactly (1-λ_ω)^k times the first one — and it
     * never changes sign, which is what "does not oscillate" means here.
     */
    const double w = 2.0 * M_PI * 200.0;
    sim_t s;
    sim_init(&s, w, 0.0);

    float prev_err = 0.0f;
    int seen = 0;
    int last_edges = 0;
    float first_err = 0.0f;

    for (int i = 0; i < 6000 && seen < 12; i++) {
        sim_step(&s);
        if (s.edges == last_edges) {
            continue;
        }
        last_edges = s.edges;
        if (s.edges < 3) {
            continue; /* first two edges seed the anchor and the period */
        }

        float err = (float)w - q16_to_float(esp_foc_rotor_est_get_omega(&s.est));
        if (seen == 0) {
            first_err = err;
        } else {
            /* No sign change and strictly shrinking. */
            TEST_ASSERT_TRUE(err * first_err > 0.0f);
            TEST_ASSERT_TRUE(fabsf(err) < fabsf(prev_err));
        }
        prev_err = err;
        seen++;
    }

    TEST_ASSERT_EQUAL_INT(12, seen);

    /* Predicted contraction over the 11 edges that were compared. */
    float lam = q16_to_float(s.est.cfg.lambda_omega);
    float predicted = first_err * powf(1.0f - lam, 11.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.15f * fabsf(first_err), predicted, prev_err);
}

TEST_CASE("rotor_est: converges under constant acceleration",
          "[espFoC][rotor_est]")
{
    sim_t s;
    sim_init(&s, 2.0 * M_PI * 50.0, 0.0);

    /* 50 → 250 Hz over 200 ms. */
    const double alpha = 2.0 * M_PI * 1000.0;
    float worst = 0.0f;
    for (int i = 0; i < 4000; i++) {
        s.omega += alpha * DT_S;
        sim_step(&s);
        /* Skip acquisition: at 50 Hz the sectors are 55 periods apart, so the
         * first handful of edges are still bringing ω̂ up from zero. */
        if (i > 1000) {
            float e = fabsf(theta_err(&s));
            if (e > worst) {
                worst = e;
            }
        }
    }

    /* Lag is bounded and well inside a sector; it must not run away. */
    TEST_ASSERT_TRUE(worst < 0.25f);
    float w_hat = q16_to_float(esp_foc_rotor_est_get_omega(&s.est));
    TEST_ASSERT_FLOAT_WITHIN(0.10f * (float)s.omega, (float)s.omega, w_hat);
}

TEST_CASE("rotor_est: a lost edge keeps the speed right", "[espFoC][rotor_est]")
{
    const double w = 2.0 * M_PI * 200.0;
    sim_t s;
    sim_init(&s, w, 0.0);
    sim_run(&s, 2000);

    float w_before = q16_to_float(esp_foc_rotor_est_get_omega(&s.est));

    /* Swallow one boundary: the next measurement jumps two sectors. */
    s.drop = 1;
    sim_run(&s, 400);

    float w_after = q16_to_float(esp_foc_rotor_est_get_omega(&s.est));
    /*
     * Δθ comes from the measurement, not from a nominal π/3, so the doubled
     * sector carries a doubled Δt and the quotient is unchanged.
     */
    TEST_ASSERT_FLOAT_WITHIN(0.03f * (float)w, w_before, w_after);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.0f, theta_err(&s));
}

TEST_CASE("rotor_est: survives a reversal", "[espFoC][rotor_est]")
{
    const double w = 2.0 * M_PI * 120.0;
    sim_t s;
    sim_init(&s, w, 0.0);
    sim_run(&s, 2000);

    /*
     * Flip without resetting: the estimator has to walk ω̂ through zero on its
     * own. The next boundary is the one just left behind, and Δθ from the
     * measurement comes back negative, which is the only thing that tells the
     * frequency loop the sign changed.
     */
    s.omega = -w;
    s.dir = -1;
    s.next_edge = floor(s.theta / SECTOR_RAD) * SECTOR_RAD;

    sim_run(&s, 5000);

    float w_hat = q16_to_float(esp_foc_rotor_est_get_omega(&s.est));
    TEST_ASSERT_TRUE(w_hat < 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.02f * (float)w, (float)-w, w_hat);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.0f, theta_err(&s));
    TEST_ASSERT_EQUAL_INT(-1, s.est.dir);
}

TEST_CASE("rotor_est: standstill decays omega and freezes the angle",
          "[espFoC][rotor_est]")
{
    const double w = 2.0 * M_PI * 100.0;
    sim_t s;
    sim_init(&s, w, 0.0);
    sim_run(&s, 2000);

    TEST_ASSERT_TRUE(esp_foc_rotor_est_is_moving(&s.est));

    /* Rotor stops dead: no more edges ever. */
    s.omega = 0.0;
    sim_run(&s, 2100); /* past the 2000-period standstill window */
    TEST_ASSERT_FALSE(esp_foc_rotor_est_is_moving(&s.est));

    sim_run(&s, 60000); /* 3 s */
    TEST_ASSERT_EQUAL_INT32(0, esp_foc_rotor_est_get_omega(&s.est));

    q16_t frozen = esp_foc_rotor_est_get_theta(&s.est);
    sim_run(&s, 20000);
    TEST_ASSERT_EQUAL_INT32(frozen, esp_foc_rotor_est_get_theta(&s.est));

    /*
     * Frozen inside the sector it stopped in. That bound is the whole argument
     * for sensored mode: worst case 60° gives cos(30°) = 0.87 of the torque,
     * available from standstill with no I-f and no observer handoff.
     */
    float band = q16_to_float(q16_add(s.est.cfg.sector_span,
                                      s.est.cfg.clamp_margin));
    TEST_ASSERT_TRUE(fabsf(theta_err(&s)) <= band);
}

TEST_CASE("rotor_est: clamp bounds the angle when omega is wrong",
          "[espFoC][rotor_est]")
{
    esp_foc_rotor_est_t e;
    esp_foc_rotor_est_config_t cfg;
    esp_foc_rotor_est_config_default(&cfg, TICK_HZ, STEP_HZ, SECTOR_RAD, STANDSTILL_MS);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_rotor_est_init(&e, &cfg));

    /* Two edges 625 µs apart establish an anchor and a direction. */
    esp_foc_rotor_est_on_edge(&e, 0, 0, +1);
    esp_foc_rotor_est_on_edge(&e, q16_from_float(SECTOR_RAD), 625, +1);

    /* Now lie: a hundredfold speed, and let it free-run a whole revolution. */
    e.omega_hat = q16_from_float(2.0f * (float)M_PI * 20000.0f);

    for (int i = 0; i < 500; i++) {
        esp_foc_rotor_est_step(&e);
    }

    float adv = q16_to_float(e.adv);
    float band = q16_to_float(q16_add(cfg.sector_span, cfg.clamp_margin));
    TEST_ASSERT_TRUE(adv <= band + 1e-4f);
    TEST_ASSERT_TRUE(e.clamped > 0);
}

TEST_CASE("rotor_est: rejects bounce below dticks_min", "[espFoC][rotor_est]")
{
    esp_foc_rotor_est_t e;
    esp_foc_rotor_est_config_t cfg;
    esp_foc_rotor_est_config_default(&cfg, TICK_HZ, STEP_HZ, SECTOR_RAD, STANDSTILL_MS);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_rotor_est_init(&e, &cfg));

    esp_foc_rotor_est_on_edge(&e, 0, 0, +1);
    esp_foc_rotor_est_on_edge(&e, q16_from_float(SECTOR_RAD), 625, +1);

    q16_t th_before = esp_foc_rotor_est_get_theta(&e);
    q16_t w_before = esp_foc_rotor_est_get_omega(&e);
    uint32_t edges_before = e.edges;

    /* One tick later: physically impossible, so nothing may move. */
    esp_foc_rotor_est_on_edge(&e, q16_from_float(2.0f * SECTOR_RAD), 626, +1);

    TEST_ASSERT_EQUAL_UINT32(1, e.rejected);
    TEST_ASSERT_EQUAL_UINT32(edges_before, e.edges);
    TEST_ASSERT_EQUAL_INT32(th_before, esp_foc_rotor_est_get_theta(&e));
    TEST_ASSERT_EQUAL_INT32(w_before, esp_foc_rotor_est_get_omega(&e));
}

TEST_CASE("rotor_est: a stale gap re-anchors without a speed update",
          "[espFoC][rotor_est]")
{
    esp_foc_rotor_est_t e;
    esp_foc_rotor_est_config_t cfg;
    esp_foc_rotor_est_config_default(&cfg, TICK_HZ, STEP_HZ, SECTOR_RAD, STANDSTILL_MS);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_rotor_est_init(&e, &cfg));

    esp_foc_rotor_est_on_edge(&e, 0, 0, +1);
    esp_foc_rotor_est_on_edge(&e, q16_from_float(SECTOR_RAD), 625, +1);
    q16_t w_before = esp_foc_rotor_est_get_omega(&e);

    uint64_t far = 625ull + (uint64_t)cfg.dticks_max + 1000ull;
    esp_foc_rotor_est_on_edge(&e, q16_from_float(2.0f * SECTOR_RAD), far, +1);

    TEST_ASSERT_EQUAL_UINT32(1, e.stale);
    TEST_ASSERT_EQUAL_INT32(w_before, esp_foc_rotor_est_get_omega(&e));
    /*
     * The angle is still adopted — a stale period is not a bad position. It
     * lands within the clamp margin of the measurement because the λ_θ step
     * alone cannot close a gap this size and the band catches the rest.
     */
    float margin = q16_to_float(cfg.clamp_margin);
    TEST_ASSERT_FLOAT_WITHIN(margin + 0.01f,
                             2.0f * (float)SECTOR_RAD,
                             q16_to_float(esp_foc_rotor_est_get_theta(&e)));
}

TEST_CASE("rotor_est: numerically stable at the speed extremes",
          "[espFoC][rotor_est]")
{
    esp_foc_rotor_est_t e;
    esp_foc_rotor_est_config_t cfg;
    esp_foc_rotor_est_config_default(&cfg, TICK_HZ, STEP_HZ, SECTOR_RAD, STANDSTILL_MS);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_rotor_est_init(&e, &cfg));

    /* Fastest admissible edge rate: dticks_min, where 1 µs hurts most. */
    uint64_t t = 0;
    q16_t th = 0;
    for (int i = 0; i < 40; i++) {
        t += cfg.dticks_min;
        th = q16_wrap_pi(q16_add(th, q16_from_float(SECTOR_RAD)));
        esp_foc_rotor_est_on_edge(&e, th, t, +1);
        esp_foc_rotor_est_step(&e);
    }
    /*
     * π/3 in 5 µs is 209 krad/s, far past what Q16.16 can hold. The property
     * being checked is that it saturates positive instead of wrapping to a
     * negative speed, which would reverse the extrapolation.
     */
    q16_t w_fast = esp_foc_rotor_est_get_omega(&e);
    TEST_ASSERT_TRUE(w_fast > 0);
    TEST_ASSERT_TRUE(esp_foc_rotor_est_get_theta(&e) <= Q16_PI);
    TEST_ASSERT_TRUE(esp_foc_rotor_est_get_theta(&e) > Q16_MINUS_PI);

    /* Slowest admissible: dticks_max, one tick short of stale. */
    esp_foc_rotor_est_reset(&e);
    t = 0;
    th = 0;
    for (int i = 0; i < 40; i++) {
        t += cfg.dticks_max;
        th = q16_wrap_pi(q16_add(th, q16_from_float(SECTOR_RAD)));
        esp_foc_rotor_est_on_edge(&e, th, t, +1);
        esp_foc_rotor_est_step(&e);
    }
    q16_t w_slow = esp_foc_rotor_est_get_omega(&e);
    TEST_ASSERT_TRUE(w_slow > 0);
    TEST_ASSERT_TRUE(w_slow < w_fast);
    TEST_ASSERT_EQUAL_UINT32(0, e.stale);
    TEST_ASSERT_TRUE(esp_foc_rotor_est_get_theta(&e) <= Q16_PI);
    TEST_ASSERT_TRUE(esp_foc_rotor_est_get_theta(&e) > Q16_MINUS_PI);
}

TEST_CASE("rotor_est: null and uninitialised calls are inert",
          "[espFoC][rotor_est]")
{
    esp_foc_rotor_est_t zero;
    memset(&zero, 0, sizeof(zero));

    esp_foc_rotor_est_step(NULL);
    esp_foc_rotor_est_on_edge(NULL, 0, 0, 1);
    esp_foc_rotor_est_reset(NULL);
    esp_foc_rotor_est_config_default(NULL, TICK_HZ, STEP_HZ, SECTOR_RAD, STANDSTILL_MS);

    esp_foc_rotor_est_step(&zero);
    esp_foc_rotor_est_on_edge(&zero, Q16_ONE, 100, 1);
    TEST_ASSERT_EQUAL_INT32(0, esp_foc_rotor_est_get_theta(&zero));
    TEST_ASSERT_EQUAL_INT32(0, esp_foc_rotor_est_get_omega(&zero));
    TEST_ASSERT_EQUAL_INT32(0, esp_foc_rotor_est_get_theta(NULL));
    TEST_ASSERT_EQUAL_INT32(0, esp_foc_rotor_est_get_omega(NULL));
    TEST_ASSERT_FALSE(esp_foc_rotor_est_is_moving(NULL));
}
