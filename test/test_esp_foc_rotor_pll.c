/*
 * Unit tests for the sensored angle tracking loop.
 *
 * The stimulus is what an absolute encoder on a fixed clock actually hands
 * over: a continuously turning rotor, sampled at the loop rate, with the
 * angle rounded to the sensor grid. The quantisation is the point — without
 * it every estimator looks good, and the whole reason this unit exists is the
 * noise a first difference makes out of that grid.
 */
#include <math.h>
#include <string.h>

#include "unity.h"
#include "espFoC/motor_control/esp_foc_rotor_pll.h"
#include "espFoC/utils/esp_foc_angle.h"
#include "espFoC/utils/esp_foc_q16.h"

/* The sensored bench: AS5600 12-bit read at PWM/8. */
#define STEP_HZ   2500u
#define ENC_BITS  12
#define ENC_COUNTS (1 << ENC_BITS)
#define DT_S      (1.0 / (double)STEP_HZ)
#define BW_HZ     100.0f
#define ZETA      1.0f

static double wrap_pi_d(double a)
{
    while (a > M_PI) {
        a -= 2.0 * M_PI;
    }
    while (a <= -M_PI) {
        a += 2.0 * M_PI;
    }
    return a;
}

/** True angle → what the encoder reports, rounded to its grid. */
static q16_t encode(double theta_rad)
{
    const double counts = (double)ENC_COUNTS;
    double c = floor(wrap_pi_d(theta_rad) * counts / (2.0 * M_PI) + 0.5);
    return q16_wrap_pi(q16_from_float((float)(c * 2.0 * M_PI / counts)));
}

static esp_err_t make(esp_foc_rotor_pll_t *p, float bw, float zeta, float wmax)
{
    esp_foc_rotor_pll_config_t cfg;
    esp_foc_rotor_pll_config_default(&cfg, STEP_HZ, bw, zeta, wmax);
    return esp_foc_rotor_pll_init(p, &cfg);
}

/** Spin at a constant speed for n steps, returning the last estimate. */
static void spin(esp_foc_rotor_pll_t *p, double omega, int n, double *theta_io)
{
    double th = (theta_io != NULL) ? *theta_io : 0.0;
    for (int i = 0; i < n; i++) {
        th += omega * DT_S;
        esp_foc_rotor_pll_step(p, encode(th), true);
    }
    if (theta_io != NULL) {
        *theta_io = th;
    }
}

TEST_CASE("rotor_pll: init rejects what cannot be a loop", "[rotor_pll]")
{
    esp_foc_rotor_pll_t p;
    esp_foc_rotor_pll_config_t cfg;

    esp_foc_rotor_pll_config_default(&cfg, STEP_HZ, BW_HZ, ZETA, 0.0f);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_rotor_pll_init(NULL, &cfg));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_rotor_pll_init(&p, NULL));

    /* Past the discretisation ceiling: at ζ=1 that is near step_hz/12. */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, make(&p, 400.0f, ZETA, 0.0f));
    /* Degenerate gains. */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, make(&p, 0.0f, ZETA, 0.0f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, make(&p, BW_HZ, 0.0f, 0.0f));

    esp_foc_rotor_pll_config_default(&cfg, STEP_HZ, BW_HZ, ZETA, 0.0f);
    cfg.domain = (esp_foc_rotor_pll_domain_t)7;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_rotor_pll_init(&p, &cfg));

    TEST_ASSERT_EQUAL(ESP_OK, make(&p, BW_HZ, ZETA, 0.0f));
}

TEST_CASE("rotor_pll: constant speed is tracked without lag", "[rotor_pll]")
{
    /* Type 2, so a ramp in θ leaves no steady-state phase error. */
    const double omega[] = {5.0, 60.0, -60.0, 400.0};

    for (unsigned k = 0; k < sizeof(omega) / sizeof(omega[0]); k++) {
        esp_foc_rotor_pll_t p;
        TEST_ASSERT_EQUAL(ESP_OK, make(&p, BW_HZ, ZETA, 0.0f));

        double th = 0.0;
        spin(&p, omega[k], 5000, &th);

        const double w_hat = (double)q16_to_float(esp_foc_rotor_pll_get_omega(&p));
        TEST_ASSERT_DOUBLE_WITHIN(fabs(omega[k]) * 0.02 + 0.5, omega[k], w_hat);

        const double e = (double)q16_to_float(esp_foc_rotor_pll_get_phase_err(&p));
        TEST_ASSERT_DOUBLE_WITHIN(2.0 * M_PI / ENC_COUNTS * 3.0, 0.0, e);

        /* θ̂ is the prediction for the next instant, one period ahead. */
        const double th_hat =
            (double)q16_to_float(esp_foc_rotor_pll_get_theta(&p));
        TEST_ASSERT_DOUBLE_WITHIN(
            0.02, 0.0, wrap_pi_d(wrap_pi_d(th + omega[k] * DT_S) - th_hat));
    }
}

TEST_CASE("rotor_pll: beats the first difference on the same grid", "[rotor_pll]")
{
    /*
     * The reason the unit exists. Both estimators see the identical quantised
     * stream; the first difference has a noise floor of one count per period
     * no matter how slowly the rotor turns.
     */
    esp_foc_rotor_pll_t p;
    TEST_ASSERT_EQUAL(ESP_OK, make(&p, BW_HZ, ZETA, 0.0f));

    const double omega = 8.0; /* mechanical rad/s, deep in the bad region */
    double th = 0.0;

    spin(&p, omega, 2000, &th); /* settle */

    double s_pll = 0.0;
    double s_fd = 0.0;
    q16_t prev = encode(th);
    const int n = 4000;

    for (int i = 0; i < n; i++) {
        th += omega * DT_S;
        const q16_t meas = encode(th);

        esp_foc_rotor_pll_step(&p, meas, true);

        const double w_fd =
            (double)q16_to_float(q16_angle_delta(prev, meas)) * (double)STEP_HZ;
        prev = meas;

        const double e_pll =
            (double)q16_to_float(esp_foc_rotor_pll_get_omega(&p)) - omega;
        const double e_fd = w_fd - omega;
        s_pll += e_pll * e_pll;
        s_fd += e_fd * e_fd;
    }

    const double rms_pll = sqrt(s_pll / n);
    const double rms_fd = sqrt(s_fd / n);

    /* One count per period is the floor the first difference cannot go under. */
    const double q_rate = (2.0 * M_PI / ENC_COUNTS) * (double)STEP_HZ;
    TEST_ASSERT_TRUE(rms_fd > 0.25 * q_rate);
    TEST_ASSERT_TRUE(rms_pll < rms_fd / 5.0);
}

TEST_CASE("rotor_pll: settles monotonically, final value holds", "[rotor_pll]")
{
    /*
     * Closed-loop final value: after a speed step the estimate must converge
     * and stop moving. ζ=1 also has to arrive without ringing, which is the
     * property the consuming speed loop is being spared.
     */
    esp_foc_rotor_pll_t p;
    TEST_ASSERT_EQUAL(ESP_OK, make(&p, BW_HZ, ZETA, 0.0f));

    double th = 0.0;
    const double target = 120.0;
    double prev_err = target;
    int worsened = 0;

    for (int i = 0; i < 400; i++) {
        th += target * DT_S;
        esp_foc_rotor_pll_step(&p, encode(th), true);
        const double err =
            target - (double)q16_to_float(esp_foc_rotor_pll_get_omega(&p));
        /* Quantisation makes single steps jitter; only count real reversals. */
        if (err > prev_err + 1.0) {
            worsened++;
        }
        prev_err = err;
    }
    TEST_ASSERT_TRUE(worsened <= 2);

    /* Final value: the swing over the last stretch is bounded and unbiased. */
    double lo = 1e9;
    double hi = -1e9;
    double sum = 0.0;
    const int n = 2500;
    for (int i = 0; i < n; i++) {
        th += target * DT_S;
        esp_foc_rotor_pll_step(&p, encode(th), true);
        const double w =
            (double)q16_to_float(esp_foc_rotor_pll_get_omega(&p));
        lo = (w < lo) ? w : lo;
        hi = (w > hi) ? w : hi;
        sum += w;
    }
    TEST_ASSERT_DOUBLE_WITHIN(target * 0.01, target, sum / n);
    TEST_ASSERT_TRUE((hi - lo) < target * 0.10);
}

TEST_CASE("rotor_pll: wraps in both directions", "[rotor_pll]")
{
    const double omega[] = {300.0, -300.0};

    for (unsigned k = 0; k < 2; k++) {
        esp_foc_rotor_pll_t p;
        TEST_ASSERT_EQUAL(ESP_OK, make(&p, BW_HZ, ZETA, 0.0f));

        /* Start on the discontinuity so the first samples straddle it. */
        double th = M_PI - 0.01;
        esp_foc_rotor_pll_seed(&p, encode(th), q16_from_float((float)omega[k]));

        for (int i = 0; i < 3000; i++) {
            th += omega[k] * DT_S;
            esp_foc_rotor_pll_step(&p, encode(th), true);

            const q16_t t_hat = esp_foc_rotor_pll_get_theta(&p);
            TEST_ASSERT_TRUE(t_hat > q16_from_float(-3.1416f));
            TEST_ASSERT_TRUE(t_hat <= q16_from_float(3.1416f));
        }

        const double th_hat = (double)q16_to_float(esp_foc_rotor_pll_get_theta(&p));
        TEST_ASSERT_DOUBLE_WITHIN(
            0.05, 0.0, wrap_pi_d(wrap_pi_d(th + omega[k] * DT_S) - th_hat));
        TEST_ASSERT_DOUBLE_WITHIN(
            fabs(omega[k]) * 0.03, omega[k],
            (double)q16_to_float(esp_foc_rotor_pll_get_omega(&p)));
    }
}

TEST_CASE("rotor_pll: reversal and standstill", "[rotor_pll]")
{
    esp_foc_rotor_pll_t p;
    TEST_ASSERT_EQUAL(ESP_OK, make(&p, BW_HZ, ZETA, 0.0f));

    double th = 0.0;
    spin(&p, 150.0, 3000, &th);
    spin(&p, -150.0, 3000, &th);
    TEST_ASSERT_DOUBLE_WITHIN(
        6.0, -150.0, (double)q16_to_float(esp_foc_rotor_pll_get_omega(&p)));

    /* Held angle: type 2 must drive ω̂ to zero, not park on a bias. */
    const q16_t still = encode(th);
    for (int i = 0; i < 6000; i++) {
        esp_foc_rotor_pll_step(&p, still, true);
    }
    TEST_ASSERT_DOUBLE_WITHIN(
        0.5, 0.0, (double)q16_to_float(esp_foc_rotor_pll_get_omega(&p)));
    TEST_ASSERT_DOUBLE_WITHIN(
        0.01, 0.0,
        wrap_pi_d((double)q16_to_float(
                      q16_angle_delta(still, esp_foc_rotor_pll_get_theta(&p)))));
}

TEST_CASE("rotor_pll: coasting dead-reckons and does not integrate", "[rotor_pll]")
{
    esp_foc_rotor_pll_t p;
    TEST_ASSERT_EQUAL(ESP_OK, make(&p, BW_HZ, ZETA, 0.0f));

    double th = 0.0;
    const double omega = 200.0;
    spin(&p, omega, 4000, &th);

    const q16_t w_before = esp_foc_rotor_pll_get_omega(&p);
    const q16_t t_before = esp_foc_rotor_pll_get_theta(&p);
    const uint32_t updates = p.updates;

    const int coast = 25;
    for (int i = 0; i < coast; i++) {
        esp_foc_rotor_pll_step(&p, 0, false);
    }

    /* A missing measurement must not move ω̂, and must still advance θ̂. */
    TEST_ASSERT_EQUAL_INT32(w_before, esp_foc_rotor_pll_get_omega(&p));
    TEST_ASSERT_EQUAL_UINT32(updates, p.updates);
    TEST_ASSERT_EQUAL_UINT32((uint32_t)coast, p.coasted);
    TEST_ASSERT_EQUAL_INT32(0, esp_foc_rotor_pll_get_phase_err(&p));

    const double advanced = (double)q16_to_float(
        q16_angle_delta(t_before, esp_foc_rotor_pll_get_theta(&p)));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, omega * coast * DT_S, advanced);
}

TEST_CASE("rotor_pll: omega ceiling clamps and counts", "[rotor_pll]")
{
    esp_foc_rotor_pll_t p;
    TEST_ASSERT_EQUAL(ESP_OK, make(&p, BW_HZ, ZETA, 50.0f));

    double th = 0.0;
    spin(&p, 400.0, 3000, &th);

    TEST_ASSERT_DOUBLE_WITHIN(
        0.01, 50.0, (double)q16_to_float(esp_foc_rotor_pll_get_omega(&p)));
    TEST_ASSERT_TRUE(p.clamped > 0u);
}

TEST_CASE("rotor_pll: seed and reset", "[rotor_pll]")
{
    esp_foc_rotor_pll_t p;
    TEST_ASSERT_EQUAL(ESP_OK, make(&p, BW_HZ, ZETA, 0.0f));

    esp_foc_rotor_pll_seed(&p, q16_from_float(1.5f), q16_from_float(77.0f));
    TEST_ASSERT_DOUBLE_WITHIN(
        1e-3, 1.5, (double)q16_to_float(esp_foc_rotor_pll_get_theta(&p)));
    TEST_ASSERT_DOUBLE_WITHIN(
        1e-3, 77.0, (double)q16_to_float(esp_foc_rotor_pll_get_omega(&p)));

    /* Seeding wraps, so a caller may hand over an unwrapped angle. */
    esp_foc_rotor_pll_seed(&p, q16_from_float(4.0f), 0);
    TEST_ASSERT_DOUBLE_WITHIN(
        1e-3, 4.0 - 2.0 * M_PI,
        (double)q16_to_float(esp_foc_rotor_pll_get_theta(&p)));

    esp_foc_rotor_pll_reset(&p);
    TEST_ASSERT_EQUAL_INT32(0, esp_foc_rotor_pll_get_theta(&p));
    TEST_ASSERT_EQUAL_INT32(0, esp_foc_rotor_pll_get_omega(&p));
    TEST_ASSERT_EQUAL_UINT32(0u, p.updates);
    TEST_ASSERT_TRUE(p.inited);
}

/* --- port layer --------------------------------------------------------- */

typedef struct {
    esp_foc_rotor_sensor_t base;
    esp_foc_rotor_state_t st;
} fake_sensor_t;

static void fake_snapshot(const esp_foc_rotor_sensor_t *s,
                          esp_foc_rotor_state_t *out)
{
    *out = ((const fake_sensor_t *)s)->st;
}

TEST_CASE("rotor_pll: update() honours the sensor contract", "[rotor_pll]")
{
    /*
     * A held latch is not a measurement. as5600 keeps returning the previous
     * angle after a failed transfer, and integrating that as genuine
     * zero-travel is what drags ω̂ toward zero on a spinning rotor.
     */
    fake_sensor_t f;
    memset(&f, 0, sizeof(f));
    f.base.snapshot = fake_snapshot;

    esp_foc_rotor_pll_t p;
    esp_foc_rotor_pll_config_t cfg;
    esp_foc_rotor_pll_config_default(&cfg, STEP_HZ, BW_HZ, ZETA, 0.0f);
    cfg.domain = ESP_FOC_ROTOR_PLL_ELEC;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_rotor_pll_init(&p, &cfg));

    double th = 0.0;
    const double omega = 250.0;
    f.st.valid = true;

    for (int i = 0; i < 4000; i++) {
        th += omega * DT_S;
        f.st.theta_e = encode(th);
        f.st.theta_m = 0; /* wrong domain on purpose: must be ignored */
        f.st.seq++;
        esp_foc_rotor_pll_update(&p, &f.base);
    }
    TEST_ASSERT_DOUBLE_WITHIN(
        omega * 0.03, omega,
        (double)q16_to_float(esp_foc_rotor_pll_get_omega(&p)));

    const q16_t w_before = esp_foc_rotor_pll_get_omega(&p);
    const uint32_t coasted = p.coasted;

    /* seq frozen: same latch handed over ten times. */
    for (int i = 0; i < 10; i++) {
        esp_foc_rotor_pll_update(&p, &f.base);
    }
    TEST_ASSERT_EQUAL_INT32(w_before, esp_foc_rotor_pll_get_omega(&p));
    TEST_ASSERT_EQUAL_UINT32(coasted + 10u, p.coasted);

    /* Invalid snapshot coasts too, even with a moving seq. */
    f.st.valid = false;
    f.st.seq++;
    esp_foc_rotor_pll_update(&p, &f.base);
    TEST_ASSERT_EQUAL_UINT32(coasted + 11u, p.coasted);

    /* A NULL sensor must not fault; snapshot() zeroes and valid stays false. */
    esp_foc_rotor_pll_update(&p, NULL);
    TEST_ASSERT_EQUAL_UINT32(coasted + 12u, p.coasted);
}
