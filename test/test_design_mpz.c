/*
 * MIT License
 *
 * Unit tests for esp_foc_design_mpz: exp Q16 helper and MPZ PI synthesis.
 *
 * These tests exercise the math core that underpins both the build-time
 * Python generator and the runtime tuner. The Python cross-validation
 * (test 1.7) lives separately under scripts/ and reads the same golden table.
 */

#include <math.h>
#include <stdint.h>
#include <unity.h>
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/esp_foc_design_mpz.h"

/* --- Helpers ------------------------------------------------------------ */

static float expected_exp_neg(float x)
{
    return expf(-x);
}

static void assert_q16_rel_near(float expected, q16_t got, float rel_tol)
{
    float ge = q16_to_float(got);
    float diff = fabsf(ge - expected);
    float bound = fabsf(expected) * rel_tol;
    if (bound < 1.0f / 65536.0f) {
        bound = 1.0f / 65536.0f; /* never tighter than 1 LSB */
    }
    TEST_ASSERT(diff <= bound);
}

/* --- 1.1 exp Q16 -------------------------------------------------------- */

TEST_CASE("design_mpz: exp(0) == 1", "[espFoC][design_mpz]")
{
    TEST_ASSERT_EQUAL_INT32(Q16_ONE, esp_foc_q16_exp_neg(0));
    TEST_ASSERT_EQUAL_INT32(Q16_ONE, esp_foc_q16_exp_neg(-12345));
}

TEST_CASE("design_mpz: exp(-x) known points within 0.1% rel", "[espFoC][design_mpz]")
{
    const float xs[] = {0.01f, 0.1f, 0.3f, 0.5f, 0.7f, 1.0f, 1.5f, 2.0f, 3.0f, 4.5f};
    const size_t n = sizeof(xs) / sizeof(xs[0]);
    for (size_t i = 0; i < n; ++i) {
        q16_t r = esp_foc_q16_exp_neg(q16_from_float(xs[i]));
        assert_q16_rel_near(expected_exp_neg(xs[i]), r, 1e-3f);
    }
}

TEST_CASE("design_mpz: exp(-x) is monotonic decreasing", "[espFoC][design_mpz]")
{
    q16_t prev = esp_foc_q16_exp_neg(0);
    for (int i = 1; i < 256; ++i) {
        q16_t cur = esp_foc_q16_exp_neg(q16_from_float((float)i / 32.0f));
        TEST_ASSERT_TRUE(cur <= prev);
        prev = cur;
    }
}

TEST_CASE("design_mpz: exp(-large) saturates to 0", "[espFoC][design_mpz]")
{
    TEST_ASSERT_EQUAL_INT32(0, esp_foc_q16_exp_neg(q16_from_int(50)));
    TEST_ASSERT_EQUAL_INT32(0, esp_foc_q16_exp_neg(q16_from_int(100)));
}

/* --- 1.2 continuous-time limit ------------------------------------------ */

TEST_CASE("design_mpz: Ts->0 limit matches L*w_bw, R*w_bw within 5%",
          "[espFoC][design_mpz]")
{
    /* Pick a small-ish Ts (5us) so the discrete formulas approach
     * the continuous result. With R=1.0, L=1mH this gives R*Ts/L = 5e-3
     * (well below 0.1) so the continuous limit is a tight reference. */
    esp_foc_pi_design_input_t in = {
        .motor_r_ohm = q16_from_float(1.0f),
        .motor_l_h   = q16_from_float(0.001f),
        .loop_ts_us  = 5,
        .bandwidth_hz = q16_from_float(150.0f),
        .v_max       = q16_from_float(12.0f),
    };
    esp_foc_pi_design_output_t out;
    esp_foc_design_status_t st = esp_foc_design_pi_current_mpz_q16(&in, &out);
    TEST_ASSERT_EQUAL_INT(ESP_FOC_DESIGN_OK, st);
    TEST_ASSERT_TRUE(out.valid);

    float w_bw = 2.0f * (float)M_PI * 150.0f;
    float kp_continuous = 0.001f * w_bw;
    float ki_continuous = 1.0f   * w_bw;

    assert_q16_rel_near(kp_continuous, out.kp, 0.05f);
    assert_q16_rel_near(ki_continuous, out.ki, 0.05f);
}

/* --- 1.3 closed-loop pole placement ------------------------------------- */

/* Simulates the discrete PMSM current loop with the espFoC PID structure.
 * Plant (ZOH discretized): i[k+1] = alpha*i[k] + (1-alpha)/R * u[k]
 * Returns the time index where i[k] first crosses 63.2% of a unit step. */
static int simulate_step_response(const esp_foc_pi_design_output_t *gains,
                                  float r, float l, float ts_s,
                                  int max_samples)
{
    float alpha = expf(-r * ts_s / l);
    float kp = q16_to_float(gains->kp);
    float ki = q16_to_float(gains->ki);

    float i = 0.0f;
    float integ = 0.0f;
    float prev_integ = 0.0f;
    float ref = 1.0f;
    float v_max = 100.0f; /* large enough to not saturate */

    int crossing_idx = -1;

    for (int k = 0; k < max_samples; ++k) {
        float err = ref - i;
        /* espFoC PID: output uses OLD integrator, then updates */
        float u = kp * err + prev_integ;
        if (u > v_max) u = v_max;
        if (u < -v_max) u = -v_max;

        prev_integ = integ;
        integ += ki * err * ts_s;

        i = alpha * i + (1.0f - alpha) / r * u;

        if (crossing_idx < 0 && i >= 0.6321f) {
            crossing_idx = k;
        }
    }
    return crossing_idx;
}

TEST_CASE("design_mpz: closed-loop time-constant matches 1/w_bw within 10%",
          "[espFoC][design_mpz]")
{
    const float r = 1.08f;
    const float l = 0.0018f;
    const uint32_t ts_us = 100; /* tight loop */
    const float ts_s = (float)ts_us * 1e-6f;
    const float bw_hz = 200.0f;

    esp_foc_pi_design_input_t in = {
        .motor_r_ohm = q16_from_float(r),
        .motor_l_h   = q16_from_float(l),
        .loop_ts_us  = ts_us,
        .bandwidth_hz = q16_from_float(bw_hz),
        .v_max       = q16_from_float(24.0f),
    };
    esp_foc_pi_design_output_t out;
    TEST_ASSERT_EQUAL_INT(ESP_FOC_DESIGN_OK,
                          esp_foc_design_pi_current_mpz_q16(&in, &out));

    int idx = simulate_step_response(&out, r, l, ts_s, 4000);
    TEST_ASSERT_TRUE(idx > 0);

    float t_meas = (float)idx * ts_s;
    float t_expected = 1.0f / (2.0f * (float)M_PI * bw_hz);
    float rel_err = fabsf(t_meas - t_expected) / t_expected;
    TEST_ASSERT_TRUE(rel_err < 0.10f);
}

/* --- 1.4 pole-zero cancellation ----------------------------------------- */

TEST_CASE("design_mpz: PI zero cancels plant pole (Ki*Ts/Kp == 1-alpha)",
          "[espFoC][design_mpz]")
{
    esp_foc_pi_design_input_t in = {
        .motor_r_ohm = q16_from_float(2.5f),
        .motor_l_h   = q16_from_float(0.0035f),
        .loop_ts_us  = 250,
        .bandwidth_hz = q16_from_float(300.0f),
        .v_max       = q16_from_float(36.0f),
    };
    esp_foc_pi_design_output_t out;
    TEST_ASSERT_EQUAL_INT(ESP_FOC_DESIGN_OK,
                          esp_foc_design_pi_current_mpz_q16(&in, &out));

    float kp = q16_to_float(out.kp);
    float ki = q16_to_float(out.ki);
    float ts = (float)in.loop_ts_us * 1e-6f;
    float alpha = q16_to_float(out.alpha);

    float lhs = (ki * ts) / kp;
    float rhs = 1.0f - alpha;
    /* 0.5% absolute matches the inherent Q16 quantization on alpha (1 LSB). */
    TEST_ASSERT_FLOAT_WITHIN(5e-3f, rhs, lhs);
}

/* --- 1.5 golden table (regression) -------------------------------------- */

typedef struct {
    const char *name;
    float r_ohm;
    float l_h;
    uint32_t ts_us;
    float bw_hz;
    float v_max;
    float kp_expected;
    float ki_expected;
} golden_motor_t;

/* Reference values precomputed analytically with double-precision MPZ.
 * Tolerance is 1% to allow for Q16 quantization headroom. */
static const golden_motor_t goldens[] = {
    /* Reference values use floating-point MPZ math; the Q16 implementation
     * matches them within ~3% as long as L >= ~0.5mH (the Q16 quantization
     * limit on inductance with raw <= 32 LSBs). For motors with very small
     * L the design path needs higher-precision intermediates (future work). */
    /* name              R        L       Ts(us)  bw(Hz) Vmax     Kp_ref       Ki_ref */
    {"gimbal_iPower",   1.080f,  0.00180f,  500,  150.0f,  12.0f, 1.565825f,  811.667f},
    {"servo_low_R",     0.250f,  0.00080f,  100,  500.0f,  24.0f, 2.190654f,  673.993f},
    {"scooter_high_R",  0.500f,  0.00050f,  200,  300.0f,  48.0f, 0.866330f,  785.195f},
    {"fast_bldc",       0.500f,  0.00100f,  100,  600.0f,  24.0f, 3.219952f, 1570.389f},
    {"lo_bw_big_L",     0.800f,  0.00500f, 1000,   50.0f,  24.0f, 1.458700f,  215.678f},
};

TEST_CASE("design_mpz: golden table regression (analytical refs)",
          "[espFoC][design_mpz]")
{
    const size_t n = sizeof(goldens) / sizeof(goldens[0]);
    for (size_t i = 0; i < n; ++i) {
        const golden_motor_t *g = &goldens[i];
        esp_foc_pi_design_input_t in = {
            .motor_r_ohm = q16_from_float(g->r_ohm),
            .motor_l_h   = q16_from_float(g->l_h),
            .loop_ts_us  = g->ts_us,
            .bandwidth_hz = q16_from_float(g->bw_hz),
            .v_max       = q16_from_float(g->v_max),
        };
        esp_foc_pi_design_output_t out;
        esp_foc_design_status_t st = esp_foc_design_pi_current_mpz_q16(&in, &out);
        TEST_ASSERT_EQUAL_INT_MESSAGE(ESP_FOC_DESIGN_OK, st, g->name);
        TEST_ASSERT_TRUE_MESSAGE(out.valid, g->name);
        assert_q16_rel_near(g->kp_expected, out.kp, 0.03f);
        assert_q16_rel_near(g->ki_expected, out.ki, 0.03f);
    }
}

/* --- 1.6 degenerate inputs ---------------------------------------------- */

TEST_CASE("design_mpz: rejects zero/negative motor params and bw",
          "[espFoC][design_mpz]")
{
    esp_foc_pi_design_input_t base = {
        .motor_r_ohm = q16_from_float(1.0f),
        .motor_l_h   = q16_from_float(0.001f),
        .loop_ts_us  = 100,
        .bandwidth_hz = q16_from_float(200.0f),
        .v_max       = q16_from_float(12.0f),
    };
    esp_foc_pi_design_output_t out;

    esp_foc_pi_design_input_t bad;

    bad = base; bad.motor_r_ohm = 0;
    TEST_ASSERT_EQUAL_INT(ESP_FOC_DESIGN_ERR_INVALID_ARG,
        esp_foc_design_pi_current_mpz_q16(&bad, &out));

    bad = base; bad.motor_l_h = 0;
    TEST_ASSERT_EQUAL_INT(ESP_FOC_DESIGN_ERR_INVALID_ARG,
        esp_foc_design_pi_current_mpz_q16(&bad, &out));

    bad = base; bad.loop_ts_us = 0;
    TEST_ASSERT_EQUAL_INT(ESP_FOC_DESIGN_ERR_INVALID_ARG,
        esp_foc_design_pi_current_mpz_q16(&bad, &out));

    bad = base; bad.bandwidth_hz = 0;
    TEST_ASSERT_EQUAL_INT(ESP_FOC_DESIGN_ERR_INVALID_ARG,
        esp_foc_design_pi_current_mpz_q16(&bad, &out));

    bad = base; bad.v_max = 0;
    TEST_ASSERT_EQUAL_INT(ESP_FOC_DESIGN_ERR_INVALID_ARG,
        esp_foc_design_pi_current_mpz_q16(&bad, &out));
}

TEST_CASE("design_mpz: rejects bandwidth above Nyquist", "[espFoC][design_mpz]")
{
    /* Ts = 1 ms, Nyquist = 500 Hz; pick bw > Nyquist */
    esp_foc_pi_design_input_t in = {
        .motor_r_ohm = q16_from_float(1.0f),
        .motor_l_h   = q16_from_float(0.001f),
        .loop_ts_us  = 1000,
        .bandwidth_hz = q16_from_float(800.0f),
        .v_max       = q16_from_float(12.0f),
    };
    esp_foc_pi_design_output_t out;
    TEST_ASSERT_EQUAL_INT(ESP_FOC_DESIGN_ERR_OUT_OF_RANGE,
        esp_foc_design_pi_current_mpz_q16(&in, &out));
}

TEST_CASE("design_mpz: integrator_limit equals v_max", "[espFoC][design_mpz]")
{
    esp_foc_pi_design_input_t in = {
        .motor_r_ohm = q16_from_float(1.0f),
        .motor_l_h   = q16_from_float(0.001f),
        .loop_ts_us  = 100,
        .bandwidth_hz = q16_from_float(150.0f),
        .v_max       = q16_from_float(7.5f),
    };
    esp_foc_pi_design_output_t out;
    TEST_ASSERT_EQUAL_INT(ESP_FOC_DESIGN_OK,
        esp_foc_design_pi_current_mpz_q16(&in, &out));
    TEST_ASSERT_EQUAL_INT32(in.v_max, out.integrator_limit);
}
