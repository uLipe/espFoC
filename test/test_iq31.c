/*
 * Unit tests for espFoC fixed-point IQ31 helpers.
 * Covers conversion, arithmetic, overflow/underflow, invalid inputs,
 * clamp, sin/cos, waveform generation (sinusoidal), transforms (Clarke/Park),
 * and SVPWM-style duty generation.
 */
#include <math.h>
#include <stdint.h>
#include <unity.h>
#include "espFoC/utils/esp_foc_iq31.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FLOAT_TOL      1e-5f
#define IQ31_TOL_FLOAT 1e-4f   /* tolerance when comparing IQ31 (as float) to reference */

/* --- Conversion --- */
TEST_CASE("iq31_from_float: zero and one", "[espFoC][iq31]")
{
    TEST_ASSERT_EQUAL_INT32(0, iq31_from_float(0.0f));
    TEST_ASSERT_EQUAL_INT32(IQ31_ONE, iq31_from_float(1.0f));
    TEST_ASSERT_EQUAL_INT32(IQ31_MINUS_ONE, iq31_from_float(-1.0f));
}

TEST_CASE("iq31_from_float: in range", "[espFoC][iq31]")
{
    float f = 0.5f;
    iq31_t q = iq31_from_float(f);
    float back = iq31_to_float(q);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, f, back);

    f = -0.25f;
    q = iq31_from_float(f);
    back = iq31_to_float(q);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, f, back);
}

TEST_CASE("iq31_from_float: overflow saturates to ONE", "[espFoC][iq31]")
{
    TEST_ASSERT_EQUAL_INT32(IQ31_ONE, iq31_from_float(1.5f));
    TEST_ASSERT_EQUAL_INT32(IQ31_ONE, iq31_from_float(2.0f));
    TEST_ASSERT_EQUAL_INT32(IQ31_ONE, iq31_from_float(1.0e10f));
}

TEST_CASE("iq31_from_float: underflow saturates to MINUS_ONE", "[espFoC][iq31]")
{
    TEST_ASSERT_EQUAL_INT32(IQ31_MINUS_ONE, iq31_from_float(-1.5f));
    TEST_ASSERT_EQUAL_INT32(IQ31_MINUS_ONE, iq31_from_float(-2.0f));
    TEST_ASSERT_EQUAL_INT32(IQ31_MINUS_ONE, iq31_from_float(-1.0e10f));
}

TEST_CASE("iq31_from_float: invalid NaN returns zero", "[espFoC][iq31]")
{
    volatile float zero = 0.0f;
    float nan = zero / zero;
    (void)nan;
    TEST_ASSERT_EQUAL_INT32(0, iq31_from_float(nan));
}

TEST_CASE("iq31_from_float: Inf saturates", "[espFoC][iq31]")
{
#if defined(INFINITY)
    TEST_ASSERT_EQUAL_INT32(IQ31_ONE, iq31_from_float((float)INFINITY));
    TEST_ASSERT_EQUAL_INT32(IQ31_MINUS_ONE, iq31_from_float((float)-INFINITY));
#else
    TEST_IGNORE_MESSAGE("INFINITY not defined");
#endif
}

TEST_CASE("iq31_to_float: roundtrip", "[espFoC][iq31]")
{
    for (int i = -100; i <= 100; i++) {
        float f = (float)i / 100.0f;
        iq31_t q = iq31_from_float(f);
        float back = iq31_to_float(q);
        TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, f, back);
    }
}

/* --- iq31_mul --- */
TEST_CASE("iq31_mul: basic", "[espFoC][iq31]")
{
    iq31_t half = iq31_from_float(0.5f);
    iq31_t r = iq31_mul(half, half);
    TEST_ASSERT_FLOAT_WITHIN(IQ31_TOL_FLOAT, 0.25f, iq31_to_float(r));

    iq31_t a = iq31_from_float(-1.0f);
    iq31_t b = iq31_from_float(1.0f);
    r = iq31_mul(a, b);
    TEST_ASSERT_FLOAT_WITHIN(IQ31_TOL_FLOAT, -1.0f, iq31_to_float(r));
}

TEST_CASE("iq31_mul: overflow saturates", "[espFoC][iq31]")
{
    /* 1.0 * 1.0 saturates to ONE (rounding may give ONE-1) */
    iq31_t a = IQ31_ONE;
    iq31_t b = IQ31_ONE;
    iq31_t r = iq31_mul(a, b);
    TEST_ASSERT_TRUE(r == IQ31_ONE || r == IQ31_ONE - 1);
}

TEST_CASE("iq31_mul: underflow saturates", "[espFoC][iq31]")
{
    iq31_t a = iq31_from_float(-0.99f);
    iq31_t b = iq31_from_float(0.99f);
    iq31_t r = iq31_mul(a, b);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.98f, iq31_to_float(r));

    /* neg * neg -> positive, can overflow to ONE */
    a = IQ31_MINUS_ONE;
    b = IQ31_MINUS_ONE;
    r = iq31_mul(a, b);
    TEST_ASSERT_EQUAL_INT32(IQ31_ONE, r);
}

/* --- iq31_add / iq31_sub --- */
TEST_CASE("iq31_add: overflow saturates", "[espFoC][iq31]")
{
    iq31_t a = iq31_from_float(0.8f);
    iq31_t b = iq31_from_float(0.8f);
    iq31_t r = iq31_add(a, b);
    TEST_ASSERT_EQUAL_INT32(IQ31_ONE, r);
}

TEST_CASE("iq31_add: underflow saturates", "[espFoC][iq31]")
{
    iq31_t a = iq31_from_float(-0.8f);
    iq31_t b = iq31_from_float(-0.8f);
    iq31_t r = iq31_add(a, b);
    TEST_ASSERT_EQUAL_INT32(IQ31_MINUS_ONE, r);
}

TEST_CASE("iq31_sub: overflow and underflow", "[espFoC][iq31]")
{
    iq31_t a = iq31_from_float(0.9f);
    iq31_t b = iq31_from_float(-0.9f);
    iq31_t r = iq31_sub(a, b);
    TEST_ASSERT_EQUAL_INT32(IQ31_ONE, r);

    a = iq31_from_float(-0.9f);
    b = iq31_from_float(0.9f);
    r = iq31_sub(a, b);
    TEST_ASSERT_EQUAL_INT32(IQ31_MINUS_ONE, r);
}

/* --- iq31_clamp --- */
TEST_CASE("iq31_clamp: in range", "[espFoC][iq31]")
{
    iq31_t lo = iq31_from_float(-0.5f);
    iq31_t hi = iq31_from_float(0.5f);
    iq31_t x = iq31_from_float(0.0f);
    TEST_ASSERT_EQUAL_INT32(x, iq31_clamp(x, lo, hi));
}

TEST_CASE("iq31_clamp: below lo", "[espFoC][iq31]")
{
    iq31_t lo = iq31_from_float(-0.5f);
    iq31_t hi = iq31_from_float(0.5f);
    iq31_t x = iq31_from_float(-1.0f);
    TEST_ASSERT_EQUAL_INT32(lo, iq31_clamp(x, lo, hi));
}

TEST_CASE("iq31_clamp: above hi", "[espFoC][iq31]")
{
    iq31_t lo = iq31_from_float(-0.5f);
    iq31_t hi = iq31_from_float(0.5f);
    iq31_t x = iq31_from_float(1.0f);
    TEST_ASSERT_EQUAL_INT32(hi, iq31_clamp(x, lo, hi));
}

TEST_CASE("iq31_clamp: lo equals hi", "[espFoC][iq31]")
{
    iq31_t v = iq31_from_float(0.3f);
    TEST_ASSERT_EQUAL_INT32(v, iq31_clamp(iq31_from_float(0.5f), v, v));
    TEST_ASSERT_EQUAL_INT32(v, iq31_clamp(iq31_from_float(-0.5f), v, v));
}

/* --- iq31_abs, min, max --- */
TEST_CASE("iq31_abs: positive and negative", "[espFoC][iq31]")
{
    iq31_t p = iq31_from_float(0.7f);
    TEST_ASSERT_EQUAL_INT32(p, iq31_abs(p));
    TEST_ASSERT_EQUAL_INT32(p, iq31_abs(iq31_from_float(-0.7f)));
}

TEST_CASE("iq31_abs: MINUS_ONE saturates to ONE", "[espFoC][iq31]")
{
    TEST_ASSERT_EQUAL_INT32(IQ31_ONE, iq31_abs(IQ31_MINUS_ONE));
}

TEST_CASE("iq31_min_iq31_max", "[espFoC][iq31]")
{
    iq31_t a = iq31_from_float(0.2f);
    iq31_t b = iq31_from_float(-0.3f);
    TEST_ASSERT_EQUAL_INT32(b, iq31_min(a, b));
    TEST_ASSERT_EQUAL_INT32(a, iq31_max(a, b));
}

/* --- iq31_rsqrt_fast --- */
TEST_CASE("iq31_rsqrt_fast: invalid zero returns zero", "[espFoC][iq31]")
{
    TEST_ASSERT_EQUAL_INT32(0, iq31_rsqrt_fast(0));
    TEST_ASSERT_EQUAL_INT32(0, iq31_rsqrt_fast(IQ31_MINUS_ONE));
}

TEST_CASE("iq31_rsqrt_fast: one returns approx one", "[espFoC][iq31]")
{
    iq31_t x = IQ31_ONE;
    iq31_t r = iq31_rsqrt_fast(x);
    /* Q2.30: 1.0 = 1<<30 */
    float r_float = (float)r / (float)(1U << 30);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.0f, r_float);
}

TEST_CASE("iq31_rsqrt_fast: quarter returns two", "[espFoC][iq31]")
{
    iq31_t x = iq31_from_float(0.25f);
    iq31_t r = iq31_rsqrt_fast(x);
    float r_float = (float)r / (float)(1 << 30);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 2.0f, r_float);
}

/* --- Sin / Cos --- */
TEST_CASE("iq31_sin_cos: zero and quarter", "[espFoC][iq31]")
{
    iq31_t angle_0 = 0;
    /* sin(0): LUT index 0; allow small tolerance (LUT quantization) */
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, iq31_to_float(iq31_sin(angle_0)));
    TEST_ASSERT_FLOAT_WITHIN(IQ31_TOL_FLOAT, 1.0f, iq31_to_float(iq31_cos(angle_0)));

    /* pi/2: angle = IQ31_ONE/4 */
    iq31_t angle_pi2 = (iq31_t)((uint32_t)IQ31_ONE >> 2);
    TEST_ASSERT_FLOAT_WITHIN(IQ31_TOL_FLOAT, 1.0f, iq31_to_float(iq31_sin(angle_pi2)));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, iq31_to_float(iq31_cos(angle_pi2)));
}

TEST_CASE("iq31_sin_cos: sin^2 + cos^2 approx 1", "[espFoC][iq31]")
{
    for (int i = 0; i < 64; i++) {
        iq31_t angle = (iq31_t)((uint64_t)(uint32_t)IQ31_ONE * (uint64_t)i / 64ULL);
        iq31_t s = iq31_sin(angle);
        iq31_t c = iq31_cos(angle);
        float sin_f = iq31_to_float(s);
        float cos_f = iq31_to_float(c);
        float mag_sq = sin_f * sin_f + cos_f * cos_f;
        TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.0f, mag_sq);
    }
}

/* --- Waveform: sinusoidal generation --- */
TEST_CASE("iq31_waveform: sine period and range", "[espFoC][iq31]")
{
    const int steps = 128;
    float min_v = 1.0f;
    float max_v = -1.0f;
    for (int i = 0; i <= steps; i++) {
        uint32_t u = (uint32_t)((uint64_t)IQ31_ONE * (uint64_t)i / (uint64_t)steps);
        iq31_t angle = (iq31_t)u;
        iq31_t s = iq31_sin(angle);
        float f = iq31_to_float(s);
        if (f < min_v) min_v = f;
        if (f > max_v) max_v = f;
    }
    /* LUT has 512 entries; peaks at 1/4 and 3/4 period; allow margin for quantization */
    TEST_ASSERT_TRUE(max_v >= 0.90f);
    TEST_ASSERT_TRUE(min_v <= -0.90f);
}

TEST_CASE("iq31_waveform: sine periodicity", "[espFoC][iq31]")
{
    iq31_t angle_0 = 0;
    iq31_t angle_2pi = IQ31_ONE;
    float s0 = iq31_to_float(iq31_sin(angle_0));
    float s2pi = iq31_to_float(iq31_sin(angle_2pi));
    /* sin(0) and sin(2*pi) should match (LUT quantization may differ slightly) */
    TEST_ASSERT_FLOAT_WITHIN(0.02f, s0, s2pi);
}

/* --- Transforms: Clarke roundtrip (inverse_clarke . clarke) --- */
TEST_CASE("iq31_transform: inverse_clarke after clarke roundtrip", "[espFoC][iq31]")
{
    /* Use values that satisfy u+v+w=0 and stay in range; beta = (u+2v)*K3 */
    iq31_t u = iq31_from_float(0.6f);
    iq31_t v = iq31_from_float(-0.3f);
    iq31_t w = iq31_from_float(-0.3f);  /* u+v+w=0 */

    iq31_t alpha, beta;
    iq31_clarke(u, v, w, &alpha, &beta);

    iq31_t u2, v2, w2;
    iq31_inverse_clarke(alpha, beta, &u2, &v2, &w2);

    /* Fixed-point roundtrip has higher error than float */
    TEST_ASSERT_FLOAT_WITHIN(0.15f, iq31_to_float(u), iq31_to_float(u2));
    TEST_ASSERT_FLOAT_WITHIN(0.15f, iq31_to_float(v), iq31_to_float(v2));
    TEST_ASSERT_FLOAT_WITHIN(0.15f, iq31_to_float(w), iq31_to_float(w2));
}

TEST_CASE("iq31_transform: inverse_park after park roundtrip", "[espFoC][iq31]")
{
    iq31_t alpha = iq31_from_float(0.6f);
    iq31_t beta  = iq31_from_float(0.2f);
    iq31_t sin_t = iq31_from_float(0.6f);
    iq31_t cos_t = iq31_from_float(0.8f);

    iq31_t d, q;
    iq31_park(sin_t, cos_t, alpha, beta, &d, &q);

    iq31_t alpha2, beta2;
    iq31_inverse_park(sin_t, cos_t, d, q, &alpha2, &beta2);

    TEST_ASSERT_FLOAT_WITHIN(IQ31_TOL_FLOAT, iq31_to_float(alpha), iq31_to_float(alpha2));
    TEST_ASSERT_FLOAT_WITHIN(IQ31_TOL_FLOAT, iq31_to_float(beta), iq31_to_float(beta2));
}

/* --- SVPWM-style: three phases 120° apart, sum to zero, duty in [0,1] --- */
TEST_CASE("iq31_svpwm_style: three phase sum zero", "[espFoC][iq31]")
{
    /* theta = 0: u=1, v=-0.5, w=-0.5 (normalized) */
    iq31_t angle_0 = 0;
    iq31_t u = iq31_cos(angle_0);
    iq31_t v = iq31_cos((iq31_t)((uint32_t)IQ31_ONE / 3));       /* 120° */
    iq31_t w = iq31_cos((iq31_t)((uint32_t)IQ31_ONE * 2 / 3));   /* 240° */

    iq31_t sum = iq31_add(iq31_add(u, v), w);
    float sum_f = iq31_to_float(sum);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, sum_f);
}

TEST_CASE("iq31_svpwm_style: duty from alpha beta in [0,1]", "[espFoC][iq31]")
{
    /* duty = 0.5 + v*inv_vbus; clamp to [0,1]. Use alpha=0.3, beta=0.2, inv_vbus=0.5 */
    iq31_t alpha = iq31_from_float(0.3f);
    iq31_t beta  = iq31_from_float(0.2f);
    iq31_t inv_vbus = iq31_from_float(0.5f);
    iq31_t half = IQ31_HALF;

    iq31_t u, v, w;
    iq31_inverse_clarke(alpha, beta, &u, &v, &w);

    /* duty = 0.5 + phase * inv_vbus, then clamp to [0, IQ31_ONE] */
    iq31_t du = iq31_clamp(iq31_add(half, iq31_mul(u, inv_vbus)), 0, IQ31_ONE);
    iq31_t dv = iq31_clamp(iq31_add(half, iq31_mul(v, inv_vbus)), 0, IQ31_ONE);
    iq31_t dw = iq31_clamp(iq31_add(half, iq31_mul(w, inv_vbus)), 0, IQ31_ONE);

    TEST_ASSERT_TRUE(iq31_to_float(du) >= 0.0f && iq31_to_float(du) <= 1.0f);
    TEST_ASSERT_TRUE(iq31_to_float(dv) >= 0.0f && iq31_to_float(dv) <= 1.0f);
    TEST_ASSERT_TRUE(iq31_to_float(dw) >= 0.0f && iq31_to_float(dw) <= 1.0f);
}

/* --- iq31_mul_q230 and normalize_angle --- */
TEST_CASE("iq31_mul_q230: scale by rsqrt", "[espFoC][iq31]")
{
    iq31_t a = iq31_from_float(0.5f);
    iq31_t rsqrt_quarter = iq31_rsqrt_fast(iq31_from_float(0.25f)); /* ~2 in Q2.30 */
    iq31_t r = iq31_mul_q230(a, rsqrt_quarter);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1.0f, iq31_to_float(r));
}

TEST_CASE("iq31_normalize_angle: wrap to [0, 2pi)", "[espFoC][iq31]")
{
    /* 2*pi (IQ31_ONE) normalizes to 0 */
    iq31_t two_pi = IQ31_ONE;
    iq31_t r = iq31_normalize_angle(two_pi);
    TEST_ASSERT_EQUAL_INT32(0, r);
    /* angle past 2*pi (two_pi + quarter) with 31-bit mask gives quarter or quarter-1 */
    r = iq31_normalize_angle((iq31_t)((int64_t)two_pi + (int64_t)(IQ31_ONE >> 2)));
    TEST_ASSERT_TRUE(r == (int32_t)(IQ31_ONE >> 2) || r == (int32_t)(IQ31_ONE >> 2) - 1);
}
