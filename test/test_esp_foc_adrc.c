/*
 * Unit tests for esp_foc_adrc (first-order linear ADRC).
 *
 * Plant: one RL axis, input in pu of Vdc, exact ZOH. The input computed at
 * sample k acts over [k+d, k+d+1]: d = 0 is an ideal bridge, d = 2 the MCPWM
 * one (sample a period old, duty loaded on the next TEZ).
 */
#include <math.h>
#include <string.h>

#include "unity.h"
#include "esp_err.h"
#include "espFoC/utils/esp_foc_adrc.h"
#include "espFoC/utils/esp_foc_q16.h"

#define RS      1.0f
#define LS      0.5e-3f
#define VDC     12.0f
#define FS      20000.0f
#define TS      (1.0f / FS)
#define VMAX    0.57735f
#define B0      (VDC / LS)
#define POLE    (RS / LS)

typedef struct {
    float a;
    float b_u;      /* A per pu over one sample */
    float b_v;      /* A per volt over one sample */
    float i;
    float uq[ESP_FOC_ADRC_DELAY_MAX + 1];   /* computed inputs, newest first */
    uint32_t d;
    float v_dist;   /* volts, e.g. back-EMF */
} rl_t;

static void rl_init_d(rl_t *p, float rs, float ls, uint32_t d)
{
    memset(p, 0, sizeof(*p));
    p->a = expf(-rs / ls * TS);
    p->b_v = (1.0f - p->a) / rs;
    p->b_u = p->b_v * VDC;
    p->d = d;
}

static void rl_init(rl_t *p, float rs, float ls)
{
    rl_init_d(p, rs, ls, 1u);
}

static void rl_advance(rl_t *p, float u)
{
    for (uint32_t j = ESP_FOC_ADRC_DELAY_MAX; j > 0u; j--) {
        p->uq[j] = p->uq[j - 1u];
    }
    p->uq[0] = u;
    p->i = p->a * p->i + p->b_u * p->uq[p->d] - p->b_v * p->v_dist;
}

static q16_t clampv(q16_t u)
{
    return q16_clamp(u, q16_from_float(-VMAX), q16_from_float(VMAX));
}

/* One control period; returns the current measured at its start. */
static float run_step(esp_foc_adrc_t *c, rl_t *p, float ref)
{
    const float y = p->i;
    const q16_t u = clampv(esp_foc_adrc_update(c, q16_from_float(ref), q16_from_float(y)));
    esp_foc_adrc_set_applied(c, u);
    rl_advance(p, q16_to_float(u));
    return y;
}

typedef struct {
    float peak;
    float final;
    float tail_p2p;
    int settle;     /* first sample after which |e| stays within 2 % */
} step_t;

static step_t run_profile(esp_foc_adrc_t *c, rl_t *p, float ref, int n)
{
    step_t s = { .peak = -1e9f, .settle = -1 };
    float tmin = 1e9f, tmax = -1e9f;
    for (int k = 0; k < n; k++) {
        const float y = run_step(c, p, ref);
        if (y > s.peak) {
            s.peak = y;
        }
        if (fabsf(y - ref) > 0.02f * fabsf(ref)) {
            s.settle = -1;
        } else if (s.settle < 0) {
            s.settle = k;
        }
        if (k >= n - n / 5) {
            tmin = fminf(tmin, y);
            tmax = fmaxf(tmax, y);
        }
        s.final = y;
    }
    s.tail_p2p = tmax - tmin;
    return s;
}

TEST_CASE("adrc init rejects bad args", "[espFoC][adrc]")
{
    esp_foc_adrc_t c;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_adrc_init(NULL, B0, POLE, 100.0f, 500.0f, TS, 1u));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_adrc_init(&c, 0.0f, POLE, 100.0f, 500.0f, TS, 1u));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_adrc_init(&c, B0, POLE, 0.0f, 500.0f, TS, 1u));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_adrc_init(&c, B0, POLE, 500.0f, 500.0f, TS, 1u));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_adrc_init(&c, B0, POLE, 100.0f, 10000.0f, TS, 1u));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_adrc_init(&c, B0, POLE, 100.0f, 500.0f, 0.0f, 1u));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_adrc_init(&c, NAN, POLE, 100.0f, 500.0f, TS, 1u));
    /* g = b0·Ts out of the representable band */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_adrc_init(&c, 1e8f, POLE, 100.0f, 500.0f, TS, 1u));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_adrc_init(&c, 1.0f, POLE, 100.0f, 500.0f, TS, 1u));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      esp_foc_adrc_init(&c, B0, POLE, 100.0f, 500.0f, TS, ESP_FOC_ADRC_DELAY_MAX + 1u));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_adrc_init(&c, B0, -1.0f, 100.0f, 500.0f, TS, 1u));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_adrc_init(&c, B0, NAN, 100.0f, 500.0f, TS, 1u));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_adrc_init(&c, B0, 5.0f * FS, 100.0f, 500.0f, TS, 1u));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_adrc_init(&c, B0, 0.0f, 100.0f, 500.0f, TS, 1u));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_adrc_init(&c, B0, POLE, 100.0f, 500.0f, TS, 1u));
}

/* Samples to 90 % of ref, -1 if never. */
static int t90_of(esp_foc_adrc_t *c, rl_t *p, float ref, int n)
{
    for (int k = 0; k < n; k++) {
        if (run_step(c, p, ref) >= 0.9f * ref) {
            return k;
        }
    }
    return -1;
}

/* Bench-like axis: Rs/Ls ~ 1.1 kHz, above a 500 Hz ESO, sample 2 periods old. */
#define FAST_RS 2.8f
#define FAST_LS 414e-6f

TEST_CASE("adrc with the pole modelled follows a plant faster than its ESO", "[espFoC][adrc]")
{
    esp_foc_adrc_t c;
    rl_t p;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_adrc_init(&c, VDC / FAST_LS, FAST_RS / FAST_LS, 100.0f,
                                                500.0f, TS, 2u));
    rl_init_d(&p, FAST_RS, FAST_LS, 2u);
    p.v_dist = 0.3f;
    /* 100 Hz first order: t90 = 2.3 tau = 73 samples, plus the delay */
    const int t90 = t90_of(&c, &p, 0.4f, 1000);
    TEST_ASSERT_TRUE(t90 > 0);
    TEST_ASSERT_TRUE(t90 < 90);
    const step_t s = run_profile(&c, &p, 0.4f, 3000);
    TEST_ASSERT_FLOAT_WITHIN(0.004f, 0.4f, s.final);
    TEST_ASSERT_TRUE(s.peak < 0.42f);
    TEST_ASSERT_TRUE(s.tail_p2p < 0.001f);
}

TEST_CASE("adrc without the pole is slow on a plant faster than its ESO", "[espFoC][adrc]")
{
    esp_foc_adrc_t c;
    rl_t p;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_adrc_init(&c, VDC / FAST_LS, 0.0f, 100.0f, 500.0f, TS, 2u));
    rl_init_d(&p, FAST_RS, FAST_LS, 2u);
    const int t90 = t90_of(&c, &p, 0.4f, 4000);
    TEST_ASSERT_TRUE((t90 < 0) || (t90 > 300));
}

TEST_CASE("adrc tolerates a modelled pole off by 0.7x and 1.3x", "[espFoC][adrc]")
{
    const float scale[] = { 0.7f, 1.3f };
    for (int j = 0; j < 2; j++) {
        esp_foc_adrc_t c;
        rl_t p;
        TEST_ASSERT_EQUAL(ESP_OK, esp_foc_adrc_init(&c, VDC / FAST_LS,
                                                    scale[j] * FAST_RS / FAST_LS, 100.0f,
                                                    500.0f, TS, 2u));
        rl_init_d(&p, FAST_RS, FAST_LS, 2u);
        const int t90 = t90_of(&c, &p, 0.4f, 1000);
        TEST_ASSERT_TRUE(t90 > 0);
        TEST_ASSERT_TRUE(t90 < 150);
        const step_t s = run_profile(&c, &p, 0.4f, 3000);
        TEST_ASSERT_FLOAT_WITHIN(0.004f, 0.4f, s.final);
        TEST_ASSERT_TRUE(s.peak < 0.44f);
        TEST_ASSERT_TRUE(s.tail_p2p < 0.001f);
    }
}

TEST_CASE("adrc step settles to the reference without overshoot", "[espFoC][adrc]")
{
    esp_foc_adrc_t c;
    rl_t p;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_adrc_init(&c, B0, POLE, 100.0f, 500.0f, TS, 1u));
    rl_init(&p, RS, LS);
    const step_t s = run_profile(&c, &p, 1.0f, 4000);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, s.final);
    TEST_ASSERT_TRUE(s.peak < 1.05f);
    TEST_ASSERT_TRUE(s.tail_p2p < 0.002f);
    /* 100 Hz first order: ~4 tau to 2 %, tau = 1.6 ms = 32 samples */
    TEST_ASSERT_TRUE(s.settle > 0);
    TEST_ASSERT_TRUE(s.settle < 400);
}

TEST_CASE("adrc matched to the plant delay settles for 0, 1, 2 and 3 samples", "[espFoC][adrc]")
{
    for (uint32_t d = 0; d <= ESP_FOC_ADRC_DELAY_MAX; d++) {
        esp_foc_adrc_t c;
        rl_t p;
        TEST_ASSERT_EQUAL(ESP_OK, esp_foc_adrc_init(&c, B0, POLE, 150.0f, 750.0f, TS, d));
        rl_init_d(&p, RS, LS, d);
        p.v_dist = 2.0f;
        const step_t s = run_profile(&c, &p, 1.0f, 4000);
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, s.final);
        TEST_ASSERT_TRUE(s.peak < 1.05f);
        TEST_ASSERT_TRUE(s.tail_p2p < 0.002f);
        TEST_ASSERT_TRUE(s.settle > 0);
        TEST_ASSERT_TRUE(s.settle < 300);
    }
}

TEST_CASE("adrc at a quarter of fs with no delay, as on the mock bridge", "[espFoC][adrc]")
{
    const float fs = 5000.0f;
    esp_foc_adrc_t c;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_adrc_init(&c, 12.0f / 1e-3f, 2.0f / 1e-3f, 300.0f, 1500.0f, 1.0f / fs, 0u));
    const float a = expf(-2.0f / 1e-3f / fs);
    const float b = (1.0f - a) * 12.0f / 2.0f;
    float i = 0.0f, peak = 0.0f;
    for (int k = 0; k < 2000; k++) {
        const q16_t u = clampv(esp_foc_adrc_update(&c, q16_from_float(0.25f), q16_from_float(i)));
        esp_foc_adrc_set_applied(&c, u);
        i = a * i + b * q16_to_float(u);
        peak = fmaxf(peak, i);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.003f, 0.25f, i);
    TEST_ASSERT_TRUE(peak < 0.27f);
}

TEST_CASE("adrc set for 2 samples survives a plant at 1 or 3", "[espFoC][adrc]")
{
    const uint32_t plant_d[] = { 1u, 3u };
    for (int j = 0; j < 2; j++) {
        esp_foc_adrc_t c;
        rl_t p;
        TEST_ASSERT_EQUAL(ESP_OK, esp_foc_adrc_init(&c, B0, POLE, 100.0f, 500.0f, TS, 2u));
        rl_init_d(&p, RS, LS, plant_d[j]);
        const step_t s = run_profile(&c, &p, 1.0f, 6000);
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, s.final);
        TEST_ASSERT_TRUE(s.peak < 1.25f);
        TEST_ASSERT_TRUE(s.tail_p2p < 0.003f);
    }
}

TEST_CASE("adrc rejects a constant back-EMF", "[espFoC][adrc]")
{
    esp_foc_adrc_t c;
    rl_t p;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_adrc_init(&c, B0, POLE, 150.0f, 750.0f, TS, 1u));
    rl_init(&p, RS, LS);
    (void)run_profile(&c, &p, 0.8f, 2000);
    p.v_dist = 3.0f;
    const step_t s = run_profile(&c, &p, 0.8f, 4000);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.8f, s.final);
    TEST_ASSERT_TRUE(s.tail_p2p < 0.002f);
}

TEST_CASE("adrc tolerates a b0 mismatch of 0.5x and 2x", "[espFoC][adrc]")
{
    const float scale[] = { 0.5f, 2.0f };
    for (int j = 0; j < 2; j++) {
        esp_foc_adrc_t c;
        rl_t p;
        TEST_ASSERT_EQUAL(ESP_OK, esp_foc_adrc_init(&c, B0 * scale[j], POLE, 100.0f, 500.0f, TS, 1u));
        rl_init(&p, RS, LS);
        const step_t s = run_profile(&c, &p, 1.0f, 6000);
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, s.final);
        TEST_ASSERT_TRUE(s.peak < 1.25f);
        TEST_ASSERT_TRUE(s.tail_p2p < 0.003f);
    }
}

TEST_CASE("adrc recovers from saturation without windup", "[espFoC][adrc]")
{
    esp_foc_adrc_t c;
    rl_t p;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_adrc_init(&c, B0, POLE, 100.0f, 500.0f, TS, 1u));
    rl_init(&p, RS, LS);
    /* Unreachable: steady state needs 50 A·1 Ω = 50 V on a 6.9 V limit. */
    (void)run_profile(&c, &p, 50.0f, 4000);
    const step_t s = run_profile(&c, &p, 1.0f, 4000);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, s.final);
    TEST_ASSERT_TRUE(s.settle > 0);
    TEST_ASSERT_TRUE(s.settle < 600);
}

TEST_CASE("adrc stays bounded over a long run with LSB noise", "[espFoC][adrc]")
{
    esp_foc_adrc_t c;
    rl_t p;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_adrc_init(&c, B0, POLE, 300.0f, 1500.0f, TS, 1u));
    rl_init(&p, RS, LS);
    uint32_t lfsr = 0xACE1u;
    const float ref = 0.5f;
    float u_max = 0.0f;
    double y_sum = 0.0;
    int n_tail = 0;
    for (int k = 0; k < 200000; k++) {
        lfsr = (lfsr >> 1) ^ (-(lfsr & 1u) & 0xB400u);
        /* +-32 LSB = +-0.5 mA on the measurement */
        const q16_t noise = (q16_t)(lfsr & 0x3Fu) - 32;
        const q16_t y = q16_add(q16_from_float(p.i), noise);
        const q16_t u = clampv(esp_foc_adrc_update(&c, q16_from_float(ref), y));
        esp_foc_adrc_set_applied(&c, u);
        rl_advance(&p, q16_to_float(u));
        u_max = fmaxf(u_max, fabsf(q16_to_float(u)));
        if (k >= 100000) {
            y_sum += p.i;
            n_tail++;
        }
    }
    TEST_ASSERT_TRUE(u_max < VMAX);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, ref, (float)(y_sum / n_tail));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, ref, p.i);
}

TEST_CASE("adrc reset clears state and output", "[espFoC][adrc]")
{
    esp_foc_adrc_t c;
    rl_t p;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_adrc_init(&c, B0, POLE, 100.0f, 500.0f, TS, 1u));
    rl_init(&p, RS, LS);
    (void)run_profile(&c, &p, 2.0f, 500);
    esp_foc_adrc_reset(&c);
    TEST_ASSERT_EQUAL(0, c.z1);
    TEST_ASSERT_EQUAL(0, c.z2);
    TEST_ASSERT_EQUAL(0, c.hist[0]);
    TEST_ASSERT_EQUAL(0, c.hist[1]);
    TEST_ASSERT_EQUAL(0, esp_foc_adrc_update(&c, 0, 0));
}
