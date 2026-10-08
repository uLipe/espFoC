/*
 * Unit tests for the machine identification kernels.
 *
 * The plant is the exact zero-order-hold response of a first-order RL, which is
 * what a PWM bridge actually presents: v is held across the whole period, and
 * the sample handed to the probe is the state before the new command lands
 * (one-sample sense lag).
 */
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "espFoC/motor_control/esp_foc_ident.h"
#include "espFoC/utils/esp_foc_q16.h"

/* Reference bench motor: 1.9 ohm / 500 uH, R/L = 3800 rad/s => 605 Hz. */
#define REF_R 1.9
#define REF_L 500e-6

typedef struct {
    double a;
    double r;
    double i;
} rl_t;

static void rl_init(rl_t *p, double r_ohm, double l_h, double fs_hz)
{
    p->a = exp(-r_ohm / (l_h * fs_hz));
    p->r = r_ohm;
    p->i = 0.0;
}

/* Advance one held-voltage period. The state read before this call is the
 * measurement, which is what makes the sense lag exactly one sample. */
static void rl_apply(rl_t *p, double v)
{
    p->i = p->i * p->a + (v / p->r) * (1.0 - p->a);
}

static uint32_t s_rng;

static double frand_sym(void)
{
    s_rng = s_rng * 1664525u + 1013904223u;
    return 2.0 * ((double)(s_rng >> 8) / (double)(1u << 24)) - 1.0;
}

/**
 * Drive an RL plant through a bridge with a deadtime dead zone of @p dead_v.
 *
 * While the current is inside the dead zone the bridge has no gain at all: the
 * deadtime interval leaves the phase floating rather than driving it, so the
 * winding sees nothing until the command clears it.
 */
static bool probe_rl_bridge(double r_ohm, double l_h, unsigned f_hz,
                            unsigned fs_hz, double v_amp, double v_bias,
                            double noise_a, int trim_cdeg, double dead_v,
                            esp_foc_ident_z_t *z)
{
    esp_foc_ident_excite_t e = {
        .v_amp = q16_from_float((float)v_amp),
        .v_bias = q16_from_float((float)v_bias),
        .f_hz = f_hz,
        .fs_hz = fs_hz,
        .periods = 32,
        .settle_periods = 2,
        .lag_samples = 1,
        .phase_trim_cdeg = trim_cdeg,
    };
    esp_foc_ident_zprobe_t p;
    if (!esp_foc_ident_zprobe_init(&p, &e)) {
        return false;
    }

    rl_t plant;
    rl_init(&plant, r_ohm, l_h, (double)fs_hz);
    s_rng = 12345u;

    uint32_t guard = (p.n_skip + p.n_target) * 2u + 16u;
    for (uint32_t k = 0; k < guard && !esp_foc_ident_zprobe_done(&p); k++) {
        double meas = plant.i;
        if (noise_a > 0.0) {
            meas += noise_a * frand_sym();
        }
        q16_t v = esp_foc_ident_zprobe_step(&p, q16_from_float((float)meas));
        double applied = q16_to_float(v);
        if (dead_v > 0.0) {
            /*
             * The deadtime error follows the current, not the command: whichever
             * body diode the current picks decides the output while both gates
             * are off. Around zero current neither diode conducts and the phase
             * floats, which is what turns the drop into a dead band.
             */
            const double i_float = 0.02;
            if (plant.i > i_float) {
                applied -= dead_v;
            } else if (plant.i < -i_float) {
                applied += dead_v;
            } else if (applied > dead_v) {
                applied -= dead_v;
            } else if (applied < -dead_v) {
                applied += dead_v;
            } else {
                applied = 0.0;
            }
        }
        rl_apply(&plant, applied);
    }
    return esp_foc_ident_zprobe_solve(&p, z);
}

static bool probe_rl(double r_ohm, double l_h, unsigned f_hz, unsigned fs_hz,
                     double v_amp, double noise_a, int trim_cdeg,
                     esp_foc_ident_z_t *z)
{
    return probe_rl_bridge(r_ohm, l_h, f_hz, fs_hz, v_amp, 0.0, noise_a,
                           trim_cdeg, 0.0, z);
}

static int pct_err(int32_t got, double want)
{
    double e = 100.0 * ((double)got / want - 1.0);
    return (int)(e < 0.0 ? -e : e);
}

TEST_CASE("ident zprobe init rejects unusable excitations", "[espFoC][ident]")
{
    esp_foc_ident_zprobe_t p;
    esp_foc_ident_excite_t e = {
        .v_amp = q16_from_float(0.8f),
        .f_hz = 605,
        .fs_hz = 20000,
        .periods = 32,
        .settle_periods = 2,
        .lag_samples = 1,
    };
    TEST_ASSERT_TRUE(esp_foc_ident_zprobe_init(&p, &e));

    TEST_ASSERT_FALSE(esp_foc_ident_zprobe_init(NULL, &e));
    TEST_ASSERT_FALSE(esp_foc_ident_zprobe_init(&p, NULL));

    esp_foc_ident_excite_t bad = e;
    bad.periods = 0;
    TEST_ASSERT_FALSE(esp_foc_ident_zprobe_init(&p, &bad));

    bad = e;
    bad.fs_hz = 0;
    TEST_ASSERT_FALSE(esp_foc_ident_zprobe_init(&p, &bad));

    bad = e;
    bad.f_hz = 0;
    TEST_ASSERT_FALSE(esp_foc_ident_zprobe_init(&p, &bad));

    /* Above fs/8 there are too few samples per period to demodulate. */
    bad = e;
    bad.f_hz = 4000;
    TEST_ASSERT_FALSE(esp_foc_ident_zprobe_init(&p, &bad));

    bad = e;
    bad.lag_samples = ESP_FOC_IDENT_LAG_MAX;
    TEST_ASSERT_FALSE(esp_foc_ident_zprobe_init(&p, &bad));

    /* The chain on this bench sits about 3.5 samples back, so the range has to
     * reach past 4 for the sweep to find it at all. */
    bad = e;
    bad.lag_samples = ESP_FOC_IDENT_LAG_MAX - 1u;
    TEST_ASSERT_TRUE(esp_foc_ident_zprobe_init(&p, &bad));

    bad = e;
    bad.v_amp = 0;
    TEST_ASSERT_FALSE(esp_foc_ident_zprobe_init(&p, &bad));
}

TEST_CASE("ident zprobe recovers R and L at the well-conditioned frequency",
          "[espFoC][ident]")
{
    esp_foc_ident_z_t z;
    TEST_ASSERT_TRUE(probe_rl(REF_R, REF_L, 605, 20000, 0.8, 0.0, 0, &z));
    TEST_ASSERT_TRUE(z.valid);
    TEST_ASSERT_TRUE(pct_err(z.r_mohm, REF_R * 1000.0) <= 3);
    TEST_ASSERT_TRUE(pct_err(z.l_uh, REF_L * 1e6) <= 3);
    /* Probing at R/L is what puts arg(Z) at 45 degrees. */
    TEST_ASSERT_INT_WITHIN(300, 4500, z.phase_cdeg);
}

TEST_CASE("ident zprobe removes the half-sample ZOH phase", "[espFoC][ident]")
{
    /*
     * Without the correction the demodulated vector sits omega*dt/2 short of
     * the true argument: 5.4 degrees at 605 Hz on a 20 kHz carrier, which reads
     * R +9% and L -9% while |Z| stays inside 0.3%. Both frequencies landing
     * within 3% is what proves the rotation, since the error scales with f.
     */
    esp_foc_ident_z_t lo;
    esp_foc_ident_z_t hi;
    TEST_ASSERT_TRUE(probe_rl(REF_R, REF_L, 100, 20000, 0.8, 0.0, 0, &lo));
    TEST_ASSERT_TRUE(probe_rl(REF_R, REF_L, 605, 20000, 0.8, 0.0, 0, &hi));
    TEST_ASSERT_TRUE(pct_err(lo.l_uh, REF_L * 1e6) <= 3);
    TEST_ASSERT_TRUE(pct_err(hi.l_uh, REF_L * 1e6) <= 3);
    TEST_ASSERT_TRUE(pct_err(lo.r_mohm, REF_R * 1000.0) <= 3);
    TEST_ASSERT_TRUE(pct_err(hi.r_mohm, REF_R * 1000.0) <= 3);
}

TEST_CASE("ident coarse probe is far more sensitive to residual phase error",
          "[espFoC][ident]")
{
    /*
     * This is why identification re-probes near R/L instead of trusting a fixed
     * low frequency. L = |Z|*sin(phi), so dL/L = dphi*cot(phi): at 100 Hz on
     * this machine phi is 9.4 degrees and cot is 6, while at 605 Hz phi is 45
     * and cot is 1. White noise does not show this — a matched filter over 32
     * periods averages it away — but any uncompensated deadtime, filter or
     * timing skew is a systematic angle.
     */
    esp_foc_ident_z_t lo;
    esp_foc_ident_z_t hi;
    TEST_ASSERT_TRUE(probe_rl(REF_R, REF_L, 100, 20000, 0.8, 0.0, 100, &lo));
    TEST_ASSERT_TRUE(probe_rl(REF_R, REF_L, 605, 20000, 0.8, 0.0, 100, &hi));

    int lo_err = pct_err(lo.l_uh, REF_L * 1e6);
    int hi_err = pct_err(hi.l_uh, REF_L * 1e6);
    TEST_ASSERT_TRUE(lo_err >= 8);
    TEST_ASSERT_TRUE(hi_err <= 4);
    TEST_ASSERT_TRUE(lo_err > 2 * hi_err);
}

TEST_CASE("ident zprobe needs a bias to see through a deadtime dead zone",
          "[espFoC][ident]")
{
    /*
     * The bench that motivated the bias: a 0.7 V dead zone against a 0.84 V
     * probe. The current spent most of the cycle pinned, and the demodulator
     * read the fundamental of a clipped pulse train as R at twice its value with
     * a negative L. Reproduced here so the failure has a host regression.
     */
    esp_foc_ident_z_t blind;
    TEST_ASSERT_TRUE(probe_rl_bridge(REF_R, REF_L, 100, 20000, 0.84, 0.0, 0.0,
                                     0, 0.7, &blind));
    TEST_ASSERT_TRUE(blind.r_mohm > (int32_t)(1.8 * REF_R * 1000.0));

    /*
     * Bias past the dead zone by more than the AC amplitude drops across R and
     * the same probe is linear again: the deadtime error is then a constant, and
     * a whole-period correlation is orthogonal to a constant.
     */
    esp_foc_ident_z_t biased;
    TEST_ASSERT_TRUE(probe_rl_bridge(REF_R, REF_L, 100, 20000, 0.84,
                                     0.7 + 0.84, 0.0, 0, 0.7, &biased));
    TEST_ASSERT_TRUE(pct_err(biased.r_mohm, REF_R * 1000.0) <= 5);
    TEST_ASSERT_TRUE(pct_err(biased.l_uh, REF_L * 1e6) <= 10);
}

TEST_CASE("ident zprobe bias does not enter R or L", "[espFoC][ident]")
{
    /* Orthogonality of the DC term: an ideal bridge must read the same either
     * way, so a bias can be applied unconditionally. */
    esp_foc_ident_z_t plain;
    esp_foc_ident_z_t biased;
    TEST_ASSERT_TRUE(probe_rl_bridge(REF_R, REF_L, 605, 20000, 0.8, 0.0, 0.0, 0,
                                     0.0, &plain));
    TEST_ASSERT_TRUE(probe_rl_bridge(REF_R, REF_L, 605, 20000, 0.8, 2.5, 0.0, 0,
                                     0.0, &biased));
    TEST_ASSERT_INT_WITHIN(20, plain.r_mohm, biased.r_mohm);
    TEST_ASSERT_INT_WITHIN(8, plain.l_uh, biased.l_uh);
}

TEST_CASE("ident zprobe rejects white noise on the response", "[espFoC][ident]")
{
    esp_foc_ident_z_t clean;
    esp_foc_ident_z_t noisy;
    TEST_ASSERT_TRUE(probe_rl(REF_R, REF_L, 605, 20000, 0.8, 0.0, 0, &clean));
    TEST_ASSERT_TRUE(probe_rl(REF_R, REF_L, 605, 20000, 0.8, 0.05, 0, &noisy));
    TEST_ASSERT_INT_WITHIN(120, clean.r_mohm, noisy.r_mohm);
    TEST_ASSERT_INT_WITHIN(40, clean.l_uh, noisy.l_uh);
}

TEST_CASE("ident zprobe refuses an open circuit", "[espFoC][ident]")
{
    esp_foc_ident_excite_t e = {
        .v_amp = q16_from_float(0.8f),
        .f_hz = 605,
        .fs_hz = 20000,
        .periods = 32,
        .settle_periods = 2,
        .lag_samples = 1,
    };
    esp_foc_ident_zprobe_t p;
    TEST_ASSERT_TRUE(esp_foc_ident_zprobe_init(&p, &e));
    for (uint32_t k = 0; k < (p.n_skip + p.n_target) + 8u; k++) {
        (void)esp_foc_ident_zprobe_step(&p, 0);
    }
    esp_foc_ident_z_t z;
    TEST_ASSERT_FALSE(esp_foc_ident_zprobe_solve(&p, &z));
    TEST_ASSERT_FALSE(z.valid);
}

TEST_CASE("ident zprobe solve refuses an unfinished window", "[espFoC][ident]")
{
    esp_foc_ident_excite_t e = {
        .v_amp = q16_from_float(0.8f),
        .f_hz = 605,
        .fs_hz = 20000,
        .periods = 32,
        .settle_periods = 2,
        .lag_samples = 1,
    };
    esp_foc_ident_zprobe_t p;
    esp_foc_ident_z_t z;
    TEST_ASSERT_TRUE(esp_foc_ident_zprobe_init(&p, &e));
    (void)esp_foc_ident_zprobe_step(&p, q16_from_float(0.2f));
    TEST_ASSERT_FALSE(esp_foc_ident_zprobe_solve(&p, &z));
    TEST_ASSERT_FALSE(esp_foc_ident_zprobe_solve(NULL, &z));
    TEST_ASSERT_FALSE(esp_foc_ident_zprobe_solve(&p, NULL));
}

TEST_CASE("ident takes L from the magnitude when the angle is unusable",
          "[espFoC][ident]")
{
    TEST_ASSERT_EQUAL(0, esp_foc_ident_l_from_mag_uh(0, 1900, 605));
    TEST_ASSERT_EQUAL(0, esp_foc_ident_l_from_mag_uh(1900, 1900, 605));
    /* Below the crossing the reactance has not separated from R yet. */
    TEST_ASSERT_EQUAL(0, esp_foc_ident_l_from_mag_uh(1500, 1900, 605));
    TEST_ASSERT_EQUAL(0, esp_foc_ident_l_from_mag_uh(2700, 1900, 0));

    /* 1.9 ohm and 500 uH at 605 Hz: XL is 1.901 ohm, so |Z| is 2.688. */
    TEST_ASSERT_INT_WITHIN(15, 500, esp_foc_ident_l_from_mag_uh(2688, 1900, 605));

    /*
     * The whole point: a probe whose angle is destroyed still yields L, because
     * the magnitude is untouched by any rotation. Five samples of unaccounted
     * delay at 605 Hz on a 20 kHz carrier is 54 degrees, which takes the reported
     * 45 degrees negative — and yet L comes back.
     */
    esp_foc_ident_z_t z;
    TEST_ASSERT_TRUE(probe_rl(REF_R, REF_L, 605, 20000, 0.8, 0.0, -5445, &z));
    TEST_ASSERT_TRUE(z.phase_cdeg < 0);
    TEST_ASSERT_INT_WITHIN(40, 500,
                           esp_foc_ident_l_from_mag_uh(z.z_mohm, 1900, 605));
}

TEST_CASE("ident best probe frequency lands on 45 degrees", "[espFoC][ident]")
{
    TEST_ASSERT_EQUAL(0, esp_foc_ident_best_probe_hz(0, 500));
    TEST_ASSERT_EQUAL(0, esp_foc_ident_best_probe_hz(1900, 0));
    TEST_ASSERT_EQUAL(0, esp_foc_ident_best_probe_hz(-1, -1));

    /* Coarse pass output feeds the fine pass; check the loop actually closes. */
    esp_foc_ident_z_t coarse;
    TEST_ASSERT_TRUE(probe_rl(REF_R, REF_L, 100, 20000, 0.8, 0.0, 0, &coarse));
    int32_t f_fine = esp_foc_ident_best_probe_hz(coarse.r_mohm, coarse.l_uh);
    TEST_ASSERT_INT_WITHIN(60, 605, f_fine);

    esp_foc_ident_z_t fine;
    TEST_ASSERT_TRUE(probe_rl(REF_R, REF_L, (unsigned)f_fine, 20000, 0.8, 0.0,
                              0, &fine));
    TEST_ASSERT_INT_WITHIN(300, 4500, fine.phase_cdeg);
}

TEST_CASE("ident zprobe stays accurate across the machine range",
          "[espFoC][ident]")
{
    static const double r_set[] = {0.5, 1.23, 1.9, 4.0, 10.0};
    static const double l_set[] = {150e-6, 500e-6, 1.05e-3, 3.15e-3, 8.0e-3};

    for (unsigned ri = 0; ri < sizeof(r_set) / sizeof(r_set[0]); ri++) {
        for (unsigned li = 0; li < sizeof(l_set) / sizeof(l_set[0]); li++) {
            double r = r_set[ri];
            double l = l_set[li];
            int32_t f = esp_foc_ident_best_probe_hz((int32_t)(r * 1000.0),
                                                    (int32_t)(l * 1e6));
            if (f < 20 || f > 2400) {
                continue;
            }
            /* Hold the response near 0.4 A so the probe sees the same scale. */
            double v = 0.4 * r * 1.414;
            esp_foc_ident_z_t z;
            TEST_ASSERT_TRUE(probe_rl(r, l, (unsigned)f, 20000, v, 0.0, 0, &z));
            TEST_ASSERT_TRUE(pct_err(z.r_mohm, r * 1000.0) <= 5);
            TEST_ASSERT_TRUE(pct_err(z.l_uh, l * 1e6) <= 5);
        }
    }
}

TEST_CASE("ident dc probe averages after the settle window", "[espFoC][ident]")
{
    esp_foc_ident_dcprobe_t p;
    esp_foc_ident_dcprobe_init(&p, 4, 8);
    TEST_ASSERT_FALSE(esp_foc_ident_dcprobe_done(&p));
    TEST_ASSERT_EQUAL_INT32(0, esp_foc_ident_dcprobe_ma(&p));

    /* Transient inside the skip window must not reach the mean. */
    for (int k = 0; k < 4; k++) {
        esp_foc_ident_dcprobe_add(&p, q16_from_float(5.0f));
    }
    for (int k = 0; k < 8; k++) {
        esp_foc_ident_dcprobe_add(&p, q16_from_float(0.600f));
    }
    TEST_ASSERT_TRUE(esp_foc_ident_dcprobe_done(&p));
    TEST_ASSERT_INT32_WITHIN(2, 600, esp_foc_ident_dcprobe_ma(&p));

    /* Samples past the window are ignored. */
    esp_foc_ident_dcprobe_add(&p, q16_from_float(5.0f));
    TEST_ASSERT_INT32_WITHIN(2, 600, esp_foc_ident_dcprobe_ma(&p));

    esp_foc_ident_dcprobe_init(&p, 0, 0);
    esp_foc_ident_dcprobe_add(&p, q16_from_float(1.0f));
    TEST_ASSERT_TRUE(esp_foc_ident_dcprobe_done(&p));
    TEST_ASSERT_INT32_WITHIN(2, 1000, esp_foc_ident_dcprobe_ma(&p));
}

TEST_CASE("ident two-point slope cancels the bridge offset", "[espFoC][ident]")
{
    /*
     * Bench numbers: 0.75 V drew 0.609 A, so a single point reads 1.23 ohm on a
     * 0.75 ohm winding because deadtime and Vds eat a fixed ~0.29 V. The slope
     * through a second point recovers the winding.
     */
    TEST_ASSERT_INT32_WITHIN(30, 750,
                             esp_foc_ident_r_slope_mohm(750, 609, 1500, 1609));
    TEST_ASSERT_EQUAL_INT32(0, esp_foc_ident_r_slope_mohm(750, 609, 1500, 609));
}

TEST_CASE("ident symmetry refuses an open delta winding", "[espFoC][ident]")
{
    esp_foc_ident_sym_t s;
    int32_t healthy[3] = {300, 305, 298};
    esp_foc_ident_symmetry(healthy, 100, &s);
    TEST_ASSERT_TRUE(s.ok);
    TEST_ASSERT_TRUE(s.spread_permil < 100);

    /* The damaged bench motor: driving one terminal pulled twice the current. */
    int32_t open_delta[3] = {150, 150, 300};
    esp_foc_ident_symmetry(open_delta, 100, &s);
    TEST_ASSERT_FALSE(s.ok);
    TEST_ASSERT_EQUAL_INT32(500, s.spread_permil);
    TEST_ASSERT_EQUAL_INT(2, s.strong_idx);

    /* Sign is irrelevant: the low-side shunt polarity flips per terminal. */
    int32_t signed_ok[3] = {-300, 305, -298};
    esp_foc_ident_symmetry(signed_ok, 100, &s);
    TEST_ASSERT_TRUE(s.ok);

    int32_t dead[3] = {0, 0, 0};
    esp_foc_ident_symmetry(dead, 100, &s);
    TEST_ASSERT_FALSE(s.ok);

    esp_foc_ident_symmetry(NULL, 100, &s);
    TEST_ASSERT_FALSE(s.ok);
}

TEST_CASE("ident flux solves the q-axis balance", "[espFoC][ident]")
{
    /*
     * Cross-check of the bench ceiling: the velocity loop saturated at 565 Hz
     * with 0.45 A on a 12 V bus, so psi is about 1.7 mWb — well under the
     * 2.56 mWb the nameplate Kt implies.
     */
    TEST_ASSERT_INT32_WITHIN(60, 1711,
                             esp_foc_ident_psi_uwb(6930, 450, 0, 1900, 500, 3550));

    /* With no IR drop it collapses to vq/omega. */
    TEST_ASSERT_INT32_WITHIN(20, 1952,
                             esp_foc_ident_psi_uwb(6930, 0, 0, 1900, 500, 3550));

    /* The omega*L*id term matters once the d axis is loaded. */
    int32_t with_id = esp_foc_ident_psi_uwb(6930, 450, 500, 1900, 500, 3550);
    TEST_ASSERT_TRUE(with_id < 1711);

    TEST_ASSERT_EQUAL_INT32(0, esp_foc_ident_psi_uwb(6930, 0, 0, 1900, 500, 0));
    TEST_ASSERT_EQUAL_INT32(0, esp_foc_ident_psi_uwb(6930, 0, 0, 1900, 500, -1));
}

/**
 * Record a step window the way the ISR does: read the current, then write the
 * command, with @p lag samples of transport between the two.
 */
static void step_window(double r_ohm, double l_h, unsigned fs_hz, double v,
                        unsigned lag, unsigned n_pre, double noise_a,
                        q16_t *out, unsigned n)
{
    rl_t plant;
    rl_init(&plant, r_ohm, l_h, fs_hz);
    double hist[ESP_FOC_IDENT_STEP_MAX + 8] = {0};

    for (unsigned k = 0; k < n; k++) {
        hist[k] = plant.i;
        /*
         * lag == 1 is the delay-free case in this convention, the same one the
         * impedance probe uses: the sample handed over now belongs to the command
         * about to be written. Each further sample of transport moves the read
         * one period further back.
         */
        const double meas = (k + 1u >= lag) ? hist[k + 1u - lag] : 0.0;
        out[k] = q16_from_float((float)(meas + noise_a * frand_sym()));
        rl_apply(&plant, (k >= n_pre) ? v : 0.0);
    }
}

TEST_CASE("ident step lag times the delay it is given", "[espFoC][ident]")
{
    q16_t w[ESP_FOC_IDENT_STEP_MAX];
    const q16_t thresh = q16_from_float(0.05f);

    for (unsigned lag = 1; lag <= 6; lag++) {
        step_window(REF_R, REF_L, 20000, 6.0, lag, 4, 0.0, w,
                    ESP_FOC_IDENT_STEP_MAX);
        TEST_ASSERT_EQUAL_INT32((int32_t)lag,
                                esp_foc_ident_step_lag(w, ESP_FOC_IDENT_STEP_MAX,
                                                       4, thresh));
    }
}

TEST_CASE("ident step lag holds under sense noise", "[espFoC][ident]")
{
    q16_t w[ESP_FOC_IDENT_STEP_MAX];
    /*
     * This bench idles at 700 mA of peak on a 3 A scale, and the step is sized so
     * one time constant clears that: threshold at 300 mA against a 2.4 A final.
     */
    s_rng = 12345u;
    for (int trial = 0; trial < 20; trial++) {
        step_window(REF_R, REF_L, 20000, 6.0, 3, 6, 0.10, w,
                    ESP_FOC_IDENT_STEP_MAX);
        TEST_ASSERT_EQUAL_INT32(3,
                                esp_foc_ident_step_lag(w, ESP_FOC_IDENT_STEP_MAX,
                                                       6, q16_from_float(0.3f)));
    }
}

TEST_CASE("ident step lag refuses a winding that never answers",
          "[espFoC][ident]")
{
    q16_t w[ESP_FOC_IDENT_STEP_MAX];

    /* Open circuit: nothing crosses the threshold. */
    memset(w, 0, sizeof(w));
    TEST_ASSERT_EQUAL_INT32(-1, esp_foc_ident_step_lag(w, ESP_FOC_IDENT_STEP_MAX,
                                                       4, q16_from_float(0.3f)));

    /* A machine so slow the window ends before the threshold is reached is the
     * same refusal — the answer would be the window length, not a delay. */
    step_window(REF_R, 40e-3, 20000, 6.0, 2, 4, 0.0, w, ESP_FOC_IDENT_STEP_MAX);
    TEST_ASSERT_EQUAL_INT32(-1, esp_foc_ident_step_lag(w, ESP_FOC_IDENT_STEP_MAX,
                                                       4, q16_from_float(0.3f)));

    TEST_ASSERT_EQUAL_INT32(-1, esp_foc_ident_step_lag(NULL, 24, 4, 100));
    TEST_ASSERT_EQUAL_INT32(-1, esp_foc_ident_step_lag(w, 24, 0, 100));
    TEST_ASSERT_EQUAL_INT32(-1, esp_foc_ident_step_lag(w, 4, 4, 100));
    TEST_ASSERT_EQUAL_INT32(-1, esp_foc_ident_step_lag(w, 24, 4, 0));
}

TEST_CASE("ident step lag ignores a standing offset", "[espFoC][ident]")
{
    q16_t w[ESP_FOC_IDENT_STEP_MAX];
    step_window(REF_R, REF_L, 20000, 6.0, 2, 4, 0.0, w,
                ESP_FOC_IDENT_STEP_MAX);
    /* A shunt zero that is off by 800 mA must not read as an early answer. */
    for (unsigned k = 0; k < ESP_FOC_IDENT_STEP_MAX; k++) {
        w[k] = q16_add(w[k], q16_from_float(0.8f));
    }
    TEST_ASSERT_EQUAL_INT32(2, esp_foc_ident_step_lag(w, ESP_FOC_IDENT_STEP_MAX,
                                                      4, q16_from_float(0.3f)));
}
