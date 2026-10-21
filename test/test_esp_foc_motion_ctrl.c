/*
 * Unit tests for esp_foc_motion_ctrl: cascade PI, LQR and LQR+I against a
 * double-precision servo plant (double integrator, current-loop lag, one
 * sample from the command to the current loop).
 */
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "esp_err.h"
#include "espFoC/motor_control/esp_foc_motion_ctrl.h"
#include "espFoC/motor_control/esp_foc_traj.h"

#define MC_K      67347.0
#define MC_PP     13
#define MC_FS     2500.0
#define MC_TS     (1.0 / MC_FS)
#define MC_IBW    1000.0
#define MC_TAU    (1.0 / (2.0 * M_PI * MC_IBW))
#define MC_IMAX   0.7
#define MC_WN_HZ  18.0f
#define MC_SUB    20

typedef struct {
    double th;      /* mechanical rad */
    double w;       /* electrical rad/s */
    double i;
    double u_prev;
} mc_plant_t;

static void plant_step(mc_plant_t *p, double u, double i_load)
{
    const double h = MC_TS / MC_SUB;
    for (int n = 0; n < MC_SUB; n++) {
        const double di = (p->u_prev - p->i) / MC_TAU;
        const double dw = MC_K * (p->i - i_load);
        p->th += h * (p->w + 0.5 * h * dw) / MC_PP;
        p->w += h * dw;
        p->i += h * di;
    }
    p->u_prev = u;
}

static void design_defaults(esp_foc_motion_ctrl_design_t *d)
{
    memset(d, 0, sizeof(*d));
    d->k_rads2_a = (float)MC_K;
    d->pole_pairs = MC_PP;
    d->fs_hz = (float)MC_FS;
    d->i_max_a = (float)MC_IMAX;
    d->position = true;
    d->bw_hz = 7.8f;
    d->zeta = 1.15f;
    d->pos_sep = 4.0f;
    d->corr_max_rads = 2.0f * (float)M_PI * 0.8f;
    d->wm_max_rads = 2.0f * (float)M_PI * 18.0f;
    d->wn_hz = MC_WN_HZ;
    d->i_bw_hz = (float)MC_IBW;
    d->fb_iq_budget_a = 0.1f;
}

static q16_t q16d(double x)
{
    return (q16_t)llround(x * 65536.0);
}

static double run_pos(esp_foc_motion_ctrl_t *c, mc_plant_t *p, double ref, double i_load,
                      int n, double *peak_over, int *crossings)
{
    double prev = ref - p->th;
    double over = 0.0;
    int cross = 0;
    const double s = (ref >= p->th) ? 1.0 : -1.0;
    for (int k = 0; k < n; k++) {
        const esp_foc_motion_in_t in = {
            .e_th = (int64_t)llround((ref - p->th) * 65536.0),
            .w_ff_m = 0,
            .w_ref_e = 0,
            .w = q16d(p->w),
            .iq_ff = 0,
        };
        q16_t w_ref;
        const q16_t u = esp_foc_motion_ctrl_update(c, ESP_FOC_MOTION_POSITION, &in, &w_ref);
        TEST_ASSERT_TRUE(fabs(u / 65536.0) <= MC_IMAX + 1e-4);
        plant_step(p, u / 65536.0, i_load);
        const double e = ref - p->th;
        over = fmax(over, -s * e);
        if ((k > 0) && (e * prev < 0.0)) {
            cross++;
        }
        prev = e;
    }
    if (peak_over != NULL) {
        *peak_over = over;
    }
    if (crossings != NULL) {
        *crossings = cross;
    }
    return ref - p->th;
}

TEST_CASE("motion lqr designs both laws at the target wn", "[espFoC][motion]")
{
    esp_foc_motion_lqr_t obj;
    esp_foc_motion_ctrl_t *c = esp_foc_motion_lqr_bind(&obj, true);
    esp_foc_motion_ctrl_design_t d;
    design_defaults(&d);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motion_ctrl_design(c, &d));
    TEST_ASSERT_EQUAL_FLOAT(MC_WN_HZ, c->tune.wn_hz);
    TEST_ASSERT_EQUAL_UINT8(5, c->tune.n_pos);
    TEST_ASSERT_EQUAL_UINT8(4, c->tune.n_vel);
    TEST_ASSERT_TRUE(c->tune.k_pos[0] > 0.0f);
    TEST_ASSERT_TRUE(c->tune.k_pos[1] > 0.0f);
    TEST_ASSERT_TRUE(c->tune.k_vel[0] > 0.0f);
    TEST_ASSERT_TRUE(obj.g[ESP_FOC_MOTION_POSITION].kz != 0);
    TEST_ASSERT_TRUE(obj.g[ESP_FOC_MOTION_POSITION].z_lim > 0);
}

TEST_CASE("motion lqr lowers wn until the speed gain fits the noise budget", "[espFoC][motion]")
{
    esp_foc_motion_lqr_t obj;
    esp_foc_motion_ctrl_t *c = esp_foc_motion_lqr_bind(&obj, false);
    esp_foc_motion_ctrl_design_t d;
    design_defaults(&d);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motion_ctrl_design(c, &d));
    const float kw0 = c->tune.k_pos[1];
    d.ripple_rads = 4.0f * d.fb_iq_budget_a / kw0;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motion_ctrl_design(c, &d));
    TEST_ASSERT_TRUE(c->tune.wn_hz < MC_WN_HZ);
    TEST_ASSERT_TRUE(fabsf(c->tune.k_pos[1]) * d.ripple_rads <= d.fb_iq_budget_a * 1.0001f);
    TEST_ASSERT_TRUE(fabsf(c->tune.k_vel[0]) * d.ripple_rads <= d.fb_iq_budget_a * 1.0001f);

    /* A physical speed knob is taken as given. */
    d.w_max_rads = 2000.0f;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motion_ctrl_design(c, &d));
    TEST_ASSERT_EQUAL_FLOAT(MC_WN_HZ, c->tune.wn_hz);
}

TEST_CASE("motion lqr position step settles without ringing", "[espFoC][motion]")
{
    esp_foc_motion_lqr_t obj;
    esp_foc_motion_ctrl_t *c = esp_foc_motion_lqr_bind(&obj, false);
    esp_foc_motion_ctrl_design_t d;
    design_defaults(&d);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motion_ctrl_design(c, &d));
    esp_foc_motion_ctrl_reset(c, ESP_FOC_MOTION_POSITION, 0);

    mc_plant_t p = { 0 };
    double over;
    int cross;
    const double ref = 0.01;
    const double e = run_pos(c, &p, ref, 0.0, 2500, &over, &cross);
    TEST_ASSERT_DOUBLE_WITHIN(2e-5, 0.0, e);
    TEST_ASSERT_TRUE(over < 0.15 * ref);
    TEST_ASSERT_TRUE(cross <= 2);
    TEST_ASSERT_DOUBLE_WITHIN(1.0, 0.0, p.w);
}

TEST_CASE("motion lqr+i removes a constant load the plain lqr leaves", "[espFoC][motion]")
{
    esp_foc_motion_ctrl_design_t d;
    design_defaults(&d);
    const double load = 0.1;

    esp_foc_motion_lqr_t lq;
    esp_foc_motion_ctrl_t *c = esp_foc_motion_lqr_bind(&lq, false);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motion_ctrl_design(c, &d));
    esp_foc_motion_ctrl_reset(c, ESP_FOC_MOTION_POSITION, 0);
    mc_plant_t p = { 0 };
    const double e_lqr = run_pos(c, &p, 0.0, load, 5000, NULL, NULL);
    /* At rest under load the lag and delay states hold the load too. */
    const float *k = c->tune.k_pos;
    const double e_ss = load * (1.0 + k[2] + k[3]) / k[0];
    TEST_ASSERT_DOUBLE_WITHIN(0.01 * e_ss, e_ss, e_lqr);

    esp_foc_motion_lqr_t lqi;
    c = esp_foc_motion_lqr_bind(&lqi, true);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motion_ctrl_design(c, &d));
    esp_foc_motion_ctrl_reset(c, ESP_FOC_MOTION_POSITION, 0);
    memset(&p, 0, sizeof(p));
    int cross;
    const double e_lqi = run_pos(c, &p, 0.0, load, 5000, NULL, &cross);
    TEST_ASSERT_DOUBLE_WITHIN(2e-5, 0.0, e_lqi);
    TEST_ASSERT_TRUE(fabs(e_lqr) > 20.0 * fabs(e_lqi));
    TEST_ASSERT_TRUE(cross <= 3);

    /* Final value: the tail no longer moves. */
    double tail_min = 1e9;
    double tail_max = -1e9;
    for (int k = 0; k < 500; k++) {
        const double e = run_pos(c, &p, 0.0, load, 1, NULL, NULL);
        tail_min = fmin(tail_min, e);
        tail_max = fmax(tail_max, e);
    }
    TEST_ASSERT_TRUE(tail_max - tail_min < 2e-5);
}

TEST_CASE("motion lqr+i saturated step does not wind up", "[espFoC][motion]")
{
    esp_foc_motion_lqr_t obj;
    esp_foc_motion_ctrl_t *c = esp_foc_motion_lqr_bind(&obj, true);
    esp_foc_motion_ctrl_design_t d;
    design_defaults(&d);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motion_ctrl_design(c, &d));
    esp_foc_motion_ctrl_reset(c, ESP_FOC_MOTION_POSITION, 0);

    mc_plant_t p = { 0 };
    double over;
    int cross;
    const double ref = 3.0;
    const double e = run_pos(c, &p, ref, 0.0, 7500, &over, &cross);
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, 0.0, e);
    TEST_ASSERT_TRUE(over < 0.1 * ref);
    TEST_ASSERT_TRUE(cross <= 4);
    const esp_foc_motion_lqr_gains_t *g = &obj.g[ESP_FOC_MOTION_POSITION];
    TEST_ASSERT_TRUE(obj.z <= g->z_lim);
    TEST_ASSERT_TRUE(obj.z >= -g->z_lim);
}

TEST_CASE("motion lqr velocity step reaches the reference under load", "[espFoC][motion]")
{
    /* The speed law integrates whether or not the position law does. */
    for (int integ = 0; integ < 2; integ++) {
        esp_foc_motion_lqr_t obj;
        esp_foc_motion_ctrl_t *c = esp_foc_motion_lqr_bind(&obj, integ != 0);
        esp_foc_motion_ctrl_design_t d;
        design_defaults(&d);
        d.position = false;
        TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motion_ctrl_design(c, &d));
        TEST_ASSERT_EQUAL_UINT8(0, c->tune.n_pos);
        TEST_ASSERT_EQUAL_UINT8(4, c->tune.n_vel);
        esp_foc_motion_ctrl_reset(c, ESP_FOC_MOTION_VELOCITY, 0);

        mc_plant_t p = { 0 };
        const double w_ref = 300.0;
        const double load = 0.05;
        double w_max = 0.0;
        for (int k = 0; k < 5000; k++) {
            const esp_foc_motion_in_t in = { .w_ref_e = q16d(w_ref), .w = q16d(p.w) };
            q16_t wr;
            const q16_t u = esp_foc_motion_ctrl_update(c, ESP_FOC_MOTION_VELOCITY, &in, &wr);
            TEST_ASSERT_EQUAL_INT32(q16d(w_ref), wr);
            plant_step(&p, u / 65536.0, load);
            w_max = fmax(w_max, p.w);
        }
        TEST_ASSERT_DOUBLE_WITHIN(0.5, w_ref, p.w);
        TEST_ASSERT_TRUE(w_max < 1.2 * w_ref);
    }
}

TEST_CASE("motion lqr+i reset is bumpless", "[espFoC][motion]")
{
    esp_foc_motion_lqr_t obj;
    esp_foc_motion_ctrl_t *c = esp_foc_motion_lqr_bind(&obj, true);
    esp_foc_motion_ctrl_design_t d;
    design_defaults(&d);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motion_ctrl_design(c, &d));

    const q16_t held = q16d(0.23);
    const q16_t ff = q16d(-0.05);
    for (int m = 0; m < 2; m++) {
        esp_foc_motion_ctrl_reset(c, (esp_foc_motion_mode_t)m, held);
        /* On the reference: ω* = 3 rad/s mechanical, both ways of saying it. */
        const esp_foc_motion_in_t in = {
            .w_ff_m = q16d(3.0), .w_ref_e = q16d(3.0 * MC_PP), .w = q16d(3.0 * MC_PP), .iq_ff = ff
        };
        q16_t wr;
        const q16_t u = esp_foc_motion_ctrl_update(c, (esp_foc_motion_mode_t)m, &in, &wr);
        TEST_ASSERT_INT32_WITHIN(4, held + ff, u);
    }
}

TEST_CASE("motion pi matches the cascade and is bumpless", "[espFoC][motion]")
{
    esp_foc_motion_pi_t obj;
    esp_foc_motion_ctrl_t *c = esp_foc_motion_pi_bind(&obj);
    esp_foc_motion_ctrl_design_t d;
    design_defaults(&d);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motion_ctrl_design(c, &d));
    TEST_ASSERT_TRUE(c->tune.kp_w > 0.0f);
    TEST_ASSERT_TRUE(c->tune.ki_w > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.0f * (float)M_PI * 2.0f * 1.15f * 7.8f / 4.0f,
                             c->tune.kp_pos);

    const q16_t held = q16d(0.2);
    esp_foc_motion_ctrl_reset(c, ESP_FOC_MOTION_VELOCITY, held);
    esp_foc_motion_in_t in = { .w_ref_e = q16d(10.0), .w = q16d(10.0), .iq_ff = q16d(0.1) };
    q16_t wr;
    TEST_ASSERT_EQUAL_INT32(held + q16d(0.1),
                            esp_foc_motion_ctrl_update(c, ESP_FOC_MOTION_VELOCITY, &in, &wr));

    /* Position: ω* = pp·clamp(w_ff + clamp(kp·e, ±corr), ±wm). */
    esp_foc_motion_ctrl_reset(c, ESP_FOC_MOTION_POSITION, 0);
    memset(&in, 0, sizeof(in));
    in.e_th = q16d(0.01);
    (void)esp_foc_motion_ctrl_update(c, ESP_FOC_MOTION_POSITION, &in, &wr);
    TEST_ASSERT_EQUAL_INT32((q16_t)(q16_mul(obj.kp_pos, q16d(0.01)) * MC_PP), wr);
    in.e_th = q16d(100.0);
    (void)esp_foc_motion_ctrl_update(c, ESP_FOC_MOTION_POSITION, &in, &wr);
    TEST_ASSERT_EQUAL_INT32(obj.corr_max * MC_PP, wr);
}

TEST_CASE("motion pi position step reaches the target", "[espFoC][motion]")
{
    esp_foc_motion_pi_t obj;
    esp_foc_motion_ctrl_t *c = esp_foc_motion_pi_bind(&obj);
    esp_foc_motion_ctrl_design_t d;
    design_defaults(&d);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motion_ctrl_design(c, &d));
    esp_foc_motion_ctrl_reset(c, ESP_FOC_MOTION_POSITION, 0);
    mc_plant_t p = { 0 };
    const double e = run_pos(c, &p, 0.05, 0.0, 10000, NULL, NULL);
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, 0.0, e);
}

/* Large move: the LQR tracks a trapezoid with the acceleration fed forward
 * and lands where the PI cascade, capped by corr_max, is still far away. */
TEST_CASE("motion lqr on a trajectory beats the pi on a large step", "[espFoC][motion]")
{
    esp_foc_motion_ctrl_design_t d;
    design_defaults(&d);
    const double target = 6.0;
    const double km = MC_K / MC_PP;
    const float a_max = (float)(0.5 * km * MC_IMAX);
    const float v_max = 2.0f * (float)M_PI * 18.0f;

    esp_foc_motion_lqr_t lq;
    esp_foc_motion_ctrl_t *c = esp_foc_motion_lqr_bind(&lq, true);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motion_ctrl_design(c, &d));
    esp_foc_motion_ctrl_reset(c, ESP_FOC_MOTION_POSITION, 0);
    esp_foc_traj_t t;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_traj_init(&t, (uint32_t)MC_FS, a_max, v_max));
    esp_foc_traj_seed(&t, 0, 0);
    esp_foc_traj_set_target(&t, (int64_t)llround(target * 65536.0));

    mc_plant_t p = { 0 };
    double e_peak = 0.0;
    int k_lqr = -1;
    for (int k = 0; k < 10000; k++) {
        esp_foc_traj_step(&t);
        const double a_ref = (double)t.acc / 4294967296.0 * MC_FS * MC_FS;
        const esp_foc_motion_in_t in = {
            .e_th = esp_foc_traj_theta_q16(&t) - (int64_t)llround(p.th * 65536.0),
            .w_ff_m = esp_foc_traj_w(&t),
            .w = q16d(p.w),
            .iq_ff = q16d(a_ref / km),
        };
        q16_t wr;
        const q16_t u = esp_foc_motion_ctrl_update(c, ESP_FOC_MOTION_POSITION, &in, &wr);
        e_peak = fmax(e_peak, fabs((double)in.e_th / 65536.0));
        plant_step(&p, u / 65536.0, 0.0);
        if ((k_lqr < 0) && esp_foc_traj_done(&t) && (fabs(target - p.th) < 1e-3)) {
            k_lqr = k;
        }
    }
    TEST_ASSERT_TRUE(k_lqr > 0);
    TEST_ASSERT_TRUE(e_peak < 0.02);
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, target, p.th);

    esp_foc_motion_pi_t pi;
    c = esp_foc_motion_pi_bind(&pi);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motion_ctrl_design(c, &d));
    esp_foc_motion_ctrl_reset(c, ESP_FOC_MOTION_POSITION, 0);
    memset(&p, 0, sizeof(p));
    (void)run_pos(c, &p, target, 0.0, k_lqr, NULL, NULL);
    TEST_ASSERT_TRUE(fabs(target - p.th) > 0.5);
}

TEST_CASE("motion design refuses bad plants", "[espFoC][motion]")
{
    esp_foc_motion_lqr_t lq;
    esp_foc_motion_pi_t pi;
    esp_foc_motion_ctrl_t *c = esp_foc_motion_lqr_bind(&lq, true);
    esp_foc_motion_ctrl_design_t d;
    design_defaults(&d);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_motion_ctrl_design(c, NULL));
    d.wn_hz = 0.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_motion_ctrl_design(c, &d));
    design_defaults(&d);
    d.i_bw_hz = 0.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_motion_ctrl_design(c, &d));
    design_defaults(&d);
    d.ripple_rads = 1.0f;
    d.fb_iq_budget_a = 0.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_motion_ctrl_design(c, &d));

    c = esp_foc_motion_pi_bind(&pi);
    design_defaults(&d);
    d.pole_pairs = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_motion_ctrl_design(c, &d));
    TEST_ASSERT_NULL(esp_foc_motion_pi_bind(NULL));
    TEST_ASSERT_NULL(esp_foc_motion_lqr_bind(NULL, false));
}
