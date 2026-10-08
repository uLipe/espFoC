/*
 * Unit tests for the sensorless stack against mock_pmsm_inverter: a 7 pp
 * PMSM (2 ohm, 1 mH, 5 mWb) with Coulomb + viscous friction worth about
 * 0.15 A + 1 mA/Hz of iq, at a 5 kHz virtual PWM. The derivation tests use
 * the bench motor's numbers at 20 kHz without ticking.
 */
#include "sdkconfig.h"

#if CONFIG_ESP_FOC_STACK_SENSORLESS

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"
#include "esp_err.h"
#include "espFoC/motor_control/esp_foc_sensorless.h"
#include "espFoC/osal/esp_foc_osal.h"
#include "espFoC/utils/esp_foc_pid.h"
#include "mock_pmsm_inverter.h"

#define TWO_PI_F 6.28318530718f
#define EV_COUNT (ESP_FOC_SL_EV_FAULT_CLEARED + 1)

static mock_pmsm_t s_m;
static volatile int s_ev_n[EV_COUNT];
static esp_foc_sensorless_event_t s_ev_last[EV_COUNT];
static volatile int s_ev_ax[CONFIG_ESP_FOC_SL_MAX_AXES][EV_COUNT];
static esp_foc_sensorless_event_t s_ev_ax_last[CONFIG_ESP_FOC_SL_MAX_AXES][EV_COUNT];

static const mock_pmsm_params_t k_plant = {
    .vdc = 12.0f,
    .rs = 2.0f,
    .ls = 1.0e-3f,
    .psi = 0.005f,
    .pp = 7,
    .j = 1.0e-5f,
    .b_visc = 5.85e-5f,
    .t_coul = 7.875e-3f,
    .pwm_hz = 5000u,
    .theta0 = 0.4f,
};

/* Bench motor: 13 pp, Kt 0.05, 498 Hz rated, 20 kHz. Not ticked. */
static const mock_pmsm_params_t k_bench = {
    .vdc = 12.0f,
    .rs = 1.9f,
    .ls = 148.0e-6f,
    .psi = 0.05f / (1.5f * 13.0f),
    .pp = 13,
    .j = 1.0e-5f,
    .b_visc = 1.0e-5f,
    .t_coul = 1.0e-3f,
    .pwm_hz = 20000u,
    .theta0 = 0.0f,
};

static void on_ev(void *ctx, const esp_foc_sensorless_event_t *e)
{
    (void)ctx;
    if ((int)e->ev < EV_COUNT) {
        s_ev_last[e->ev] = *e;
        s_ev_n[e->ev]++;
        if (e->axis < CONFIG_ESP_FOC_SL_MAX_AXES) {
            s_ev_ax_last[e->axis][e->ev] = *e;
            s_ev_ax[e->axis][e->ev]++;
        }
    }
}

static void ev_reset(void)
{
    memset((void *)s_ev_n, 0, sizeof(s_ev_n));
    memset(s_ev_last, 0, sizeof(s_ev_last));
    memset((void *)s_ev_ax, 0, sizeof(s_ev_ax));
    memset(s_ev_ax_last, 0, sizeof(s_ev_ax_last));
}

static bool wait_ev(esp_foc_sensorless_ev_t ev, int n, uint32_t timeout_ms)
{
    for (uint32_t t = 0; t < timeout_ms; t += 5u) {
        if (s_ev_n[ev] >= n) {
            return true;
        }
        esp_foc_sleep_ms(5);
    }
    return s_ev_n[ev] >= n;
}

static bool wait_state(esp_foc_sensorless_state_t st, uint32_t timeout_ms)
{
    for (uint32_t t = 0; t < timeout_ms; t += 5u) {
        if (esp_foc_sensorless_get_state(0) == st) {
            return true;
        }
        esp_foc_sleep_ms(5);
    }
    return esp_foc_sensorless_get_state(0) == st;
}

typedef float (*st_get_t)(const esp_foc_sensorless_status_t *s);

typedef struct {
    float mean;
    float min;
    float max;
} win_t;

static float g_iq(const esp_foc_sensorless_status_t *s)
{
    return s->iq_a;
}

static float g_id(const esp_foc_sensorless_status_t *s)
{
    return s->id_a;
}

static float g_iq_ref(const esp_foc_sensorless_status_t *s)
{
    return s->iq_ref_a;
}

static float g_w_ctrl(const esp_foc_sensorless_status_t *s)
{
    return s->w_ctrl_rads;
}

static float g_vmag(const esp_foc_sensorless_status_t *s)
{
    return sqrtf(s->vd_v * s->vd_v + s->vq_v * s->vq_v);
}

static win_t sample(st_get_t get, uint32_t ms)
{
    win_t w = {0.0f, 1.0e9f, -1.0e9f};
    uint32_t n = 0;
    for (uint32_t t = 0; t < ms; t += 2u) {
        esp_foc_sensorless_status_t st;
        esp_foc_sensorless_get_status(0, &st);
        const float x = get(&st);
        w.mean += x;
        w.min = fminf(w.min, x);
        w.max = fmaxf(w.max, x);
        n++;
        esp_foc_sleep_ms(2);
    }
    w.mean /= (float)((n > 0u) ? n : 1u);
    return w;
}

static void cfg_mock(esp_foc_sensorless_config_t *c, bool speed)
{
    esp_foc_sensorless_default_config(c);
    c->rs_ohm = k_plant.rs;
    c->ls_h = k_plant.ls;
    c->psi_wb = k_plant.psi;
    c->pole_pairs = (uint8_t)k_plant.pp;
    c->fe_rated_hz = 200.0f;
    c->speed_loop = speed;
    /* 5 kHz: the bench 1 kHz loop would sit at the one-sample-delay edge. */
    c->i_bw_hz = 300.0f;
    c->observer.obs_bw_hz = 400.0f;
    c->startup.cal_rounds = 0;
    c->startup.align_ms = 200u;
    c->startup.align_settle_ms = 150u;
    c->startup.accel_rads2 = 400.0f;
    c->catch_up.hold_ms = 300u;
    c->catch_up.slew_a_s = 2.0f;
    c->wref_slew_hz_s = 100.0f;
    c->rev_decel_hz_s = 100.0f;
    c->rev_decel_a_s = 2.0f;
    c->coast_ms = 500u;
    c->on_event = on_ev;
}

static void cfg_bench(esp_foc_sensorless_config_t *c, bool speed)
{
    esp_foc_sensorless_default_config(c);
    c->rs_ohm = k_bench.rs;
    c->ls_h = k_bench.ls;
    c->psi_wb = k_bench.psi;
    c->pole_pairs = (uint8_t)k_bench.pp;
    c->fe_rated_hz = 498.0f;
    c->speed_loop = speed;
    c->on_event = on_ev;
}

/* A failed assertion longjmps out with the stack still up; every test
 * starts from a clean bench whatever the previous one left. */
static void bench_reset(void)
{
    for (uint8_t a = 0; a < CONFIG_ESP_FOC_SL_MAX_AXES; a++) {
        esp_foc_sensorless_deinit(a);
    }
    mock_pmsm_stop();
    ev_reset();
}

static void bench_up(bool speed)
{
    esp_foc_sensorless_config_t c;
    bench_reset();
    mock_pmsm_init(&s_m, &k_plant);
    cfg_mock(&c, speed);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_init(&s_m.base, &c));
    mock_pmsm_start(&s_m);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_run(0));
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SL_EV_ARMED, 1, 200));
}

static void bench_down(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_stop(0));
    TEST_ASSERT_EQUAL(ESP_FOC_SL_STATE_IDLE, esp_foc_sensorless_get_state(0));
    TEST_ASSERT_FALSE(s_m.enabled);
    bench_reset();
}

static float wrap_pi(float x)
{
    while (x > (float)M_PI) {
        x -= TWO_PI_F;
    }
    while (x <= -(float)M_PI) {
        x += TWO_PI_F;
    }
    return x;
}

TEST_CASE("sensorless derives the bench loop stack from fe_rated", "[espFoC][sensorless]")
{
    esp_foc_sensorless_config_t c;
    esp_foc_sensorless_tuning_t t;
    esp_foc_sensorless_status_t st;

    bench_reset();
    mock_pmsm_init(&s_m, &k_bench);
    cfg_bench(&c, true);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_init(&s_m.base, &c));
    esp_foc_sensorless_get_tuning(0, &t);
    esp_foc_sensorless_get_status(0, &st);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, 7.669f, t.speed_bw_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.80f, t.blend_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 19.17f, t.track_bw_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 6.902f, t.speed_filt_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 2000.0f, t.slot_hz);
    /* i_max over the 2 % of fe_rated speed error that saturates it. */
    TEST_ASSERT_FLOAT_WITHIN(0.00719f * 0.02f, 0.00719f, t.kp_w);
    /* wn^2 = K ki and 2 zeta wn = K kp on the integrator plant. */
    TEST_ASSERT_FLOAT_WITHIN(0.2475f * 0.05f, t.kp_w * TWO_PI_F * 7.669f / 1.4f, t.ki_w);

    float kp_i = 0.0f;
    float ki_i = 0.0f;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_pid_design_imc_zoh(12.0f / 1.9f, 148.0e-6f / 1.9f,
                                                         20000.0f, 1000.0f, &kp_i, &ki_i));
    TEST_ASSERT_FLOAT_WITHIN(kp_i * 1.0e-4f, kp_i, t.kp_i);
    TEST_ASSERT_FLOAT_WITHIN(ki_i * 1.0e-4f, ki_i, t.ki_i);

    /* f_base 430 Hz puts 18.5 % at 80 Hz, over the 50 Hz ceiling. */
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 430.1f, st.f_base_hz);
    TEST_ASSERT_EQUAL_FLOAT(50.0f, st.plateau_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 27.8f, st.we_min_hz);
    TEST_ASSERT_EQUAL(ESP_FOC_SL_STATE_IDLE, st.state);
    bench_reset();

    /* The test motor lands inside the clamp: 0.185 * 220.5 Hz. */
    mock_pmsm_init(&s_m, &k_plant);
    cfg_mock(&c, true);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_init(&s_m.base, &c));
    esp_foc_sensorless_get_status(0, &st);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 220.5f, st.f_base_hz);
    TEST_ASSERT_EQUAL_FLOAT(41.0f, st.plateau_hz);
    bench_reset();
}

TEST_CASE("sensorless rejects bad configs and calls before init", "[espFoC][sensorless]")
{
    esp_foc_sensorless_config_t c;

    bench_reset();
    mock_pmsm_init(&s_m, &k_bench);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensorless_run(0));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensorless_stop(0));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensorless_set_iq(0, 0.1f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensorless_clear_fault(0));

    const uint8_t past = CONFIG_ESP_FOC_SL_MAX_AXES;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensorless_run(past));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensorless_set_iq(past, 0.1f));
    TEST_ASSERT_EQUAL(ESP_FOC_SL_STATE_IDLE, esp_foc_sensorless_get_state(past));
    esp_foc_sensorless_deinit(past);
    cfg_bench(&c, true);
    c.axis = past;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensorless_init(&s_m.base, &c));

    cfg_bench(&c, true);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensorless_init(NULL, &c));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensorless_init(&s_m.base, NULL));

    cfg_bench(&c, true);
    c.rs_ohm = 0.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensorless_init(&s_m.base, &c));

    cfg_bench(&c, true);
    c.fe_rated_hz = 0.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensorless_init(&s_m.base, &c));

    cfg_bench(&c, true);
    c.iq_min_a = c.i_max_a * 2.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensorless_init(&s_m.base, &c));

    cfg_bench(&c, true);
    c.startup.vf_hz = 60.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensorless_init(&s_m.base, &c));

    /* The speed loop has to sit under the PLL it is fed by. */
    cfg_bench(&c, true);
    c.speed_bw_hz = 25.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensorless_init(&s_m.base, &c));

    cfg_bench(&c, true);
    c.observer.w_max_frac = 1.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensorless_init(&s_m.base, &c));

    cfg_bench(&c, true);
    c.observer.obs_bw_hz = 5.0f;
    TEST_ASSERT_NOT_EQUAL(ESP_OK, esp_foc_sensorless_init(&s_m.base, &c));

    cfg_bench(&c, true);
    c.map_valid = true;
    c.map.pwm_to_hw[1] = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensorless_init(&s_m.base, &c));

    /* Torque only: the speed bandwidth rule does not apply. */
    cfg_bench(&c, false);
    c.speed_bw_hz = 25.0f;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_init(&s_m.base, &c));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensorless_init(&s_m.base, &c));
    bench_reset();
}

TEST_CASE("sensorless setters keep to their ranges and retune live", "[espFoC][sensorless]")
{
    esp_foc_sensorless_config_t c;
    esp_foc_sensorless_tuning_t t;

    bench_reset();
    mock_pmsm_init(&s_m, &k_bench);
    cfg_bench(&c, false);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_init(&s_m.base, &c));

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensorless_set_speed_ref_hz(0, 100.0f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensorless_set_iq(0, c.i_max_a + 0.01f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensorless_set_iq(0, NAN));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_iq(0, -c.i_max_a));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_iq(0, 0.0f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensorless_set_id(0, 1.0f));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_id(0, 0.2f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensorless_set_vdq_ff(0, 0.0f, 7.0f));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_vdq_ff(0, 0.5f, -0.5f));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_vdq_ff(0, 0.0f, 0.0f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensorless_set_speed_slew(0, 0.0f));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_speed_slew(0, 20.0f));

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensorless_set_current_pi(0, 0.0f, 10.0f));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_current_pi(0, 0.2f, 500.0f));
    esp_foc_sensorless_get_tuning(0, &t);
    TEST_ASSERT_EQUAL_FLOAT(0.2f, t.kp_i);
    TEST_ASSERT_EQUAL_FLOAT(500.0f, t.ki_i);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensorless_set_speed_bw(0, t.track_bw_hz));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensorless_set_speed_bw(0, -1.0f));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_speed_bw(0, 5.0f));
    esp_foc_sensorless_get_tuning(0, &t);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, t.speed_bw_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 4.5f, t.speed_filt_hz);
    /* Kp is fixed by the saturation rule, only ki follows the bandwidth. */
    TEST_ASSERT_FLOAT_WITHIN(0.00719f * 0.02f, 0.00719f, t.kp_w);
    TEST_ASSERT_FLOAT_WITHIN(0.005f, t.kp_w * TWO_PI_F * 5.0f / 1.4f, t.ki_w);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_speed_pi(0, 0.01f, 0.5f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensorless_set_speed_pi(0, -0.01f, 0.5f));

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensorless_clear_fault(0));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_run(0));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensorless_run(0));
    TEST_ASSERT_EQUAL(ESP_FOC_SL_STATE_ARMED, esp_foc_sensorless_get_state(0));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_stop(0));
    TEST_ASSERT_EQUAL(ESP_FOC_SL_STATE_IDLE, esp_foc_sensorless_get_state(0));
    TEST_ASSERT_EQUAL(0, s_m.enable_count);
    TEST_ASSERT_EQUAL(0, s_m.idle_disables);
    bench_reset();
}

TEST_CASE("sensorless armed without a reference keeps the bridge off", "[espFoC][sensorless]")
{
    bench_up(false);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_iq(0, 0.04f));
    esp_foc_sleep_ms(300);
    TEST_ASSERT_EQUAL(ESP_FOC_SL_STATE_ARMED, esp_foc_sensorless_get_state(0));
    TEST_ASSERT_EQUAL(0, s_m.enable_count);
    TEST_ASSERT_EQUAL(0, s_ev_n[ESP_FOC_SL_EV_STARTUP]);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_stop(0));
    TEST_ASSERT_EQUAL(1, s_ev_n[ESP_FOC_SL_EV_STOPPED]);
    TEST_ASSERT_EQUAL(0, s_m.idle_disables);
    bench_reset();
}

TEST_CASE("sensorless torque: both directions on the observer, iq final value",
          "[espFoC][sensorless]")
{
    esp_foc_sensorless_status_t st;
    esp_foc_sensorless_tuning_t t0;

    bench_up(false);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_iq(0, 0.30f));
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SL_EV_RUNNING, 1, 15000));
    TEST_ASSERT_EQUAL(1, s_ev_n[ESP_FOC_SL_EV_STARTUP]);
    TEST_ASSERT_EQUAL(1, s_ev_last[ESP_FOC_SL_EV_STARTUP].dir);
    TEST_ASSERT_EQUAL(1, s_ev_n[ESP_FOC_SL_EV_LOCKED]);
    TEST_ASSERT_EQUAL(1, s_ev_n[ESP_FOC_SL_EV_HANDOFF]);
    TEST_ASSERT_FALSE(s_ev_last[ESP_FOC_SL_EV_RUNNING].speed_mode);
    TEST_ASSERT_TRUE(s_m.wd_on);
    TEST_ASSERT_EQUAL(1, s_m.cal_count);

    esp_foc_sleep_ms(1000);
    const win_t a = sample(g_iq, 500);
    const win_t b = sample(g_iq, 500);
    printf("sl torque+: iq %.3f [%.3f %.3f] fe %.1f Hz\n", b.mean, b.min, b.max,
           mock_pmsm_fe_hz(&s_m));
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.30f, b.mean);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, a.mean, b.mean);
    TEST_ASSERT_LESS_THAN_FLOAT(0.10f, b.max - b.min);

    esp_foc_critical_enter();
    const float th_true = s_m.theta_e;
    esp_foc_sensorless_get_status(0, &st);
    esp_foc_critical_leave();
    const float fe = mock_pmsm_fe_hz(&s_m);
    TEST_ASSERT_TRUE(st.park_on_observer);
    TEST_ASSERT_EQUAL(ESP_FOC_SL_STATE_RUNNING, st.state);
    TEST_ASSERT_GREATER_THAN_FLOAT(60.0f, fe);
    TEST_ASSERT_FLOAT_WITHIN(0.05f * fe, fe, st.we_rads / TWO_PI_F);
    TEST_ASSERT_LESS_THAN_FLOAT(0.5f, fabsf(wrap_pi(st.theta_e_rad - th_true)));

    mock_pmsm_timing_reset(&s_m);
    esp_foc_sleep_ms(500);
    const float avg_us = (float)s_m.cb_us_sum / (float)((s_m.cb_n > 0u) ? s_m.cb_n : 1u);
    printf("sl fast path: avg %.1f us max %u us over %u TEZ\n", avg_us,
           (unsigned)s_m.cb_us_max, (unsigned)s_m.cb_n);
    TEST_ASSERT_LESS_THAN_FLOAT(100.0f, avg_us);

    /* With the current PI slowed to a crawl the feedforward is the whole of
     * the step on vq; restoring the gains must not kick the loop. */
    esp_foc_sensorless_get_tuning(0, &t0);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_current_pi(0, 0.001f, 5.0f));
    esp_foc_sensorless_get_status(0, &st);
    const float vq0 = st.vq_v;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_vdq_ff(0, 0.0f, 0.5f));
    esp_foc_sleep_ms(3);
    esp_foc_sensorless_get_status(0, &st);
    const float dvq = st.vq_v - vq0;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_vdq_ff(0, 0.0f, 0.0f));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_current_pi(0, t0.kp_i, t0.ki_i));
    printf("sl ff: dvq %.3f V\n", dvq);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 0.5f, dvq);
    esp_foc_sleep_ms(500);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.30f, sample(g_iq, 300).mean);
    TEST_ASSERT_EQUAL(0, s_ev_n[ESP_FOC_SL_EV_ABORT]);

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_iq(0, 0.0f));
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SL_EV_CUT, 1, 500));
    TEST_ASSERT_TRUE(wait_state(ESP_FOC_SL_STATE_ARMED, 100));
    TEST_ASSERT_FALSE(s_m.enabled);
    TEST_ASSERT_FALSE(s_m.wd_on);

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_iq(0, -0.30f));
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SL_EV_RUNNING, 2, 15000));
    TEST_ASSERT_EQUAL(-1, s_ev_last[ESP_FOC_SL_EV_STARTUP].dir);
    TEST_ASSERT_EQUAL(-1, s_ev_last[ESP_FOC_SL_EV_RUNNING].dir);
    esp_foc_sleep_ms(1000);
    const win_t c = sample(g_iq, 500);
    printf("sl torque-: iq %.3f [%.3f %.3f] fe %.1f Hz\n", c.mean, c.min, c.max,
           mock_pmsm_fe_hz(&s_m));
    TEST_ASSERT_FLOAT_WITHIN(0.03f, -0.30f, c.mean);
    TEST_ASSERT_LESS_THAN_FLOAT(-60.0f, mock_pmsm_fe_hz(&s_m));

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_iq(0, 0.0f));
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SL_EV_CUT, 2, 500));
    TEST_ASSERT_EQUAL(0, s_ev_n[ESP_FOC_SL_EV_STARTUP_FAILED]);
    bench_down();
}

TEST_CASE("sensorless speed: w_ctrl final value, user iq and id on top",
          "[espFoC][sensorless]")
{
    const float w_t = TWO_PI_F * 100.0f;

    bench_up(true);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_speed_ref_hz(0, 100.0f));
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SL_EV_RUNNING, 1, 15000));
    TEST_ASSERT_TRUE(s_ev_last[ESP_FOC_SL_EV_RUNNING].speed_mode);

    esp_foc_sleep_ms(1500);
    const win_t a = sample(g_w_ctrl, 1000);
    const win_t b = sample(g_w_ctrl, 1000);
    printf("sl speed: w %.1f [%.1f %.1f] rad/s fe %.1f Hz\n", b.mean, b.min, b.max,
           mock_pmsm_fe_hz(&s_m));
    TEST_ASSERT_FLOAT_WITHIN(0.03f * w_t, w_t, b.mean);
    TEST_ASSERT_FLOAT_WITHIN(0.01f * w_t, a.mean, b.mean);
    TEST_ASSERT_LESS_THAN_FLOAT(0.05f * w_t, b.max - b.min);
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 100.0f, mock_pmsm_fe_hz(&s_m));

    /* The speed integrator moves at ~3 Hz: a few ms later the step is all
     * the user's, and seconds later the loop has taken it back out. */
    const float iq0 = sample(g_iq_ref, 50).mean;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_iq(0, 0.05f));
    esp_foc_sleep_ms(4);
    const float iq1 = sample(g_iq_ref, 4).mean;
    printf("sl speed: iq_ref %.3f -> %.3f\n", iq0, iq1);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.05f, iq1 - iq0);
    esp_foc_sleep_ms(2000);
    TEST_ASSERT_FLOAT_WITHIN(0.03f * w_t, w_t, sample(g_w_ctrl, 500).mean);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_iq(0, 0.0f));

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_id(0, 0.2f));
    esp_foc_sleep_ms(500);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.2f, sample(g_id, 200).mean);

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_speed_ref_hz(0, 0.0f));
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SL_EV_CUT, 1, 500));
    TEST_ASSERT_FALSE(s_m.enabled);
    TEST_ASSERT_EQUAL(0, s_ev_n[ESP_FOC_SL_EV_ABORT]);
    bench_down();
}

TEST_CASE("sensorless speed: a sign change ramps down and relaunches the other way",
          "[espFoC][sensorless]")
{
    const float w_t = TWO_PI_F * 80.0f;

    bench_up(true);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_speed_ref_hz(0, 80.0f));
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SL_EV_RUNNING, 1, 15000));
    esp_foc_sleep_ms(500);

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_speed_ref_hz(0, -80.0f));
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SL_EV_REVERSING, 1, 500));
    TEST_ASSERT_EQUAL(-1, s_ev_last[ESP_FOC_SL_EV_REVERSING].dir);
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SL_EV_RUNNING, 2, 20000));
    TEST_ASSERT_EQUAL(2, s_ev_n[ESP_FOC_SL_EV_STARTUP]);
    TEST_ASSERT_EQUAL(-1, s_ev_last[ESP_FOC_SL_EV_STARTUP].dir);
    TEST_ASSERT_EQUAL(0, s_ev_n[ESP_FOC_SL_EV_CUT]);

    esp_foc_sleep_ms(1500);
    const win_t b = sample(g_w_ctrl, 1000);
    printf("sl speed rev: w %.1f [%.1f %.1f] rad/s fe %.1f Hz\n", b.mean, b.min, b.max,
           mock_pmsm_fe_hz(&s_m));
    TEST_ASSERT_FLOAT_WITHIN(0.03f * w_t, -w_t, b.mean);
    TEST_ASSERT_LESS_THAN_FLOAT(-70.0f, mock_pmsm_fe_hz(&s_m));

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_speed_ref_hz(0, 0.0f));
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SL_EV_CUT, 1, 500));
    bench_down();
}

TEST_CASE("sensorless torque: a sign change in the startup cuts and relaunches",
          "[espFoC][sensorless]")
{
    bench_up(false);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_iq(0, 0.30f));
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SL_EV_STARTUP, 1, 500));
    esp_foc_sleep_ms(300);
    TEST_ASSERT_EQUAL(ESP_FOC_SL_STATE_ALIGN, esp_foc_sensorless_get_state(0));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_iq(0, -0.30f));
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SL_EV_REVERSING, 1, 500));
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SL_EV_RUNNING, 1, 15000));
    TEST_ASSERT_EQUAL(-1, s_ev_last[ESP_FOC_SL_EV_RUNNING].dir);
    TEST_ASSERT_EQUAL(2, s_ev_n[ESP_FOC_SL_EV_STARTUP]);
    TEST_ASSERT_EQUAL(1, s_ev_n[ESP_FOC_SL_EV_HANDOFF]);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_iq(0, 0.0f));
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SL_EV_CUT, 1, 500));
    bench_down();
}

TEST_CASE("sensorless torque: vlim saturation stays bounded and does not wind up",
          "[espFoC][sensorless]")
{
    bench_up(false);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_iq(0, 0.45f));
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SL_EV_RUNNING, 1, 15000));
    esp_foc_sleep_ms(1500);
    const win_t v = sample(g_vmag, 1000);
    const win_t i = sample(g_iq, 500);
    printf("sl vlim: |v| %.2f max %.2f V iq %.3f fe %.1f Hz\n", v.mean, v.max, i.mean,
           mock_pmsm_fe_hz(&s_m));
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(12.0f * 0.57735f * 1.01f, v.max);
    TEST_ASSERT_LESS_THAN_FLOAT(0.6f, i.max);
    TEST_ASSERT_LESS_THAN_FLOAT(250.0f, mock_pmsm_fe_hz(&s_m));
    TEST_ASSERT_EQUAL(0, s_ev_n[ESP_FOC_SL_EV_ABORT]);

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_iq(0, 0.20f));
    esp_foc_sleep_ms(300);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.20f, sample(g_iq, 300).mean);

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_iq(0, 0.0f));
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SL_EV_CUT, 1, 500));
    bench_down();
}

TEST_CASE("sensorless: a stalled rotor aborts and latches until clear_fault",
          "[espFoC][sensorless]")
{
    esp_foc_sensorless_status_t st;

    bench_up(false);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_iq(0, 0.30f));
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SL_EV_RUNNING, 1, 15000));
    esp_foc_sleep_ms(500);
    s_m.locked = true;
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SL_EV_ABORT, 1, 2000));
    const esp_foc_sensorless_abort_t why = s_ev_last[ESP_FOC_SL_EV_ABORT].abort;
    printf("sl stall running: abort %d\n", (int)why);
    TEST_ASSERT_TRUE((why == ESP_FOC_SL_ABORT_BEMF) || (why == ESP_FOC_SL_ABORT_COLLAPSE));
    TEST_ASSERT_TRUE(wait_state(ESP_FOC_SL_STATE_FAULT, 100));
    TEST_ASSERT_FALSE(s_m.enabled);
    esp_foc_sensorless_get_status(0, &st);
    TEST_ASSERT_EQUAL(why, st.abort);

    esp_foc_sleep_ms(700);
    TEST_ASSERT_EQUAL(1, s_ev_n[ESP_FOC_SL_EV_STARTUP]);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensorless_run(0));

    /* Locked from the start: either the follow proof or the BEMF gate. */
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_clear_fault(0));
    TEST_ASSERT_EQUAL(1, s_ev_n[ESP_FOC_SL_EV_FAULT_CLEARED]);
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SL_EV_STARTUP, 2, 2000));
    const bool latched = wait_state(ESP_FOC_SL_STATE_FAULT, 8000);
    printf("sl stall start: abort %d fail %d\n", (int)s_ev_last[ESP_FOC_SL_EV_ABORT].abort,
           (int)s_ev_last[ESP_FOC_SL_EV_STARTUP_FAILED].fail);
    TEST_ASSERT_TRUE(latched);
    TEST_ASSERT_EQUAL(2, s_ev_n[ESP_FOC_SL_EV_ABORT] + s_ev_n[ESP_FOC_SL_EV_STARTUP_FAILED]);
    TEST_ASSERT_EQUAL(1, s_ev_n[ESP_FOC_SL_EV_RUNNING]);
    TEST_ASSERT_FALSE(s_m.enabled);

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_iq(0, 0.0f));
    s_m.locked = false;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_clear_fault(0));
    TEST_ASSERT_EQUAL(ESP_FOC_SL_STATE_ARMED, esp_foc_sensorless_get_state(0));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensorless_clear_fault(0));
    bench_down();
}

TEST_CASE("sensorless: an inverter fault latches and clears through the stack",
          "[espFoC][sensorless]")
{
    esp_foc_sensorless_status_t st;

    bench_up(false);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_iq(0, 0.30f));
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SL_EV_RUNNING, 1, 15000));
    esp_foc_sleep_ms(300);
    mock_pmsm_trip(&s_m, ESP_FOC_FAULT_ILIMIT);
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SL_EV_FAULT, 1, 500));
    TEST_ASSERT_EQUAL(ESP_FOC_FAULT_ILIMIT, s_ev_last[ESP_FOC_SL_EV_FAULT].fault);
    TEST_ASSERT_EQUAL(ESP_FOC_SL_STATE_RUNNING, s_ev_last[ESP_FOC_SL_EV_FAULT].state);
    TEST_ASSERT_TRUE(wait_state(ESP_FOC_SL_STATE_FAULT, 100));
    esp_foc_sensorless_get_status(0, &st);
    TEST_ASSERT_EQUAL(ESP_FOC_FAULT_ILIMIT, st.fault);

    esp_foc_sleep_ms(300);
    TEST_ASSERT_EQUAL(1, s_ev_n[ESP_FOC_SL_EV_STARTUP]);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_iq(0, 0.0f));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_clear_fault(0));
    TEST_ASSERT_EQUAL(1, s_m.clear_count);
    TEST_ASSERT_FALSE(s_m.faulted);
    TEST_ASSERT_EQUAL(ESP_FOC_SL_STATE_ARMED, esp_foc_sensorless_get_state(0));
    esp_foc_sensorless_get_status(0, &st);
    TEST_ASSERT_EQUAL(ESP_FOC_FAULT_NONE, st.fault);
    bench_down();
}

#if CONFIG_ESP_FOC_SL_MAX_AXES >= 2
static float iq_mean_axis(uint8_t axis, uint32_t ms)
{
    float acc = 0.0f;
    uint32_t n = 0;
    for (uint32_t t = 0; t < ms; t += 2u) {
        esp_foc_sensorless_status_t st;
        esp_foc_sensorless_get_status(axis, &st);
        acc += st.iq_a;
        n++;
        esp_foc_sleep_ms(2);
    }
    return acc / (float)((n > 0u) ? n : 1u);
}

TEST_CASE("sensorless: two axes run independently on their own inverters",
          "[espFoC][sensorless]")
{
    static mock_pmsm_t m1;
    mock_pmsm_params_t p1 = k_plant;
    esp_foc_sensorless_config_t c;

    bench_reset();
    p1.theta0 = -1.1f;
    mock_pmsm_init(&s_m, &k_plant);
    mock_pmsm_init(&m1, &p1);
    cfg_mock(&c, false);
    c.axis = 0;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_init(&s_m.base, &c));
    c.axis = 1;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_init(&m1.base, &c));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensorless_init(&m1.base, &c));
    mock_pmsm_start(&s_m);
    mock_pmsm_start(&m1);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_run(0));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_run(1));
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SL_EV_ARMED, 2, 200));
    TEST_ASSERT_EQUAL(1, s_ev_ax[0][ESP_FOC_SL_EV_ARMED]);
    TEST_ASSERT_EQUAL(1, s_ev_ax[1][ESP_FOC_SL_EV_ARMED]);

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_iq(0, 0.30f));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_iq(1, -0.30f));
    for (uint32_t t = 0; (t < 20000u) && ((s_ev_ax[0][ESP_FOC_SL_EV_RUNNING] < 1) ||
                                         (s_ev_ax[1][ESP_FOC_SL_EV_RUNNING] < 1));
         t += 5u) {
        esp_foc_sleep_ms(5);
    }
    TEST_ASSERT_EQUAL(1, s_ev_ax[0][ESP_FOC_SL_EV_RUNNING]);
    TEST_ASSERT_EQUAL(1, s_ev_ax[1][ESP_FOC_SL_EV_RUNNING]);
    TEST_ASSERT_EQUAL(1, s_ev_ax_last[0][ESP_FOC_SL_EV_RUNNING].dir);
    TEST_ASSERT_EQUAL(-1, s_ev_ax_last[1][ESP_FOC_SL_EV_RUNNING].dir);

    esp_foc_sleep_ms(1000);
    const float iq0 = iq_mean_axis(0, 400);
    const float iq1 = iq_mean_axis(1, 400);
    printf("sl 2 axes: iq0 %.3f fe0 %.1f Hz | iq1 %.3f fe1 %.1f Hz\n", iq0,
           mock_pmsm_fe_hz(&s_m), iq1, mock_pmsm_fe_hz(&m1));
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.30f, iq0);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, -0.30f, iq1);
    TEST_ASSERT_GREATER_THAN_FLOAT(60.0f, mock_pmsm_fe_hz(&s_m));
    TEST_ASSERT_LESS_THAN_FLOAT(-60.0f, mock_pmsm_fe_hz(&m1));

    /* Cutting one axis leaves the other caught and driving. */
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_set_iq(0, 0.0f));
    for (uint32_t t = 0; (t < 500u) && (s_ev_ax[0][ESP_FOC_SL_EV_CUT] < 1); t += 5u) {
        esp_foc_sleep_ms(5);
    }
    TEST_ASSERT_EQUAL(1, s_ev_ax[0][ESP_FOC_SL_EV_CUT]);
    TEST_ASSERT_EQUAL(0, s_ev_ax[1][ESP_FOC_SL_EV_CUT]);
    TEST_ASSERT_FALSE(s_m.enabled);
    TEST_ASSERT_TRUE(m1.enabled);
    esp_foc_sleep_ms(300);
    TEST_ASSERT_EQUAL(ESP_FOC_SL_STATE_RUNNING, esp_foc_sensorless_get_state(1));
    TEST_ASSERT_FLOAT_WITHIN(0.03f, -0.30f, iq_mean_axis(1, 300));

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_stop(1));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensorless_stop(0));
    TEST_ASSERT_FALSE(m1.enabled);
    TEST_ASSERT_EQUAL(0, s_ev_n[ESP_FOC_SL_EV_ABORT]);
    TEST_ASSERT_EQUAL(0, s_ev_n[ESP_FOC_SL_EV_STARTUP_FAILED]);
    bench_reset();
}
#endif

#endif /* CONFIG_ESP_FOC_STACK_SENSORLESS */
