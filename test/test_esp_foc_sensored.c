/*
 * Unit tests for the sensored stack against mock_pmsm_inverter and its
 * absolute encoder: a 7 pp PMSM (2 ohm, 1 mH, 5 mWb, K = 36750 rad/s^2/A)
 * with Coulomb + viscous friction worth about 0.15 A + 1 mA/Hz of iq, at a
 * 5 kHz virtual PWM and a 2.5 kHz encoder.
 */
#include "sdkconfig.h"

#if CONFIG_ESP_FOC_STACK_SENSORED

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"
#include "esp_err.h"
#include "espFoC/motor_control/esp_foc_sensored.h"
#include "espFoC/osal/esp_foc_osal.h"
#include "espFoC/utils/esp_foc_pid.h"
#include "mock_pmsm_inverter.h"

#define TWO_PI_F 6.28318530718f
#define EV_COUNT (ESP_FOC_SD_EV_FAULT_CLEARED + 1)
#define K_MOCK (7.0f * 1.5f * 7.0f * 0.005f / 1.0e-5f)
/* Encoder mounting error the Park offset has to take back out. */
#define ENC_OFFSET_E 0.3f

static mock_pmsm_t s_m;
static mock_pmsm_rotor_t s_r;
static volatile int s_ev_n[EV_COUNT];
static esp_foc_sensored_event_t s_ev_last[EV_COUNT];
static volatile int s_ev_ax[CONFIG_ESP_FOC_SD_MAX_AXES][EV_COUNT];

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

static void on_ev(void *ctx, const esp_foc_sensored_event_t *e)
{
    (void)ctx;
    if ((int)e->ev < EV_COUNT) {
        s_ev_last[e->ev] = *e;
        s_ev_n[e->ev]++;
        if (e->axis < CONFIG_ESP_FOC_SD_MAX_AXES) {
            s_ev_ax[e->axis][e->ev]++;
        }
    }
}

static void ev_reset(void)
{
    memset((void *)s_ev_n, 0, sizeof(s_ev_n));
    memset(s_ev_last, 0, sizeof(s_ev_last));
    memset((void *)s_ev_ax, 0, sizeof(s_ev_ax));
}

static bool wait_ev(esp_foc_sensored_ev_t ev, int n, uint32_t timeout_ms)
{
    for (uint32_t t = 0; t < timeout_ms; t += 5u) {
        if (s_ev_n[ev] >= n) {
            return true;
        }
        esp_foc_sleep_ms(5);
    }
    return s_ev_n[ev] >= n;
}

typedef float (*st_get_t)(const esp_foc_sensored_status_t *s);

typedef struct {
    float mean;
    float min;
    float max;
    float rms_dev;
} win_t;

static float g_iq(const esp_foc_sensored_status_t *s)
{
    return s->iq_a;
}

static float g_id(const esp_foc_sensored_status_t *s)
{
    return s->id_a;
}

static float g_iq_ref(const esp_foc_sensored_status_t *s)
{
    return s->iq_ref_a;
}

static float g_we_hz(const esp_foc_sensored_status_t *s)
{
    return s->we_rads / TWO_PI_F;
}

static float g_theta(const esp_foc_sensored_status_t *s)
{
    return s->theta_m_rad;
}

static float g_vmag(const esp_foc_sensored_status_t *s)
{
    return sqrtf(s->vd_v * s->vd_v + s->vq_v * s->vq_v);
}

static win_t sample_axis(uint8_t axis, st_get_t get, uint32_t ms)
{
    win_t w = {0.0f, 1.0e9f, -1.0e9f, 0.0f};
    float sq = 0.0f;
    uint32_t n = 0;
    /* 1 ms is an odd number of PWM periods, so successive samples land on
     * both phases of the fetch cycle instead of aliasing one of them. */
    for (uint32_t t = 0; t < ms; t++) {
        esp_foc_sensored_status_t st;
        esp_foc_sensored_get_status(axis, &st);
        const float x = get(&st);
        w.mean += x;
        sq += x * x;
        w.min = fminf(w.min, x);
        w.max = fmaxf(w.max, x);
        n++;
        esp_foc_sleep_ms(1);
    }
    const float dn = (float)((n > 0u) ? n : 1u);
    w.mean /= dn;
    w.rms_dev = sqrtf(fmaxf(sq / dn - w.mean * w.mean, 0.0f));
    return w;
}

static win_t sample(st_get_t get, uint32_t ms)
{
    return sample_axis(0, get, ms);
}

static void cfg_mock(esp_foc_sensored_config_t *c, esp_foc_sensored_control_t control)
{
    esp_foc_sensored_default_config(c);
    c->control = control;
    c->rs_ohm = k_plant.rs;
    c->ls_h = k_plant.ls;
    c->psi_wb = k_plant.psi;
    c->pole_pairs = (uint8_t)k_plant.pp;
    c->k_rads2_a = K_MOCK;
    c->j_kgm2 = k_plant.j;
    c->park_offset_rad = -ENC_OFFSET_E;
    c->fetch_hz = 2500u;
    /* 5 kHz: the bench current bandwidth would sit at the one-sample edge. */
    c->i_bw_hz = 300.0f;
    c->i_tune_backoff = 1.0f;
    c->guard.overspeed_hz = 450.0f;
    c->on_event = on_ev;
}

/* A failed assertion longjmps out with the stack still up; every test
 * starts from a clean bench whatever the previous one left. */
static void bench_reset(void)
{
    for (uint8_t a = 0; a < CONFIG_ESP_FOC_SD_MAX_AXES; a++) {
        esp_foc_sensored_deinit(a);
    }
    mock_pmsm_stop();
    ev_reset();
}

static void plant_up(const mock_pmsm_params_t *p)
{
    bench_reset();
    mock_pmsm_init(&s_m, p);
    mock_pmsm_rotor_init(&s_r, &s_m, ENC_OFFSET_E);
    s_r.extrapolate = true;
}

static void bench_up_cfg(const esp_foc_sensored_config_t *c)
{
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_init(&s_m.base, &s_r.base, c));
    mock_pmsm_start(&s_m);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_run(c->axis));
    TEST_ASSERT_EQUAL(ESP_FOC_SD_STATE_RUNNING, esp_foc_sensored_get_state(c->axis));
}

static void bench_up(esp_foc_sensored_control_t control)
{
    esp_foc_sensored_config_t c;
    plant_up(&k_plant);
    cfg_mock(&c, control);
    bench_up_cfg(&c);
}

static void bench_down(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_stop(0));
    TEST_ASSERT_EQUAL(ESP_FOC_SD_STATE_IDLE, esp_foc_sensored_get_state(0));
    TEST_ASSERT_FALSE(s_m.enabled);
    bench_reset();
}

TEST_CASE("sensored rejects bad configs, calls before init and setters above the level",
          "[espFoC][sensored]")
{
    esp_foc_sensored_config_t c;

    plant_up(&k_plant);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensored_run(0));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensored_stop(0));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensored_set_iq(0, 0.1f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensored_clear_fault(0));

    const uint8_t past = CONFIG_ESP_FOC_SD_MAX_AXES;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensored_run(past));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensored_set_iq(past, 0.1f));
    TEST_ASSERT_EQUAL(ESP_FOC_SD_STATE_IDLE, esp_foc_sensored_get_state(past));
    esp_foc_sensored_deinit(past);
    cfg_mock(&c, ESP_FOC_SD_CONTROL_TORQUE);
    c.axis = past;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensored_init(&s_m.base, &s_r.base, &c));

    /* The encoder is not optional. */
    cfg_mock(&c, ESP_FOC_SD_CONTROL_TORQUE);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensored_init(&s_m.base, NULL, &c));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensored_init(NULL, &s_r.base, &c));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensored_init(&s_m.base, &s_r.base, NULL));

    cfg_mock(&c, ESP_FOC_SD_CONTROL_TORQUE);
    c.fetch_hz = 3000u;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensored_init(&s_m.base, &s_r.base, &c));
    cfg_mock(&c, ESP_FOC_SD_CONTROL_VELOCITY);
    c.k_rads2_a = 0.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensored_init(&s_m.base, &s_r.base, &c));
    cfg_mock(&c, ESP_FOC_SD_CONTROL_POSITION);
    c.cogging.smooth = 8u;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensored_init(&s_m.base, &s_r.base, &c));
    cfg_mock(&c, ESP_FOC_SD_CONTROL_POSITION);
    c.rs_ohm = 0.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensored_init(&s_m.base, &s_r.base, &c));

    /* A torque-only axis refuses everything above it, and K is not needed. */
    cfg_mock(&c, ESP_FOC_SD_CONTROL_TORQUE);
    c.k_rads2_a = 0.0f;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_init(&s_m.base, &s_r.base, &c));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensored_init(&s_m.base, &s_r.base, &c));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensored_set_speed_ref_hz(0, 10.0f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensored_set_speed_pi(0, 0.01f, 1.0f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensored_set_position_ref_rad(0, 1.0f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensored_set_joint(0, 1.0f, 0.0f, 0.0f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      esp_foc_sensored_set_mode(0, ESP_FOC_SD_MODE_VELOCITY));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensored_cogging_learn(0, 1000u, NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensored_cogging_enable(0, true));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensored_set_iq(0, c.i_max_a * 2.0f));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_iq(0, 0.1f));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_iq(0, 0.0f));

    esp_foc_sensored_tuning_t t;
    esp_foc_sensored_get_tuning(0, &t);
    float kp_i = 0.0f;
    float ki_i = 0.0f;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_pid_design_imc_zoh(12.0f / 2.0f, 1.0e-3f / 2.0f, 5000.0f,
                                                         300.0f, &kp_i, &ki_i));
    TEST_ASSERT_FLOAT_WITHIN(kp_i * 1.0e-4f, kp_i, t.kp_i);
    TEST_ASSERT_FLOAT_WITHIN(ki_i * 1.0e-4f, ki_i, t.ki_i);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 2500.0f, t.slot_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 220.5f, t.f_base_hz);
    bench_reset();

    /* A velocity axis takes torque and velocity, never position. */
    cfg_mock(&c, ESP_FOC_SD_CONTROL_VELOCITY);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_init(&s_m.base, &s_r.base, &c));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensored_set_position_ref_rad(0, 1.0f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensored_set_position_kp(0, 1.0f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      esp_foc_sensored_set_mode(0, ESP_FOC_SD_MODE_POSITION));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensored_set_speed_ref_hz(0, 1000.0f));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_mode(0, ESP_FOC_SD_MODE_TORQUE));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensored_set_speed_ref_hz(0, 10.0f));
    bench_reset();
}

TEST_CASE("sensored torque mode tracks iq both ways on the compensated angle",
          "[espFoC][sensored]")
{
    bench_up(ESP_FOC_SD_CONTROL_TORQUE);
    TEST_ASSERT_EQUAL(1, s_ev_n[ESP_FOC_SD_EV_ARMED]);
    TEST_ASSERT_EQUAL(1, s_ev_n[ESP_FOC_SD_EV_RUNNING]);
    TEST_ASSERT_TRUE(s_m.enabled);
    TEST_ASSERT_EQUAL(1, s_m.cal_count);

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_iq(0, 0.25f));
    esp_foc_sleep_ms(800);
    const win_t iq_p = sample(g_iq, 300);
    const win_t id_p = sample(g_id, 300);
    const float fe_p = mock_pmsm_fe_hz(&s_m);

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_iq(0, -0.25f));
    esp_foc_sleep_ms(1500);
    const win_t iq_n = sample(g_iq, 300);
    const win_t id_n = sample(g_id, 300);
    const float fe_n = mock_pmsm_fe_hz(&s_m);
    printf("sd torque: iq+ %.3f id+ %.3f [%.3f..%.3f] fe+ %.1f | iq- %.3f id- %.3f fe- %.1f Hz\n",
           iq_p.mean, id_p.mean, id_p.min, id_p.max, fe_p, iq_n.mean, id_n.mean, fe_n);

    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.25f, iq_p.mean);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, -0.25f, iq_n.mean);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, id_p.mean);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, id_n.mean);
    TEST_ASSERT_LESS_THAN_FLOAT(0.03f, iq_p.rms_dev);
    TEST_ASSERT_GREATER_THAN_FLOAT(20.0f, fe_p);
    TEST_ASSERT_LESS_THAN_FLOAT(-20.0f, fe_n);
    /* The two legs only differ by sign once Park is on the true d axis. */
    TEST_ASSERT_FLOAT_WITHIN(0.10f * fabsf(fe_p), fabsf(fe_p), fabsf(fe_n));
    TEST_ASSERT_EQUAL(0, s_ev_n[ESP_FOC_SD_EV_ABORT]);
    bench_down();
}

TEST_CASE("sensored velocity mode settles without oscillating, user iq and id on top",
          "[espFoC][sensored]")
{
    bench_up(ESP_FOC_SD_CONTROL_VELOCITY);
    esp_foc_sensored_tuning_t t;
    esp_foc_sensored_get_tuning(0, &t);
    printf("sd speed design: ripple %.3f rad/s want %.2f noise %.1f phase %.2f bw %.2f fc %.2f "
           "Kp %.5f Ki %.4f\n",
           t.ripple_rads, t.speed_bw_want_hz, t.speed_bw_noise_hz, t.speed_bw_phase_hz,
           t.speed_bw_hz, t.speed_fc_hz, t.kp_w, t.ki_w);
    /* 1.54 % of 220.5 Hz lands under the 4 Hz floor; the PLL phase ceiling
     * 0.5 * 180 / 5 / 2.3 sits above it. */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 4.0f, t.speed_bw_want_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 7.826f, t.speed_bw_phase_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 4.0f, t.speed_bw_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 9.2f, t.speed_fc_hz);
    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT(0.0f, t.ripple_rads);

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_speed_ref_hz(0, 50.0f));
    esp_foc_sleep_ms(1500);
    const win_t w1 = sample(g_we_hz, 300);
    esp_foc_sensored_window_t win;
    esp_foc_sensored_get_window(0, &win);
    esp_foc_sleep_ms(200);
    esp_foc_sensored_get_window(0, &win);
    const win_t w2 = sample(g_we_hz, 300);
    printf("sd speed +50: we %.2f [%.2f..%.2f] then %.2f rms %.3f | win n %lu mean %.2f "
           "werr rms %.3f fe %.1f\n",
           w1.mean, w1.min, w1.max, w2.mean, w2.rms_dev, (unsigned long)win.n,
           win.w_mean_hz, sqrtf(win.werr_sq_hz2 / (float)win.n), mock_pmsm_fe_hz(&s_m));
    TEST_ASSERT_FLOAT_WITHIN(1.5f, 50.0f, w1.mean);
    TEST_ASSERT_FLOAT_WITHIN(1.5f, 50.0f, w2.mean);
    TEST_ASSERT_FLOAT_WITHIN(0.3f, w1.mean, w2.mean);
    TEST_ASSERT_LESS_THAN_FLOAT(1.0f, w2.rms_dev);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 50.0f, mock_pmsm_fe_hz(&s_m));
    TEST_ASSERT_GREATER_THAN_UINT32(400u, win.n);
    TEST_ASSERT_FLOAT_WITHIN(1.5f, 50.0f, win.w_mean_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, win.w_ref_hz);

    /* User iq is feedforward here: the integrator takes it back out. */
    const win_t iqr0 = sample(g_iq_ref, 200);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_iq(0, 0.10f));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_id(0, -0.20f));
    esp_foc_sleep_ms(1500);
    const win_t w3 = sample(g_we_hz, 300);
    const win_t id3 = sample(g_id, 200);
    const win_t iqr3 = sample(g_iq_ref, 200);
    printf("sd speed ff: we %.2f id %.3f iq* %.3f -> %.3f\n", w3.mean, id3.mean, iqr0.mean,
           iqr3.mean);
    TEST_ASSERT_FLOAT_WITHIN(1.5f, 50.0f, w3.mean);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, -0.20f, id3.mean);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, iqr0.mean, iqr3.mean);

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_iq(0, 0.0f));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_id(0, 0.0f));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_speed_ref_hz(0, -50.0f));
    esp_foc_sleep_ms(2000);
    const win_t w4 = sample(g_we_hz, 300);
    printf("sd speed -50: we %.2f rms %.3f fe %.1f\n", w4.mean, w4.rms_dev,
           mock_pmsm_fe_hz(&s_m));
    TEST_ASSERT_FLOAT_WITHIN(1.5f, -50.0f, w4.mean);
    TEST_ASSERT_LESS_THAN_FLOAT(1.0f, w4.rms_dev);
    TEST_ASSERT_EQUAL(0, s_ev_n[ESP_FOC_SD_EV_ABORT]);
    bench_down();
}

TEST_CASE("sensored position mode reaches multi-turn targets without overshoot",
          "[espFoC][sensored]")
{
    bench_up(ESP_FOC_SD_CONTROL_POSITION);
    esp_foc_sensored_tuning_t t;
    esp_foc_sensored_get_tuning(0, &t);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, TWO_PI_F * t.speed_fc_hz / 4.0f, t.kp_pos);

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_origin(0));
    esp_foc_sleep_ms(200);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, sample(g_theta, 50).mean);

    const float goals[3] = {1.0f, 4.0f * TWO_PI_F + 0.5f, -0.75f};
    float from = 0.0f;
    for (int k = 0; k < 3; k++) {
        const float goal = goals[k];
        TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_position_ref_rad(0, goal));
        const float dir = (goal > from) ? 1.0f : -1.0f;
        float peak = from;
        for (uint32_t ms = 0; ms < 9000u; ms += 10u) {
            esp_foc_sensored_status_t st;
            esp_foc_sensored_get_status(0, &st);
            peak = (dir > 0.0f) ? fmaxf(peak, st.theta_m_rad) : fminf(peak, st.theta_m_rad);
            esp_foc_sleep_ms(10);
        }
        const win_t th = sample(g_theta, 300);
        esp_foc_sensored_status_t st;
        esp_foc_sensored_get_status(0, &st);
        const float over = dir * (peak - goal);
        printf("sd pos -> %.3f: th %.4f [%.4f..%.4f] overshoot %.4f inpos %d\n", goal, th.mean,
               th.min, th.max, over, st.inpos ? 1 : 0);
        TEST_ASSERT_FLOAT_WITHIN(0.01f, goal, th.mean);
        TEST_ASSERT_LESS_THAN_FLOAT(0.005f, th.max - th.min);
        TEST_ASSERT_LESS_THAN_FLOAT(0.02f, over);
        TEST_ASSERT_TRUE(st.inpos);
        TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, goal, st.theta_ref_rad);
        from = goal;
    }

    /* The origin moves under the reference, not the reference under the shaft. */
    const win_t before = sample(g_theta, 100);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_origin(0));
    esp_foc_sleep_ms(500);
    const win_t after = sample(g_theta, 200);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, after.mean);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.75f, before.mean);
    TEST_ASSERT_EQUAL(0, s_ev_n[ESP_FOC_SD_EV_ABORT]);
    bench_down();
}

TEST_CASE("sensored joint samples track a constant-speed ramp", "[espFoC][sensored]")
{
    bench_up(ESP_FOC_SD_CONTROL_POSITION);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_origin(0));
    esp_foc_sleep_ms(200);

    /* Samples come every 10 ms, so the reference is a staircase w·10 ms high
     * around the shaft: 0.02 rad at this speed. */
    const float w = 2.0f;
    float err_max = 0.0f;
    float th_ref = 0.0f;
    for (uint32_t ms = 0; ms < 3000u; ms += 10u) {
        th_ref = w * (float)ms * 1.0e-3f;
        TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_joint(0, th_ref, w, 0.0f));
        esp_foc_sleep_ms(10);
        if (ms >= 500u) {
            esp_foc_sensored_status_t st;
            esp_foc_sensored_get_status(0, &st);
            err_max = fmaxf(err_max, fabsf(st.theta_m_rad - st.theta_ref_rad));
        }
    }
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_joint(0, th_ref, 0.0f, 0.0f));
    esp_foc_sleep_ms(1500);
    const win_t th = sample(g_theta, 200);
    printf("sd joint ramp %.1f rad/s: err_max %.4f rad end %.4f (ref %.4f)\n", w, err_max,
           th.mean, th_ref);
    TEST_ASSERT_LESS_THAN_FLOAT(0.05f, err_max);
    /* A stop out of motion parks inside the Coulomb band: only the speed
     * integrator, fed by kp_pos·e, pulls the last few tens of mrad in. */
    TEST_ASSERT_FLOAT_WITHIN(0.04f, th_ref, th.mean);
    TEST_ASSERT_EQUAL(0, s_ev_n[ESP_FOC_SD_EV_ABORT]);
    bench_down();
}

TEST_CASE("sensored set_mode is bumpless through position, torque and velocity",
          "[espFoC][sensored]")
{
    bench_up(ESP_FOC_SD_CONTROL_POSITION);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_mode(0, ESP_FOC_SD_MODE_VELOCITY));
    TEST_ASSERT_EQUAL(1, s_ev_n[ESP_FOC_SD_EV_MODE]);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_speed_ref_hz(0, 30.0f));
    esp_foc_sleep_ms(1500);
    TEST_ASSERT_FLOAT_WITHIN(1.5f, 30.0f, sample(g_we_hz, 200).mean);

    /* Into position at speed: the shaft stops where the switch found it. */
    esp_foc_sensored_status_t s0;
    esp_foc_sensored_get_status(0, &s0);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_mode(0, ESP_FOC_SD_MODE_POSITION));
    esp_foc_sensored_status_t s1;
    esp_foc_sensored_get_status(0, &s1);
    esp_foc_sleep_ms(3000);
    const win_t th = sample(g_theta, 200);
    const win_t w = sample(g_we_hz, 200);
    printf("sd mode v->p: th0 %.3f ref %.3f -> %.3f we %.2f\n", s0.theta_m_rad,
           s1.theta_ref_rad, th.mean, w.mean);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, s0.theta_m_rad, s1.theta_ref_rad);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, s1.theta_ref_rad, th.mean);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, w.mean);

    /* Velocity -> torque keeps the iq in flight; torque -> velocity keeps
     * the speed. */
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_mode(0, ESP_FOC_SD_MODE_VELOCITY));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_speed_ref_hz(0, 30.0f));
    esp_foc_sleep_ms(1500);
    const win_t iq_v = sample(g_iq_ref, 200);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_mode(0, ESP_FOC_SD_MODE_TORQUE));
    const win_t iq_t = sample(g_iq_ref, 100);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, iq_v.mean, iq_t.mean);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_mode(0, ESP_FOC_SD_MODE_VELOCITY));
    const win_t w_v = sample(g_we_hz, 300);
    printf("sd mode v->t->v: iq* %.3f -> %.3f, we %.2f [%.2f..%.2f]\n", iq_v.mean, iq_t.mean,
           w_v.mean, w_v.min, w_v.max);
    TEST_ASSERT_GREATER_THAN_FLOAT(20.0f, w_v.min);
    TEST_ASSERT_LESS_THAN_FLOAT(40.0f, w_v.max);
    TEST_ASSERT_EQUAL(5, s_ev_n[ESP_FOC_SD_EV_MODE]);
    TEST_ASSERT_EQUAL(0, s_ev_n[ESP_FOC_SD_EV_ABORT]);
    bench_down();
}

TEST_CASE("sensored speed loop recovers from voltage saturation without windup",
          "[espFoC][sensored]")
{
    bench_up(ESP_FOC_SD_CONTROL_VELOCITY);
    /* f_base is 220 Hz: 400 Hz asks for twice the BEMF the bus can give. */
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_speed_ref_hz(0, 400.0f));
    esp_foc_sleep_ms(2500);
    const win_t v = sample(g_vmag, 300);
    const win_t w_sat = sample(g_we_hz, 300);
    const win_t iq_sat = sample(g_iq_ref, 100);
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(12.0f / sqrtf(3.0f) + 0.05f, v.max);
    TEST_ASSERT_LESS_THAN_FLOAT(300.0f, w_sat.mean);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.70f, iq_sat.mean);

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_speed_ref_hz(0, 50.0f));
    float lo = 1.0e9f;
    for (uint32_t ms = 0; ms < 2000u; ms += 5u) {
        esp_foc_sensored_status_t st;
        esp_foc_sensored_get_status(0, &st);
        if (ms > 300u) {
            lo = fminf(lo, st.we_rads / TWO_PI_F);
        }
        esp_foc_sleep_ms(5);
    }
    const win_t w = sample(g_we_hz, 300);
    printf("sd vlim: |v| max %.2f V we_sat %.1f Hz iq* %.3f | back to 50: %.2f min %.2f\n",
           v.max, w_sat.mean, iq_sat.mean, w.mean, lo);
    TEST_ASSERT_FLOAT_WITHIN(1.5f, 50.0f, w.mean);
    TEST_ASSERT_GREATER_THAN_FLOAT(40.0f, lo);
    TEST_ASSERT_EQUAL(0, s_ev_n[ESP_FOC_SD_EV_ABORT]);
    bench_down();
}

TEST_CASE("sensored stale and failing encoder abort and latch until clear_fault",
          "[espFoC][sensored]")
{
    bench_up(ESP_FOC_SD_CONTROL_VELOCITY);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_speed_ref_hz(0, 30.0f));
    esp_foc_sleep_ms(800);

    s_r.freeze = true;
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SD_EV_ABORT, 1, 500));
    TEST_ASSERT_EQUAL(ESP_FOC_SD_ABORT_SENSOR_STALE, s_ev_last[ESP_FOC_SD_EV_ABORT].abort);
    TEST_ASSERT_EQUAL(ESP_FOC_SD_STATE_FAULT, esp_foc_sensored_get_state(0));
    TEST_ASSERT_FALSE(s_m.enabled);
    esp_foc_sensored_status_t st;
    esp_foc_sensored_get_status(0, &st);
    TEST_ASSERT_EQUAL(ESP_FOC_SD_ABORT_SENSOR_STALE, st.abort);
    s_r.freeze = false;
    esp_foc_sleep_ms(200);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensored_run(0));
    TEST_ASSERT_EQUAL(ESP_FOC_SD_STATE_FAULT, esp_foc_sensored_get_state(0));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_clear_fault(0));
    TEST_ASSERT_EQUAL(1, s_ev_n[ESP_FOC_SD_EV_FAULT_CLEARED]);
    TEST_ASSERT_EQUAL(ESP_FOC_SD_STATE_IDLE, esp_foc_sensored_get_state(0));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensored_clear_fault(0));

    /* Coast down so the run finds a still rotor. */
    esp_foc_sleep_ms(1500);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_run(0));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_speed_ref_hz(0, 30.0f));
    esp_foc_sleep_ms(800);
    s_r.fail = true;
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SD_EV_ABORT, 2, 500));
    TEST_ASSERT_EQUAL(ESP_FOC_SD_ABORT_SENSOR_FAIL, s_ev_last[ESP_FOC_SD_EV_ABORT].abort);
    TEST_ASSERT_FALSE(s_m.enabled);
    s_r.fail = false;
    esp_foc_sensored_get_status(0, &st);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(CONFIG_ESP_FOC_SD_FAIL_MAX, st.fetch_fail);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_clear_fault(0));
    bench_reset();
}

TEST_CASE("sensored inverter fault latches until clear_fault", "[espFoC][sensored]")
{
    bench_up(ESP_FOC_SD_CONTROL_TORQUE);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_iq(0, 0.2f));
    esp_foc_sleep_ms(300);
    mock_pmsm_trip(&s_m, ESP_FOC_FAULT_ILIMIT);
    TEST_ASSERT_TRUE(wait_ev(ESP_FOC_SD_EV_FAULT, 1, 500));
    TEST_ASSERT_EQUAL(ESP_FOC_FAULT_ILIMIT, s_ev_last[ESP_FOC_SD_EV_FAULT].fault);
    TEST_ASSERT_EQUAL(ESP_FOC_SD_STATE_FAULT, esp_foc_sensored_get_state(0));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_sensored_run(0));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_clear_fault(0));
    TEST_ASSERT_EQUAL(1, s_m.clear_count);
    TEST_ASSERT_EQUAL(ESP_FOC_SD_STATE_IDLE, esp_foc_sensored_get_state(0));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_iq(0, 0.0f));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_run(0));
    TEST_ASSERT_EQUAL(ESP_FOC_SD_STATE_RUNNING, esp_foc_sensored_get_state(0));
    bench_down();
}

static float werr_rms_hz(uint32_t ms)
{
    esp_foc_sensored_window_t w;
    esp_foc_sensored_get_window(0, &w);
    esp_foc_sleep_ms(ms);
    esp_foc_sensored_get_window(0, &w);
    const float n = (float)((w.n > 0u) ? w.n : 1u);
    const float mean = w.werr_sum_hz / n;
    return sqrtf(fmaxf(w.werr_sq_hz2 / n - mean * mean, 0.0f));
}

TEST_CASE("sensored cogging learn converges and flattens the speed error",
          "[espFoC][sensored]")
{
    mock_pmsm_params_t p = k_plant;
    /* 0.075 A worth of position-locked torque at 24 per revolution. */
    p.t_cog = 0.075f * 1.5f * 7.0f * 0.005f;
    p.cog_n = 24;
    esp_foc_sensored_config_t c;
    plant_up(&p);
    cfg_mock(&c, ESP_FOC_SD_CONTROL_POSITION);
    c.cogging.sweep_hz = 1.0f;
    c.cogging.revs = 1.0f;
    c.cogging.skip_ms = 200u;
    c.cogging.rest_ms = 200u;
    c.speed_bw_min_hz = 8.0f;
    c.speed_bw_max_hz = 8.0f;
    bench_up_cfg(&c);

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_mode(0, ESP_FOC_SD_MODE_VELOCITY));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_speed_ref_hz(0, 7.0f));
    esp_foc_sleep_ms(1500);
    const float rms_raw = werr_rms_hz(2000);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_mode(0, ESP_FOC_SD_MODE_POSITION));
    esp_foc_sleep_ms(500);

    esp_foc_sensored_cogging_info_t info;
    memset(&info, 0, sizeof(info));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_cogging_learn(0, 60000u, &info));
    printf("sd cogging: passes %lu converged %d change %.4f A p2p %.4f A mapped %lu/%lu\n",
           (unsigned long)info.passes, info.converged ? 1 : 0, info.change_rms_a, info.p2p_a,
           (unsigned long)info.bins_mapped, (unsigned long)info.bins);
    TEST_ASSERT_TRUE(info.converged);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(c.cogging.passes_max, info.passes);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(2u, info.passes);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.15f, info.p2p_a);
    TEST_ASSERT_EQUAL(ESP_FOC_SD_STATE_RUNNING, esp_foc_sensored_get_state(0));
    TEST_ASSERT_GREATER_OR_EQUAL(2, s_ev_n[ESP_FOC_SD_EV_LEARN_PASS]);
    TEST_ASSERT_EQUAL(1, s_ev_n[ESP_FOC_SD_EV_LEARN_DONE]);
    esp_foc_sensored_status_t st;
    esp_foc_sensored_get_status(0, &st);
    TEST_ASSERT_TRUE(st.cogging_on);

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_mode(0, ESP_FOC_SD_MODE_VELOCITY));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_speed_ref_hz(0, 7.0f));
    esp_foc_sleep_ms(1500);
    const float rms_cog = werr_rms_hz(2000);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_cogging_enable(0, false));
    esp_foc_sleep_ms(500);
    const float rms_off = werr_rms_hz(2000);
    printf("sd cogging werr rms: raw %.3f with table %.3f table off %.3f Hz\n", rms_raw,
           rms_cog, rms_off);
    TEST_ASSERT_LESS_THAN_FLOAT(0.5f * rms_raw, rms_cog);
    TEST_ASSERT_LESS_THAN_FLOAT(0.5f * rms_off, rms_cog);
    TEST_ASSERT_EQUAL(0, s_ev_n[ESP_FOC_SD_EV_ABORT]);
    bench_down();
}

#if CONFIG_ESP_FOC_SD_MAX_AXES >= 2
TEST_CASE("sensored axes run independently", "[espFoC][sensored]")
{
    static mock_pmsm_t m1;
    static mock_pmsm_rotor_t r1;
    esp_foc_sensored_config_t c;

    plant_up(&k_plant);
    mock_pmsm_params_t p1 = k_plant;
    p1.theta0 = -1.1f;
    mock_pmsm_init(&m1, &p1);
    mock_pmsm_rotor_init(&r1, &m1, ENC_OFFSET_E);
    r1.extrapolate = true;

    cfg_mock(&c, ESP_FOC_SD_CONTROL_VELOCITY);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_init(&s_m.base, &s_r.base, &c));
    c.axis = 1;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_sensored_init(&m1.base, NULL, &c));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_init(&m1.base, &r1.base, &c));
    mock_pmsm_start(&s_m);
    mock_pmsm_start(&m1);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_run(0));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_run(1));
    TEST_ASSERT_EQUAL(1, s_ev_ax[0][ESP_FOC_SD_EV_RUNNING]);
    TEST_ASSERT_EQUAL(1, s_ev_ax[1][ESP_FOC_SD_EV_RUNNING]);

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_speed_ref_hz(0, 40.0f));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_set_speed_ref_hz(1, -40.0f));
    esp_foc_sleep_ms(1500);
    const win_t w0 = sample_axis(0, g_we_hz, 300);
    const win_t w1 = sample_axis(1, g_we_hz, 300);
    printf("sd 2 axes: we0 %.2f fe0 %.1f | we1 %.2f fe1 %.1f\n", w0.mean, mock_pmsm_fe_hz(&s_m),
           w1.mean, mock_pmsm_fe_hz(&m1));
    TEST_ASSERT_FLOAT_WITHIN(1.5f, 40.0f, w0.mean);
    TEST_ASSERT_FLOAT_WITHIN(1.5f, -40.0f, w1.mean);

    /* A trip on one axis leaves the other driving. */
    mock_pmsm_trip(&m1, ESP_FOC_FAULT_ILIMIT);
    for (uint32_t t = 0; (t < 500u) && (s_ev_ax[1][ESP_FOC_SD_EV_FAULT] < 1); t += 5u) {
        esp_foc_sleep_ms(5);
    }
    TEST_ASSERT_EQUAL(1, s_ev_ax[1][ESP_FOC_SD_EV_FAULT]);
    TEST_ASSERT_EQUAL(0, s_ev_ax[0][ESP_FOC_SD_EV_FAULT]);
    esp_foc_sleep_ms(300);
    TEST_ASSERT_EQUAL(ESP_FOC_SD_STATE_RUNNING, esp_foc_sensored_get_state(0));
    TEST_ASSERT_EQUAL(ESP_FOC_SD_STATE_FAULT, esp_foc_sensored_get_state(1));
    TEST_ASSERT_FLOAT_WITHIN(1.5f, 40.0f, sample_axis(0, g_we_hz, 300).mean);
    TEST_ASSERT_TRUE(s_m.enabled);

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_clear_fault(1));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_sensored_stop(0));
    TEST_ASSERT_FALSE(s_m.enabled);
    TEST_ASSERT_EQUAL(0, s_ev_n[ESP_FOC_SD_EV_ABORT]);
    bench_reset();
}
#endif

#endif /* CONFIG_ESP_FOC_STACK_SENSORED */
