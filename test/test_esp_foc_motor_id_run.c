/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * esp_foc_motor_id_run() against mock_pmsm_inverter: the whole identification
 * in both modes on a 7 pp PMSM (2 ohm, 1 mH, 5 mWb) with Coulomb + viscous
 * friction, plus its lifecycle (callbacks and bridge released on every
 * return, one run at a time) and its fault path.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"
#include "esp_err.h"
#include "espFoC/motor_control/esp_foc_motor_id.h"
#include "espFoC/osal/esp_foc_osal.h"
#include "mock_pmsm_inverter.h"

#define EV_COUNT (ESP_FOC_MOTOR_ID_EV_ANGLE_FIT + 1)

static mock_pmsm_t s_m;
static mock_pmsm_rotor_t s_rot;

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

typedef struct {
    int n[EV_COUNT];
    int phase_n[ESP_FOC_MOTOR_ID_FAIL + 1];
    esp_foc_motor_id_event_t last[EV_COUNT];
    bool reenter;
    esp_err_t reenter_err;
    int trip_after_phase;
    esp_foc_motor_id_phase_t trip_phase;
} ev_log_t;

static ev_log_t s_ev;

static void on_ev(void *ctx, const esp_foc_motor_id_event_t *e)
{
    ev_log_t *l = (ev_log_t *)ctx;
    l->n[e->ev]++;
    l->last[e->ev] = *e;
    if (e->ev == ESP_FOC_MOTOR_ID_EV_PHASE) {
        l->phase_n[e->phase]++;
        if (l->reenter) {
            l->reenter = false;
            esp_foc_motor_id_config_t c;
            esp_foc_motor_id_default_config(&c);
            c.pole_pairs = 7;
            c.vdc = 12.0f;
            esp_foc_motor_id_result_t r;
            l->reenter_err = esp_foc_motor_id_run(&s_m.base, NULL, &c, &r);
        }
        if ((l->trip_after_phase > 0) && (e->phase == l->trip_phase)) {
            l->trip_after_phase = 0;
            mock_pmsm_trip(&s_m, ESP_FOC_FAULT_ILIMIT);
        }
    }
}

static void fast_config(esp_foc_motor_id_config_t *c)
{
    esp_foc_motor_id_default_config(c);
    c->pole_pairs = k_plant.pp;
    c->vdc = k_plant.vdc;
    c->probe_periods = 16u;
    c->tries = 2u;
    c->settle_ms = 50u;
    c->coast_ms = 50u;
    c->retry_ms = 50u;
    c->i_abort_a = 2.0f;
    c->sensored.fetch_hz = 1000u;
    c->sensored.pll_bw_hz = 60.0f;
    c->sensored.probe_base_hz = 40;
    c->sensored.probe_step_ms = 2u;
    c->sensored.probe_settle_ms = 400u;
    c->sensored.probe_avg_ms = 200u;
    c->on_event = on_ev;
    c->ctx = &s_ev;
}

static void plant_up(float offset_e)
{
    memset(&s_ev, 0, sizeof(s_ev));
    mock_pmsm_init(&s_m, &k_plant);
    mock_pmsm_rotor_init(&s_rot, &s_m, offset_e);
    mock_pmsm_start(&s_m);
}

/*
 * The flux leg reads the EMF on the q axis of the I-f frame only. The shaft
 * lags that frame by the load angle it needs to carry the friction, so the
 * leg measures psi * cos(delta), not psi.
 */
static float flux_leg_psi(const esp_foc_motor_id_config_t *c)
{
    const float w_m = 6.2831853f * (float)c->flux_hz / (float)k_plant.pp;
    const float t_load = k_plant.b_visc * w_m + k_plant.t_coul;
    const float sin_d = t_load / (1.5f * (float)k_plant.pp * k_plant.psi * c->i_flux_a);
    return k_plant.psi * sqrtf(1.0f - sin_d * sin_d);
}

/* The sequence runs on the block's own task: a caller this small must do. */
#define SMALL_CALLER_STACK 2048

typedef struct {
    esp_foc_motor_id_config_t c;
    esp_foc_motor_id_result_t r;
    esp_err_t err;
    volatile bool done;
} small_run_t;

static small_run_t s_small;

static void small_caller(void *arg)
{
    small_run_t *s = (small_run_t *)arg;
    s->err = esp_foc_motor_id_run(&s_m.base, NULL, &s->c, &s->r);
    s->done = true;
    esp_foc_task_delete_self();
}

static void assert_released(void)
{
    TEST_ASSERT_NULL(s_m.pwm_cb);
    TEST_ASSERT_NULL(s_m.dma_cb);
    TEST_ASSERT_NULL(s_m.fault_cb);
    TEST_ASSERT_FALSE(s_m.enabled);
    TEST_ASSERT_EQUAL(s_m.enable_count, s_m.disable_count);
    TEST_ASSERT_EQUAL(0, s_m.idle_disables);
}

TEST_CASE("motor_id run rejects bad arguments", "[espFoC][motor_id_run]")
{
    plant_up(0.0f);
    esp_foc_motor_id_config_t c;
    esp_foc_motor_id_result_t r;
    fast_config(&c);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_motor_id_run(NULL, NULL, &c, &r));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_motor_id_run(&s_m.base, NULL, NULL, &r));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_motor_id_run(&s_m.base, NULL, &c, NULL));

    c.pole_pairs = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_motor_id_run(&s_m.base, NULL, &c, &r));
    fast_config(&c);
    c.vdc = 0.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_motor_id_run(&s_m.base, NULL, &c, &r));
    fast_config(&c);
    c.tries = 0u;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_motor_id_run(&s_m.base, NULL, &c, &r));

    /* The rotor rate must divide the carrier, or the fetch drifts against it. */
    fast_config(&c);
    c.sensored.fetch_hz = 1500u;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_motor_id_run(&s_m.base, &s_rot.base, &c, &r));
    c.sensored.fetch_hz = 0u;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_motor_id_run(&s_m.base, &s_rot.base, &c, &r));

    TEST_ASSERT_EQUAL(0, s_m.enable_count);
    TEST_ASSERT_NULL(s_m.pwm_cb);
    mock_pmsm_stop();
}

TEST_CASE("motor_id sensorless identifies R, L and psi and releases the inverter",
          "[espFoC][motor_id_run]")
{
    plant_up(0.0f);
    small_run_t *s = &s_small;
    memset(s, 0, sizeof(*s));
    fast_config(&s->c);
    s_ev.reenter = true;
    const uint64_t t0 = esp_foc_now_us();
    TEST_ASSERT_EQUAL(0, esp_foc_task_spawn(small_caller, s, "id_caller", SMALL_CALLER_STACK,
                                            5, NULL));
    while (!s->done) {
        esp_foc_sleep_ms(10);
    }
    const esp_foc_motor_id_config_t c = s->c;
    const esp_foc_motor_id_result_t r = s->r;
    const esp_err_t err = s->err;
    const float psi_leg = flux_leg_psi(&c);
    printf("motor_id sensorless err=%d %.1f s R=%.3f L=%.1f uH psi=%.2f mWb (leg %.2f) "
           "kp=%.4f ki=%.1f mask=0x%lx failed_at=%s\n",
           (int)err, (double)(esp_foc_now_us() - t0) * 1e-6, (double)r.r_loop_ohm,
           (double)(r.ls_h * 1e6f), (double)(r.psi_f_wb * 1e3f), (double)(psi_leg * 1e3f),
           (double)r.kp, (double)r.ki, (unsigned long)r.valid_mask,
           esp_foc_motor_id_phase_name(r.failed_at));
    mock_pmsm_stop();

    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, s_ev.reenter_err);
    TEST_ASSERT_FLOAT_WITHIN(0.15f * k_plant.rs, k_plant.rs, r.r_loop_ohm);
    TEST_ASSERT_FLOAT_WITHIN(0.20f * k_plant.ls, k_plant.ls, r.ls_h);
    TEST_ASSERT_FLOAT_WITHIN(0.20f * psi_leg, psi_leg, r.psi_f_wb);
    TEST_ASSERT_EQUAL(k_plant.pp, r.pole_pairs);
    TEST_ASSERT_TRUE((r.valid_mask & ESP_FOC_MOTOR_ID_VALID_GAINS) != 0u);
    TEST_ASSERT_TRUE((r.valid_mask & ESP_FOC_MOTOR_ID_VALID_PSI_F) != 0u);
    TEST_ASSERT_EQUAL(0u, r.valid_mask & (ESP_FOC_MOTOR_ID_VALID_DIR | ESP_FOC_MOTOR_ID_VALID_K |
                                          ESP_FOC_MOTOR_ID_VALID_PARK));
    TEST_ASSERT_TRUE(r.kp > 0.0f && r.ki > 0.0f);
    TEST_ASSERT_EQUAL(0, s_ev.phase_n[ESP_FOC_MOTOR_ID_MECH]);
    TEST_ASSERT_EQUAL(ESP_FOC_MOTOR_ID_DONE, s_ev.last[ESP_FOC_MOTOR_ID_EV_PHASE].phase);
    TEST_ASSERT_EQUAL(2, s_ev.n[ESP_FOC_MOTOR_ID_EV_ATTEMPT]);
    assert_released();
}

TEST_CASE("motor_id sensored fits the encoder offset and measures K",
          "[espFoC][motor_id_run]")
{
    const float offset = 0.35f;
    plant_up(offset);
    esp_foc_motor_id_config_t c;
    esp_foc_motor_id_result_t r;
    fast_config(&c);
    const uint64_t t0 = esp_foc_now_us();
    const esp_err_t err = esp_foc_motor_id_run(&s_m.base, &s_rot.base, &c, &r);
    const float k_true = 1.5f * (float)(k_plant.pp * k_plant.pp) * k_plant.psi / k_plant.j;
    printf("motor_id sensored err=%d %.1f s R=%.3f L=%.1f uH psi=%.2f mWb dir=%.2f Hz "
           "K=%.0f (raw %.0f, true %.0f) J=%.2e d0=%+.3f rad tau=%+.1f us rms=%.3f "
           "mask=0x%lx failed_at=%s fetches=%d\n",
           (int)err, (double)(esp_foc_now_us() - t0) * 1e-6, (double)r.r_loop_ohm,
           (double)(r.ls_h * 1e6f), (double)(r.psi_f_wb * 1e3f), (double)r.dir_fm_hz,
           (double)r.k_rad_s2_per_a, (double)r.k_raw_rad_s2_per_a, (double)k_true,
           (double)r.j_kgm2, (double)r.park_offset_rad, (double)(r.park_lead_s * 1e6f),
           (double)r.park_fit_rms_rad, (unsigned long)r.valid_mask,
           esp_foc_motor_id_phase_name(r.failed_at), s_rot.n_fetch);
    mock_pmsm_stop();

    TEST_ASSERT_EQUAL(ESP_OK, err);
    const uint32_t want = ESP_FOC_MOTOR_ID_VALID_GAINS | ESP_FOC_MOTOR_ID_VALID_PSI_F |
                          ESP_FOC_MOTOR_ID_VALID_DIR | ESP_FOC_MOTOR_ID_VALID_K |
                          ESP_FOC_MOTOR_ID_VALID_J | ESP_FOC_MOTOR_ID_VALID_PARK;
    TEST_ASSERT_EQUAL_HEX32(want, r.valid_mask & want);
    TEST_ASSERT_TRUE(r.dir_fm_hz > c.sensored.fm_min_hz);
    /* The encoder reads θ + offset, so Park must add -offset. */
    TEST_ASSERT_FLOAT_WITHIN(0.10f, -offset, r.park_offset_rad);
    TEST_ASSERT_TRUE(fabsf(r.park_lead_s) < c.sensored.tau_max_s);
    TEST_ASSERT_FLOAT_WITHIN(0.35f * k_true, k_true, r.k_rad_s2_per_a);
    TEST_ASSERT_FLOAT_WITHIN(0.50f * k_plant.j, k_plant.j, r.j_kgm2);
    TEST_ASSERT_EQUAL(2, s_ev.phase_n[ESP_FOC_MOTOR_ID_MECH]);
    TEST_ASSERT_EQUAL(2, s_ev.n[ESP_FOC_MOTOR_ID_EV_ANGLE_FIT]);
    TEST_ASSERT_EQUAL(16, s_ev.n[ESP_FOC_MOTOR_ID_EV_ANGLE_POINT]);
    /* The residual pass is what the offset left: it must have shrunk. */
    TEST_ASSERT_TRUE(r.park_fit_rms_rad < 0.10f);
    TEST_ASSERT_TRUE(s_rot.n_fetch > 1000);
    assert_released();
}

TEST_CASE("motor_id sensored without angle compensation stops after the raw K",
          "[espFoC][motor_id_run]")
{
    plant_up(0.0f);
    esp_foc_motor_id_config_t c;
    esp_foc_motor_id_result_t r;
    fast_config(&c);
    c.sensored.angle_comp = false;
    const esp_err_t err = esp_foc_motor_id_run(&s_m.base, &s_rot.base, &c, &r);
    mock_pmsm_stop();

    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_TRUE((r.valid_mask & ESP_FOC_MOTOR_ID_VALID_K) != 0u);
    TEST_ASSERT_EQUAL(0u, r.valid_mask & ESP_FOC_MOTOR_ID_VALID_PARK);
    TEST_ASSERT_EQUAL_FLOAT(r.k_raw_rad_s2_per_a, r.k_rad_s2_per_a);
    TEST_ASSERT_EQUAL(1, s_ev.phase_n[ESP_FOC_MOTOR_ID_MECH]);
    TEST_ASSERT_EQUAL(0, s_ev.n[ESP_FOC_MOTOR_ID_EV_ANGLE_POINT]);
    assert_released();
}

TEST_CASE("motor_id fails on a trip and still releases the inverter",
          "[espFoC][motor_id_run]")
{
    plant_up(0.0f);
    esp_foc_motor_id_config_t c;
    esp_foc_motor_id_result_t r;
    fast_config(&c);
    c.tries = 1u;
    s_ev.trip_after_phase = 1;
    s_ev.trip_phase = ESP_FOC_MOTOR_ID_ROVERL_COARSE;
    const esp_err_t err = esp_foc_motor_id_run(&s_m.base, &s_rot.base, &c, &r);
    mock_pmsm_stop();

    TEST_ASSERT_NOT_EQUAL(ESP_OK, err);
    TEST_ASSERT_NOT_EQUAL(ESP_FOC_MOTOR_ID_IDLE, r.failed_at);
    TEST_ASSERT_EQUAL(0u, r.valid_mask & ESP_FOC_MOTOR_ID_VALID_GAINS);
    TEST_ASSERT_EQUAL(ESP_FOC_MOTOR_ID_FAIL, s_ev.last[ESP_FOC_MOTOR_ID_EV_PHASE].phase);
    TEST_ASSERT_NULL(s_m.pwm_cb);
    TEST_ASSERT_NULL(s_m.fault_cb);
    TEST_ASSERT_FALSE(s_m.enabled);

    /* Released means a second run may follow at once. */
    plant_up(0.0f);
    fast_config(&c);
    c.sensored.angle_comp = false;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_run(&s_m.base, NULL, &c, &r));
    mock_pmsm_stop();
    assert_released();
}
