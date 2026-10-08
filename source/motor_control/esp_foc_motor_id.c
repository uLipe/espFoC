/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#include <math.h>
#include <string.h>

#include "sdkconfig.h"
#include "espFoC/motor_control/esp_foc_motor_id.h"
#include "espFoC/motor_control/esp_foc_rotor_pll.h"
#include "espFoC/osal/esp_foc_osal.h"
#include "espFoC/utils/esp_foc_angle.h"
#include "espFoC/utils/esp_foc_clarke.h"
#include "espFoC/utils/esp_foc_park.h"
#include "espFoC/utils/esp_foc_pid.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/esp_foc_svm.h"
#include "espFoC/utils/esp_foc_trig.h"
#include "espFoC/utils/esp_foc_vlim.h"
#include "esp_foc_mech_id.h"
#include "esp_foc_motor_id_probe.h"
#include "esp_foc_motor_id_seq.h"

#define MID_TWO_PI 6.28318531f
#define MID_DT_MS 20u
#define MID_ARM_SETTLE_MS 50u
#define MID_ROTOR_STACK 4096
#define MID_ROTOR_WAIT_MS 50u
#define MID_TASK_TIMEOUT_MS 500u
#define MID_JOIN_POLL_MS 10u
#define MID_DIR_MEAS_MS 500u
#define MID_STOP_MS 1000u
#define MID_STILL_CALM_MS 300u
#define MID_STILL_TIMEOUT_MS 8000u
#define MID_BAND_TIMEOUT_MS 8000u
/* Above the band the nudge drops to this share: holds the shaft against drag
 * without accelerating it further. */
#define MID_NUDGE_HI_FRAC 0.4f
#define MID_ANGLE_PTS 8
/*
 * Ripple sums are of the deviation from the first sample of the window,
 * shifted down before squaring: a window drifting to a stop at ~300 Hz elec
 * otherwise wraps int64 and reads back as a residual of exactly zero.
 */
#define MID_RIP_SHIFT 8
#define MID_RIP_MIN_N 64u
/* Placeholder until TUNE_I applies the designed gains; no current loop runs
 * before that. */
#define MID_PI_SEED_KP 0.1f
#define MID_PI_SEED_KI 100.0f

/*
 * Positional speed PI, gains in Q32. P never enters the state and I only
 * integrates when it does not push further into saturation, so Kp·noise at
 * the clamp is not integrated into a drift.
 */
typedef struct {
    int64_t kp_q32;
    int64_t ki_ts_q32;
    int64_t i_q32;
    int64_t i_lim_q32;
    q16_t u_lim;
} mid_hold_t;

typedef struct {
    esp_foc_inverter_t *inv;
    esp_foc_rotor_sensor_t *rotor;
    esp_foc_motor_id_probe_t probe;
    esp_foc_pid_t pi_d;
    esp_foc_pid_t pi_q;
    mid_hold_t hold;
    esp_foc_rotor_pll_t pll;

    volatile bool run;
    volatile bool on_rotor;
    volatile bool hold_on;
    volatile bool fresh;
    volatile bool fault_seen;
    volatile bool overspeed;
    volatile bool quit;
    volatile bool rotor_alive;
    volatile bool meas_on;
    volatile bool acc_on;
    volatile q16_t theta_ol;
    volatile q16_t dth;
    volatile q16_t id_ref;
    volatile q16_t iq_ref;
    volatile q16_t id;
    volatile q16_t iq;
    volatile q16_t vd;
    volatile q16_t vq;
    volatile q16_t w_e;
    volatile q16_t w_ref;
    volatile q16_t w_m_meas;
    volatile q16_t th_m_acc;
    volatile q16_t th_off;
    volatile int32_t tau_q32;
    int32_t vlead_q32;
    uint32_t div;
    uint32_t slow_div;
    volatile esp_foc_event_handle_t rotor_ev;
    q16_t fetch_hz_q;
    q16_t w_overspeed;
    q16_t th_m_prev;
    bool th_have;

    volatile uint32_t acc_n;
    volatile int64_t acc_vd;
    volatile int64_t acc_vq;
    volatile int64_t acc_id;
    volatile int64_t acc_iq;
    volatile int64_t acc_w;

    volatile uint32_t rip_n;
    volatile uint32_t rip_budget;
    q16_t rip_seed;
    int64_t rip_sum;
    int64_t rip_sq;
    int64_t rip_ix;

    esp_foc_motor_id_config_t cfg;
    esp_foc_motor_id_seq_t seq;
    esp_foc_motor_id_phase_t phase;
    uint32_t pwm_hz;
    q16_t vdc_q;
    bool bridge_on;

    esp_foc_motor_id_result_t *out;
    esp_err_t err;
    volatile bool worker_alive;
} mid_t;

static mid_t s_mid;
static volatile bool s_busy;

static void emit(mid_t *m, esp_foc_motor_id_ev_t ev, uint8_t pass, uint8_t attempt,
                 esp_err_t err, float v0, float v1, float v2, float v3,
                 const esp_foc_motor_id_result_t *r)
{
    if (m->cfg.on_event == NULL) {
        return;
    }
    const esp_foc_motor_id_event_t e = {
        .ev = ev,
        .phase = m->phase,
        .pass = pass,
        .attempt = attempt,
        .err = err,
        .v = { v0, v1, v2, v3 },
        .result = r,
    };
    m->cfg.on_event(m->cfg.ctx, &e);
}

static void go(mid_t *m, esp_foc_motor_id_phase_t ph)
{
    m->phase = ph;
    emit(m, ESP_FOC_MOTOR_ID_EV_PHASE, 0, 0, ESP_OK, 0.0f, 0.0f, 0.0f, 0.0f, NULL);
}

static q16_t hold_step(mid_t *m)
{
    const q16_t e = q16_sub(m->w_ref, m->w_e);
    const int64_t kp = m->hold.kp_q32;
    const int64_t ki_ts = m->hold.ki_ts_q32;
    const int64_t i_old = m->hold.i_q32;
    const int64_t i_lim = m->hold.i_lim_q32;
    const q16_t u_lim = m->hold.u_lim;

    const int64_t inc = (ki_ts * (int64_t)e) >> 16;
    int64_t i_new = i_old + inc;
    if (i_new > i_lim) {
        i_new = i_lim;
    } else if (i_new < -i_lim) {
        i_new = -i_lim;
    }
    int64_t u = ((kp * (int64_t)e) >> 32) + (i_new >> 16);
    bool push_out = false;
    if (u > u_lim) {
        u = u_lim;
        push_out = (inc > 0);
    } else if (u < -u_lim) {
        u = -u_lim;
        push_out = (inc < 0);
    }
    if (!push_out) {
        m->hold.i_q32 = i_new;
    }
    return (q16_t)u;
}

/* A probe in flight owns the duties and must also see a fault, or its window
 * never closes and the trip surfaces as a timeout states later. */
static void mid_tez(void *arg)
{
    mid_t *m = (mid_t *)arg;
    esp_foc_inverter_t *inv = m->inv;
    esp_foc_rotor_sensor_t *rotor = m->rotor;

    if (rotor != NULL) {
        esp_foc_rotor_sensor_step(rotor);
        const uint32_t div = m->div + 1u;
        if (div >= m->slow_div) {
            m->div = 0;
            esp_foc_event_handle_t ev = m->rotor_ev;
            if (ev != NULL) {
                esp_foc_event_post_auto(ev);
            }
        } else {
            m->div = div;
        }
    }

    if (esp_foc_motor_id_probe_isr(&m->probe)) {
        return;
    }
    if (!m->run || inv->is_faulted(inv)) {
        return;
    }

    const bool on_rotor = m->on_rotor;
    const q16_t idr = m->id_ref;
    q16_t iqr = m->iq_ref;
    const q16_t w_e = m->w_e;
    const q16_t th_off = m->th_off;
    const int32_t tau_q32 = m->tau_q32;
    const int32_t vlead_q32 = m->vlead_q32;
    if (m->hold_on && m->fresh) {
        m->fresh = false;
        iqr = hold_step(m);
        m->iq_ref = iqr;
    }

    q16_t th;
    q16_t vlead = 0;
    if (on_rotor) {
        esp_foc_rotor_state_t snap;
        esp_foc_rotor_sensor_snapshot(rotor, &snap);
        const q16_t lead = (q16_t)(((int64_t)w_e * tau_q32) >> 32);
        th = q16_wrap_pi(q16_add(snap.theta_e, q16_add(th_off, lead)));
        vlead = (q16_t)(((int64_t)w_e * vlead_q32) >> 32);
    } else {
        th = q16_wrap_pi(q16_add(m->theta_ol, m->dth));
    }

    q16_t iu;
    q16_t iv;
    q16_t iw;
    inv->fetch_currents(inv, &iu, &iv, &iw);
    q16_t ia;
    q16_t ib;
    esp_foc_clarke(iu, iv, iw, &ia, &ib);
    q16_t s;
    q16_t c;
    esp_foc_sincos(th, &s, &c);
    q16_t id;
    q16_t iq;
    esp_foc_park(s, c, ia, ib, &id, &iq);
    q16_t vd = esp_foc_pid_update(&m->pi_d, idr, id);
    q16_t vq = esp_foc_pid_update(&m->pi_q, iqr, iq);
    esp_foc_vlim_dq(&vd, &vq, Q16_INV_SQRT3);
    esp_foc_pid_set_applied(&m->pi_d, vd);
    esp_foc_pid_set_applied(&m->pi_q, vq);

    /*
     * Duties computed now are averaged over the period centred 1.5 Ts later,
     * so inverse Park leads by that much. Rotated by a cubic instead of a
     * second CORDIC, which does not fit the period; |vlead| stays under
     * 0.3 rad where the cubic is < 1e-4 off.
     */
    if (vlead != 0) {
        const q16_t d2 = q16_mul(vlead, vlead);
        const q16_t cd = Q16_ONE - (d2 >> 1);
        const q16_t sd = vlead - q16_mul(q16_mul(d2, vlead), (q16_t)10923);
        const q16_t s0 = s;
        s = q16_add(q16_mul(s0, cd), q16_mul(c, sd));
        c = q16_sub(q16_mul(c, cd), q16_mul(s0, sd));
    }
    q16_t a;
    q16_t b;
    q16_t du;
    q16_t dv;
    q16_t dw;
    esp_foc_inv_park(s, c, vd, vq, &a, &b);
    esp_foc_svm(a, b, &du, &dv, &dw);
    inv->set_duties(inv, du, dv, dw);

    if (m->acc_on) {
        m->acc_vd += vd;
        m->acc_vq += vq;
        m->acc_id += id;
        m->acc_iq += iq;
        m->acc_w += w_e;
        m->acc_n++;
    }
    if (!on_rotor) {
        m->theta_ol = th;
    }
    m->id = id;
    m->iq = iq;
    m->vd = vd;
    m->vq = vq;
}

static void mid_fault(void *arg, esp_foc_fault_reason_t reason)
{
    mid_t *m = (mid_t *)arg;
    (void)reason;
    m->run = false;
    m->fault_seen = true;
}

static void rotor_sample(mid_t *m)
{
    esp_foc_rotor_pll_update(&m->pll, m->rotor);
    const q16_t w = esp_foc_rotor_pll_get_omega(&m->pll);

    const uint32_t rn = m->rip_n;
    if (rn < m->rip_budget) {
        if (rn == 0u) {
            m->rip_seed = w;
        }
        const int64_t x = (int64_t)q16_sub(w, m->rip_seed) >> MID_RIP_SHIFT;
        m->rip_sum += x;
        m->rip_sq += x * x;
        m->rip_ix += (int64_t)rn * x;
        m->rip_n = rn + 1u;
    }

    esp_foc_rotor_state_t st;
    esp_foc_rotor_sensor_snapshot(m->rotor, &st);
    const q16_t dth_m = q16_angle_delta(m->th_m_prev, st.theta_m);
    m->th_m_prev = st.theta_m;
    if (m->th_have) {
        m->w_m_meas = q16_mul(dth_m, m->fetch_hz_q);
        if (m->meas_on) {
            m->th_m_acc = q16_add(m->th_m_acc, dth_m);
        }
    }
    m->th_have = true;

    const q16_t wabs = (w < 0) ? q16_neg(w) : w;
    if (wabs > m->w_overspeed) {
        m->overspeed = true;
    }
    m->w_e = w;
    m->fresh = true;
}

static void mid_rotor_task(void *arg)
{
    mid_t *m = (mid_t *)arg;
    m->rotor_ev = esp_foc_event_handle_self();
    while (!m->quit) {
        if (!esp_foc_event_wait_ms(MID_ROTOR_WAIT_MS)) {
            continue;
        }
        esp_foc_event_clear();
        if (m->quit) {
            break;
        }
        (void)esp_foc_rotor_sensor_fetch(m->rotor);
        rotor_sample(m);
    }
    m->rotor_ev = NULL;
    m->rotor_alive = false;
    esp_foc_task_delete_self();
}

static void drive_idle(mid_t *m)
{
    m->run = false;
    m->hold_on = false;
    m->id_ref = 0;
    m->iq_ref = 0;
    m->dth = 0;
    m->inv->set_duties(m->inv, Q16_HALF, Q16_HALF, Q16_HALF);
}

/* Only undoes our own enable: disable() on an idle inverter reruns the whole
 * stop sequence of the converter and the PWM. */
static void bridge_off(mid_t *m)
{
    drive_idle(m);
    if (!m->bridge_on) {
        return;
    }
    m->inv->disable(m->inv);
    m->bridge_on = false;
}

static void coast(mid_t *m)
{
    bridge_off(m);
    esp_foc_sleep_ms(m->cfg.coast_ms);
}

static esp_err_t bridge_arm(mid_t *m)
{
    esp_foc_inverter_t *inv = m->inv;
    drive_idle(m);
    m->on_rotor = false;
    m->w_ref = 0;
    if (inv->is_faulted(inv)) {
        (void)inv->clear_fault(inv);
    }
    esp_foc_critical_enter();
    esp_foc_pid_reset(&m->pi_d);
    esp_foc_pid_reset(&m->pi_q);
    esp_foc_critical_leave();
    m->theta_ol = 0;
    esp_err_t err = inv->enable(inv);
    if (err != ESP_OK) {
        return err;
    }
    m->bridge_on = true;
    inv->set_duties(inv, Q16_HALF, Q16_HALF, Q16_HALF);
    /* Re-zeroed per attempt so retries are independent. */
    inv->calibrate_currents(inv, m->cfg.cal_rounds);
    inv->set_duties(inv, Q16_HALF, Q16_HALF, Q16_HALF);
    esp_foc_sleep_ms(MID_ARM_SETTLE_MS);
    m->fault_seen = false;
    m->overspeed = false;
    if (inv->is_faulted(inv)) {
        bridge_off(m);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static bool aborted(mid_t *m)
{
    return m->fault_seen || m->overspeed || m->inv->is_faulted(m->inv);
}

static void op_set_theta(void *ctx, q16_t theta)
{
    (void)ctx;
    s_mid.theta_ol = theta;
}

/* Only used to park: every voltage excitation goes through the probes, which
 * drive the duties themselves. */
static void op_set_vdq(void *ctx, q16_t vd, q16_t vq)
{
    (void)ctx;
    (void)vd;
    (void)vq;
    drive_idle(&s_mid);
}

static void op_set_idq(void *ctx, q16_t id, q16_t iq)
{
    (void)ctx;
    if ((id == 0) && (iq == 0)) {
        drive_idle(&s_mid);
        return;
    }
    s_mid.id_ref = id;
    s_mid.iq_ref = iq;
    s_mid.run = true;
}

static void op_set_fe(void *ctx, q16_t fe_hz)
{
    (void)ctx;
    s_mid.dth = (q16_t)(((int64_t)fe_hz * (int64_t)Q16_TWO_PI) /
                        ((int64_t)s_mid.pwm_hz << 16));
}

static void op_fetch_dq(void *ctx, q16_t *vd, q16_t *vq, q16_t *id, q16_t *iq)
{
    (void)ctx;
    *vd = q16_mul(s_mid.vd, s_mid.vdc_q);
    *vq = q16_mul(s_mid.vq, s_mid.vdc_q);
    *id = s_mid.id;
    *iq = s_mid.iq;
}

static void set_gains(mid_t *m, float kp, float ki)
{
    esp_foc_critical_enter();
    (void)esp_foc_pid_set_kp(&m->pi_d, kp);
    (void)esp_foc_pid_set_ki(&m->pi_d, ki);
    (void)esp_foc_pid_set_kp(&m->pi_q, kp);
    (void)esp_foc_pid_set_ki(&m->pi_q, ki);
    esp_foc_critical_leave();
}

static void op_apply_gains(void *ctx, float kp, float ki)
{
    (void)ctx;
    set_gains(&s_mid, kp, ki);
}

static void op_sleep(void *ctx, uint32_t ms)
{
    (void)ctx;
    esp_foc_sleep_ms(ms);
}

static bool op_faulted(void *ctx)
{
    (void)ctx;
    return s_mid.inv->is_faulted(s_mid.inv);
}

/* The sequence ends once per electrical leg; that outcome is the ATTEMPT
 * event, and DONE/FAIL belong to the whole run. */
static void op_on_phase(void *ctx, esp_foc_motor_id_phase_t ph)
{
    (void)ctx;
    if ((ph == ESP_FOC_MOTOR_ID_DONE) || (ph == ESP_FOC_MOTOR_ID_FAIL)) {
        return;
    }
    go(&s_mid, ph);
}

static bool op_fetch_rotor(void *ctx, q16_t *th_m, q16_t *w_m)
{
    (void)ctx;
    esp_foc_rotor_state_t st;
    esp_foc_rotor_sensor_snapshot(s_mid.rotor, &st);
    *th_m = st.theta_m;
    *w_m = s_mid.w_m_meas;
    return st.valid;
}

void esp_foc_motor_id_default_config(esp_foc_motor_id_config_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));
    cfg->i_probe_target_a = (float)CONFIG_ESP_FOC_MOTOR_ID_I_TARGET_MA * 1e-3f;
    cfg->i_probe_max_a = (float)CONFIG_ESP_FOC_MOTOR_ID_I_PROBE_MAX_MA * 1e-3f;
    cfg->i_abort_a = (float)CONFIG_ESP_FOC_MOTOR_ID_I_ABORT_MA * 1e-3f;
    cfg->v_deadzone_v = (float)CONFIG_ESP_FOC_MOTOR_ID_V_DEADZONE_MV * 1e-3f;
    cfg->sym_limit_permil = 600;
    cfg->probe_periods = CONFIG_ESP_FOC_MOTOR_ID_PROBE_PERIODS;
    cfg->settle_periods = 8u;
    cfg->i_bw_hz = (float)CONFIG_ESP_FOC_MOTOR_ID_I_BW_HZ;
    cfg->tune_backoff = 0.5f;
    cfg->i_flux_a = (float)CONFIG_ESP_FOC_MOTOR_ID_I_FLUX_MA * 1e-3f;
    cfg->flux_hz = CONFIG_ESP_FOC_MOTOR_ID_FLUX_HZ;
    cfg->r_min_ohm = 0.20f;
    cfg->r_max_ohm = 40.0f;
    cfg->l_min_h = 20e-6f;
    cfg->l_max_h = 50e-3f;
    cfg->psi_min_wb = 200e-6f;
    cfg->psi_max_wb = 50e-3f;
    cfg->tries = CONFIG_ESP_FOC_MOTOR_ID_TRIES;
    cfg->settle_ms = 1500u;
    cfg->coast_ms = 2000u;
    cfg->retry_ms = 4000u;
    cfg->cal_rounds = 32;

    cfg->sensored.fetch_hz = CONFIG_ESP_FOC_MOTOR_ID_FETCH_HZ;
    cfg->sensored.pll_bw_hz = 180.0f;
    cfg->sensored.pll_zeta = 1.0f;
    cfg->sensored.i_bw_hz = (float)CONFIG_ESP_FOC_MOTOR_ID_SPIN_I_BW_HZ;
    cfg->sensored.iq_dir_a = (float)CONFIG_ESP_FOC_MOTOR_ID_IQ_DIR_MA * 1e-3f;
    cfg->sensored.dir_ramp_ms = 800u;
    cfg->sensored.fm_min_hz = 2.0f;
    cfg->sensored.i_max_a = (float)CONFIG_ESP_FOC_MOTOR_ID_I_MAX_MA * 1e-3f;
    /* K is against measured iq; with the converter locked to TEZ a small
     * outrunner reads 67-90 k rad/s^2/A. */
    cfg->sensored.k_max = 200000.0f;
    cfg->sensored.k_fallback = (float)CONFIG_ESP_FOC_MOTOR_ID_K_FALLBACK;
    cfg->sensored.band_lo_hz = 40.0f;
    cfg->sensored.band_hi_hz = 180.0f;
    cfg->sensored.nudge_a = 0.20f;
    cfg->sensored.still_hz = 3.0f;
    cfg->sensored.hold_zeta = 1.15f;
    cfg->sensored.hold_bw_frac = 0.0154f;
    cfg->sensored.hold_bw_min_hz = 4.0f;
    cfg->sensored.hold_bw_max_hz = 8.0f;
    cfg->sensored.fb_iq_budget_a = 0.10f;
    cfg->sensored.pll_sep_min = 5.0f;
    cfg->sensored.ripple_ms = 400u;
#if CONFIG_ESP_FOC_MOTOR_ID_ANGLE_COMP
    cfg->sensored.angle_comp = true;
#endif
    cfg->sensored.probe_base_hz = 80;
    cfg->sensored.probe_step_ms = 10u;
    cfg->sensored.probe_settle_ms = 1200u;
    cfg->sensored.probe_avg_ms = 500u;
    cfg->sensored.d0_max_rad = 60.0f * MID_TWO_PI / 360.0f;
    cfg->sensored.tau_max_s = 2e-3f;
    cfg->sensored.pwm_delay_ts = 1.5f;
    cfg->sensored.overspeed_hz = (float)CONFIG_ESP_FOC_MOTOR_ID_OVERSPEED_HZ;
}

const char *esp_foc_motor_id_phase_name(esp_foc_motor_id_phase_t ph)
{
    switch (ph) {
    case ESP_FOC_MOTOR_ID_IDLE: return "idle";
    case ESP_FOC_MOTOR_ID_BIAS: return "bias";
    case ESP_FOC_MOTOR_ID_TERMINAL: return "terminal";
    case ESP_FOC_MOTOR_ID_ROVERL_COARSE: return "roverl_coarse";
    case ESP_FOC_MOTOR_ID_LAGCAL: return "lagcal";
    case ESP_FOC_MOTOR_ID_ROVERL_FINE: return "roverl_fine";
    case ESP_FOC_MOTOR_ID_TUNE_I: return "tune_i";
    case ESP_FOC_MOTOR_ID_RS: return "rs";
    case ESP_FOC_MOTOR_ID_RAMPUP: return "rampup";
    case ESP_FOC_MOTOR_ID_RATED_FLUX: return "rated_flux";
    case ESP_FOC_MOTOR_ID_RAMPDOWN: return "rampdown";
    case ESP_FOC_MOTOR_ID_DIRECTION: return "direction";
    case ESP_FOC_MOTOR_ID_MECH: return "mech";
    case ESP_FOC_MOTOR_ID_PARK_ANGLE: return "park_angle";
    case ESP_FOC_MOTOR_ID_DONE: return "done";
    case ESP_FOC_MOTOR_ID_FAIL: return "fail";
    default: return "?";
    }
}

static bool plant_plausible(const mid_t *m, const esp_foc_motor_id_result_t *r, bool flux)
{
    const esp_foc_motor_id_config_t *c = &m->cfg;
    if ((r->r_loop_ohm < c->r_min_ohm) || (r->r_loop_ohm > c->r_max_ohm) ||
        (r->ls_h < c->l_min_h) || (r->ls_h > c->l_max_h)) {
        return false;
    }
    if (!flux) {
        return (r->valid_mask & ESP_FOC_MOTOR_ID_VALID_GAINS) != 0u;
    }
    if ((r->psi_f_wb < c->psi_min_wb) || (r->psi_f_wb > c->psi_max_wb)) {
        return false;
    }
    /* A shaft that slipped under I-f reads the wrong pole count, and its psi
     * is scaled by the same wrong speed. */
    if ((m->rotor != NULL) && (r->pole_pairs != c->pole_pairs)) {
        return false;
    }
    return true;
}

/*
 * One attempt of one electrical leg. The flux leg is handed the standstill
 * plant and skips the DC/AC probes: those need a restrained shaft, and
 * re-exciting the winding to learn the impedance twice is what trips the
 * bridge on re-arm.
 */
static esp_err_t elec_attempt(mid_t *m, bool flux, const esp_foc_motor_id_result_t *plant,
                              uint8_t attempt)
{
    const esp_foc_motor_id_config_t *mc = &m->cfg;
    esp_foc_motor_id_seq_config_t *c = &m->seq.cfg;
    esp_foc_motor_id_seq_default_config(c);
    c->vdc = mc->vdc;
    c->pwm_hz = m->pwm_hz;
    c->pole_pairs = mc->pole_pairs;
    c->i_probe_target_a = mc->i_probe_target_a;
    c->i_probe_max_a = mc->i_probe_max_a;
    c->i_bw_hz = mc->i_bw_hz;
    c->tune_backoff = mc->tune_backoff;
    c->sym_limit_permil = mc->sym_limit_permil;
    c->v_deadzone_v = mc->v_deadzone_v;
    c->probe_periods = mc->probe_periods;
    c->settle_periods = mc->settle_periods;
    c->skip_terminal = flux || mc->skip_terminal;
    c->skip_rs = flux;
    if (plant != NULL) {
        c->known_r_loop_ohm = plant->r_loop_ohm;
        c->known_ls_h = plant->ls_h;
        c->lag_samples = plant->lag_samples;
        c->phase_trim_cdeg = plant->phase_trim_cdeg;
    }
    c->do_flux = flux;
    c->i_flux_a = mc->i_flux_a;
    c->flux_hz = mc->flux_hz;

    esp_foc_motor_id_seq_ops_t *o = &m->seq.ops;
    memset(o, 0, sizeof(*o));
    o->set_theta = op_set_theta;
    o->set_vdq = op_set_vdq;
    o->set_idq = op_set_idq;
    o->set_fe_hz = op_set_fe;
    o->fetch_dq = op_fetch_dq;
    o->apply_gains = op_apply_gains;
    o->sleep_ms = op_sleep;
    o->faulted = op_faulted;
    o->on_phase = op_on_phase;
    o->fetch_rotor = (m->rotor != NULL) ? op_fetch_rotor : NULL;
    esp_foc_motor_id_probe_bind(&m->probe, o);

    esp_err_t err = bridge_arm(m);
    if (err == ESP_OK) {
        if (!flux) {
            esp_foc_sleep_ms(mc->settle_ms);
        }
        err = esp_foc_motor_id_seq_run(&m->seq);
        coast(m);
        if ((err == ESP_OK) && !plant_plausible(m, &m->seq.result, flux)) {
            m->seq.result.failed_at = flux ? ESP_FOC_MOTOR_ID_RATED_FLUX
                                           : ESP_FOC_MOTOR_ID_TUNE_I;
            err = ESP_ERR_INVALID_RESPONSE;
        }
    }
    emit(m, ESP_FOC_MOTOR_ID_EV_ATTEMPT, flux ? 1u : 0u, attempt, err,
         0.0f, 0.0f, 0.0f, 0.0f, &m->seq.result);
    return err;
}

static esp_err_t elec_leg(mid_t *m, bool flux, const esp_foc_motor_id_result_t *plant,
                          esp_foc_motor_id_result_t *out)
{
    esp_err_t err = ESP_FAIL;
    for (uint8_t k = 1u; k <= m->cfg.tries; k++) {
        err = elec_attempt(m, flux, plant, k);
        if (err == ESP_OK) {
            break;
        }
        if (k < m->cfg.tries) {
            esp_foc_sleep_ms(m->cfg.retry_ms);
        }
    }
    *out = m->seq.result;
    return err;
}

static esp_err_t run_elec(mid_t *m, esp_foc_motor_id_result_t *out)
{
    esp_foc_motor_id_result_t plant;
    esp_err_t err = elec_leg(m, false, NULL, &plant);
    *out = plant;
    if (err != ESP_OK) {
        return err;
    }

    esp_foc_motor_id_result_t spun;
    err = elec_leg(m, true, &plant, &spun);
    if (err == ESP_OK) {
        out->psi_f_wb = spun.psi_f_wb;
        out->flux_fm_hz = spun.flux_fm_hz;
        out->flux_we_rad_s = spun.flux_we_rad_s;
        out->valid_mask |= spun.valid_mask &
                           (ESP_FOC_MOTOR_ID_VALID_PSI_F | ESP_FOC_MOTOR_ID_VALID_PP);
    } else if (m->rotor == NULL) {
        out->failed_at = spun.failed_at;
        return err;
    }
    out->pole_pairs = m->cfg.pole_pairs;
    return ESP_OK;
}

static float we_hz(const mid_t *m)
{
    return q16_to_float(m->w_e) / MID_TWO_PI;
}

static bool ramp_iq(mid_t *m, float from_a, float to_a, uint32_t ms)
{
    const int steps = (ms < MID_DT_MS) ? 1 : (int)(ms / MID_DT_MS);
    for (int i = 0; i <= steps; i++) {
        m->iq_ref = q16_from_float(from_a + ((to_a - from_a) * (float)i / (float)steps));
        esp_foc_sleep_ms(MID_DT_MS);
        if (aborted(m)) {
            return false;
        }
    }
    return true;
}

static float meas_fm_hz(mid_t *m, uint32_t ms)
{
    m->th_m_acc = 0;
    m->meas_on = true;
    esp_foc_sleep_ms(ms);
    m->meas_on = false;
    return q16_to_float(m->th_m_acc) / (MID_TWO_PI * (float)ms * 1e-3f);
}

static bool bring_band(mid_t *m)
{
    const float lo = m->cfg.sensored.band_lo_hz;
    const float hi = m->cfg.sensored.band_hi_hz;
    const q16_t nudge = q16_from_float(m->cfg.sensored.nudge_a);
    const q16_t nudge_hi = q16_from_float(m->cfg.sensored.nudge_a * MID_NUDGE_HI_FRAC);
    m->iq_ref = nudge;
    for (uint32_t t = 0; t < MID_BAND_TIMEOUT_MS; t += MID_DT_MS) {
        if (aborted(m)) {
            return false;
        }
        const float f = fabsf(we_hz(m));
        if ((f >= lo) && (f < hi)) {
            return true;
        }
        m->iq_ref = (f >= hi) ? nudge_hi : nudge;
        esp_foc_sleep_ms(MID_DT_MS);
    }
    const float f = fabsf(we_hz(m));
    return (f >= lo) && (f < hi);
}

/* Rest means |we| under still_hz for MID_STILL_CALM_MS straight, iq* at zero. */
static bool wait_still(mid_t *m)
{
    m->iq_ref = 0;
    uint32_t calm = 0;
    for (uint32_t t = 0; t < MID_STILL_TIMEOUT_MS; t += MID_DT_MS) {
        if (aborted(m)) {
            return false;
        }
        calm = (fabsf(we_hz(m)) < m->cfg.sensored.still_hz) ? calm + MID_DT_MS : 0u;
        if (calm >= MID_STILL_CALM_MS) {
            return true;
        }
        esp_foc_sleep_ms(MID_DT_MS);
    }
    return false;
}

static void mech_set_idq(void *ctx, q16_t id, q16_t iq)
{
    mid_t *m = (mid_t *)ctx;
    m->id_ref = id;
    m->iq_ref = iq;
}

static void mech_get_idq(void *ctx, q16_t *id, q16_t *iq)
{
    mid_t *m = (mid_t *)ctx;
    *id = m->id_ref;
    *iq = m->iq_ref;
}

static q16_t mech_get_we(void *ctx)
{
    return ((mid_t *)ctx)->w_e;
}

static bool mech_faulted(void *ctx)
{
    return aborted((mid_t *)ctx);
}

static void mech_sleep(void *ctx, uint32_t ms)
{
    (void)ctx;
    esp_foc_sleep_ms(ms);
}

/*
 * Two-level torque step on the encoder speed with the speed hold off. A
 * refused fit is not fatal, the caller keeps its K; an abort or a shaft that
 * does not come to rest is.
 */
static esp_err_t mech_stage(mid_t *m, uint8_t pass, esp_foc_motor_id_result_t *out,
                            float *k_out)
{
    go(m, ESP_FOC_MOTOR_ID_MECH);
    m->hold_on = false;
    m->w_ref = 0;
    *k_out = 0.0f;
    if (!bring_band(m)) {
        return aborted(m) ? ESP_ERR_INVALID_STATE : ESP_ERR_TIMEOUT;
    }
    const float we0 = we_hz(m);

    esp_foc_mech_id_config_t mcfg;
    esp_foc_mech_id_default_config(&mcfg);
    mcfg.k_max = m->cfg.sensored.k_max;
    mcfg.i_max_a = m->cfg.sensored.i_max_a;
    mcfg.pole_pairs = m->cfg.pole_pairs;
    mcfg.psi_f_wb = ((out->valid_mask & ESP_FOC_MOTOR_ID_VALID_PSI_F) != 0u) ? out->psi_f_wb
                                                                             : 0.0f;
    const esp_foc_mech_id_ops_t ops = {
        .ctx = m,
        .set_idq = mech_set_idq,
        .get_idq = mech_get_idq,
        .get_omega_e = mech_get_we,
        .faulted = mech_faulted,
        .sleep_ms = mech_sleep,
    };
    esp_foc_mech_id_result_t r;
    memset(&r, 0, sizeof(r));
    const esp_err_t err = esp_foc_mech_id_run(&ops, &mcfg, &r);
    const bool k_ok = (err == ESP_OK) && ((r.valid_mask & ESP_FOC_MECH_ID_VALID_K) != 0u);
    /* Zero-acceleration crossing of the two-level line: drag at the probe speed. */
    const float drag = k_ok ? (r.iq_a[0] - (r.accel[0] / r.k_rad_s2_per_a)) : 0.0f;
    emit(m, ESP_FOC_MOTOR_ID_EV_MECH, pass, 0, err, r.k_rad_s2_per_a, drag, we0, r.j_kgm2,
         NULL);

    m->iq_ref = 0;
    esp_foc_sleep_ms(MID_STOP_MS);
    if (aborted(m)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!wait_still(m)) {
        return aborted(m) ? ESP_ERR_INVALID_STATE : ESP_ERR_TIMEOUT;
    }
    if (k_ok) {
        *k_out = r.k_rad_s2_per_a;
        out->drag_a = drag;
    }
    return ESP_OK;
}

/*
 * sigma of the PLL speed over ripple_ms, residual about a least-squares line,
 * so a slow coast does not read as noise. Taken at rest with iq* = 0: that is
 * the sensor + PLL noise Kp amplifies.
 */
static float measure_ripple(mid_t *m)
{
    const uint32_t fs = m->cfg.sensored.fetch_hz;
    m->rip_sum = 0;
    m->rip_sq = 0;
    m->rip_ix = 0;
    m->rip_n = 0u;
    m->rip_budget = (m->cfg.sensored.ripple_ms * fs) / 1000u;
    esp_foc_sleep_ms(m->cfg.sensored.ripple_ms + 40u);
    const uint32_t n = m->rip_n;
    const double sx = (double)m->rip_sum;
    const double sxx = (double)m->rip_sq;
    const double six = (double)m->rip_ix;
    m->rip_budget = 0u;
    if (n < MID_RIP_MIN_N) {
        return 0.0f;
    }
    const double dn = (double)n;
    const double sxx_c = sxx - sx * sx / dn;
    const double sii_c = dn * (dn * dn - 1.0) / 12.0;
    const double sxi_c = six - 0.5 * (dn - 1.0) * sx;
    double rss = sxx_c - ((sii_c > 0.0) ? (sxi_c * sxi_c / sii_c) : 0.0);
    if (rss < 0.0) {
        rss = 0.0;
    }
    const double q16_per_lsb = (double)(1u << MID_RIP_SHIFT) / 65536.0;
    return (float)(sqrt(rss / (dn - 2.0)) * q16_per_lsb);
}

/*
 * wn = min(want, noise ceiling, phase ceiling), both ceilings on the
 * crossover 2·zeta·wn. Noise: Kp may spend fb_iq_budget_a on one sigma of
 * feedback noise. Phase: the PLL's group delay is a pole near pll_bw/2, kept
 * pll_sep_min above the crossover.
 */
static esp_err_t hold_design(mid_t *m, float k, float psi_wb, float ripple)
{
    const esp_foc_motor_id_sensored_config_t *sc = &m->cfg.sensored;
    const float two_z = 2.0f * sc->hold_zeta;
    float bw = sc->hold_bw_min_hz;
    if (psi_wb > 1e-6f) {
        const float f_base = (m->cfg.vdc / sqrtf(3.0f)) / (MID_TWO_PI * psi_wb);
        bw = fminf(fmaxf(sc->hold_bw_frac * f_base, sc->hold_bw_min_hz), sc->hold_bw_max_hz);
    }
    if (ripple > 0.0f) {
        bw = fminf(bw, (sc->fb_iq_budget_a / ripple) * k / (two_z * MID_TWO_PI));
    }
    bw = fminf(bw, 0.5f * sc->pll_bw_hz / sc->pll_sep_min / two_z);

    const float fs = (float)sc->fetch_hz;
    float kp = 0.0f;
    float ki = 0.0f;
    const esp_err_t err = esp_foc_pid_design_integrator(k, fs, bw, sc->hold_zeta, &kp, &ki);
    emit(m, ESP_FOC_MOTOR_ID_EV_HOLD, 0, 0, err, ripple, bw, kp, ki, NULL);
    if (err != ESP_OK) {
        return err;
    }
    const q16_t u_lim = q16_from_float(sc->i_max_a);
    esp_foc_critical_enter();
    m->hold.kp_q32 = (int64_t)((double)kp * 4294967296.0 + 0.5);
    m->hold.ki_ts_q32 = (int64_t)((double)ki / (double)fs * 4294967296.0 + 0.5);
    m->hold.u_lim = u_lim;
    m->hold.i_lim_q32 = (int64_t)u_lim << 16;
    esp_foc_critical_leave();
    return ESP_OK;
}

static void hold_arm(mid_t *m)
{
    esp_foc_critical_enter();
    m->hold.i_q32 = (int64_t)m->iq_ref << 16;
    m->fresh = false;
    m->hold_on = true;
    esp_foc_critical_leave();
}

static bool ramp_wref(mid_t *m, int f0, int f1)
{
    const int df = (f1 >= f0) ? 1 : -1;
    for (int f = f0; f != f1;) {
        f += df;
        m->w_ref = q16_from_float(MID_TWO_PI * (float)f);
        esp_foc_sleep_ms(m->cfg.sensored.probe_step_ms);
        if (aborted(m)) {
            return false;
        }
    }
    return true;
}

typedef struct {
    float w;
    float iq;
    float park_err;
    float psi;
} mid_dq_t;

/* Mean dq over a window; the Park error is the angle that puts the back-EMF
 * on +q, independent of how much of it there is. */
static bool meas_dq(mid_t *m, const esp_foc_motor_id_result_t *out, mid_dq_t *d)
{
    esp_foc_critical_enter();
    m->acc_vd = 0;
    m->acc_vq = 0;
    m->acc_id = 0;
    m->acc_iq = 0;
    m->acc_w = 0;
    m->acc_n = 0;
    m->acc_on = true;
    esp_foc_critical_leave();
    esp_foc_sleep_ms(m->cfg.sensored.probe_avg_ms);
    esp_foc_critical_enter();
    m->acc_on = false;
    const uint32_t n = m->acc_n;
    const int64_t svd = m->acc_vd;
    const int64_t svq = m->acc_vq;
    const int64_t sid = m->acc_id;
    const int64_t siq = m->acc_iq;
    const int64_t sw = m->acc_w;
    esp_foc_critical_leave();
    if (n == 0u) {
        return false;
    }
    const float k = 1.0f / (65536.0f * (float)n);
    const float vd = (float)svd * k * m->cfg.vdc;
    const float vq = (float)svq * k * m->cfg.vdc;
    const float id = (float)sid * k;
    const float iq = (float)siq * k;
    const float w = (float)sw * k;
    const float r = out->r_loop_ohm;
    const float l = out->ls_h;
    const float rd = vd - (r * id) + (w * l * iq);
    const float rq = vq - (r * iq) - (w * l * id);
    const float sg = (w >= 0.0f) ? 1.0f : -1.0f;
    d->w = w;
    d->iq = iq;
    d->park_err = -atan2f(sg * rd, sg * rq);
    d->psi = sqrtf((rd * rd) + (rq * rq)) / fmaxf(fabsf(w), 1.0f);
    return true;
}

/*
 * One pass over ±{1..4}·probe_base_hz under the speed hold, then a
 * least-squares fit e = d0 + tau·ω. The top point stays clear of the iq
 * rail an uncompensated d0 reaches near 300 Hz elec on a small outrunner.
 */
static bool angle_pass(mid_t *m, uint8_t pass, const esp_foc_motor_id_result_t *out,
                       float *d0_out, float *tau_out, float *rms_out, float *psi_out)
{
    const int base = m->cfg.sensored.probe_base_hz;
    const int pts[MID_ANGLE_PTS] = { base, 2 * base, 3 * base, 4 * base,
                                     -base, -2 * base, -3 * base, -4 * base };
    float w[MID_ANGLE_PTS];
    float e[MID_ANGLE_PTS];
    float psi_sum = 0.0f;
    int f_now = 0;
    for (int i = 0; i < MID_ANGLE_PTS; i++) {
        if (!ramp_wref(m, f_now, pts[i])) {
            return false;
        }
        f_now = pts[i];
        esp_foc_sleep_ms(m->cfg.sensored.probe_settle_ms);
        mid_dq_t d;
        if (aborted(m) || !meas_dq(m, out, &d)) {
            return false;
        }
        w[i] = d.w;
        e[i] = d.park_err;
        psi_sum += d.psi;
        emit(m, ESP_FOC_MOTOR_ID_EV_ANGLE_POINT, pass, 0, ESP_OK, d.w / MID_TWO_PI,
             d.park_err, d.psi, d.iq, NULL);
    }
    if (!ramp_wref(m, f_now, 0)) {
        return false;
    }

    float wm = 0.0f;
    float em = 0.0f;
    for (int i = 0; i < MID_ANGLE_PTS; i++) {
        wm += w[i];
        em += e[i];
    }
    wm /= (float)MID_ANGLE_PTS;
    em /= (float)MID_ANGLE_PTS;
    float sww = 0.0f;
    float swe = 0.0f;
    for (int i = 0; i < MID_ANGLE_PTS; i++) {
        sww += (w[i] - wm) * (w[i] - wm);
        swe += (w[i] - wm) * (e[i] - em);
    }
    const float tau = (sww > 0.0f) ? (swe / sww) : 0.0f;
    const float d0 = em - (tau * wm);
    float sres = 0.0f;
    for (int i = 0; i < MID_ANGLE_PTS; i++) {
        const float res = e[i] - (d0 + (tau * w[i]));
        sres += res * res;
    }
    *d0_out = d0;
    *tau_out = tau;
    *rms_out = sqrtf(sres / (float)MID_ANGLE_PTS);
    *psi_out = psi_sum / (float)MID_ANGLE_PTS;
    return true;
}

/* Two passes: the first fits the raw encoder frame, the second reads the
 * residual on the compensated one and folds it in. */
static esp_err_t angle_compensate(mid_t *m, esp_foc_motor_id_result_t *out, float *psi_out)
{
    for (uint8_t pass = 0; pass < 2u; pass++) {
        float d0 = 0.0f;
        float tau = 0.0f;
        float rms = 0.0f;
        if (!angle_pass(m, pass, out, &d0, &tau, &rms, psi_out)) {
            return ESP_ERR_INVALID_STATE;
        }
        const float d0_tot = out->park_offset_rad + d0;
        const float tau_tot = out->park_lead_s + tau;
        const bool ok = (fabsf(d0_tot) <= m->cfg.sensored.d0_max_rad) &&
                        (fabsf(tau_tot) <= m->cfg.sensored.tau_max_s);
        emit(m, ESP_FOC_MOTOR_ID_EV_ANGLE_FIT, pass, 0,
             ok ? ESP_OK : ESP_ERR_INVALID_RESPONSE, d0_tot, tau_tot, rms, *psi_out, NULL);
        if (!ok) {
            break;
        }
        esp_foc_critical_enter();
        m->th_off = q16_from_float(d0_tot);
        m->tau_q32 = (int32_t)lrintf(tau_tot * 4294967296.0f);
        esp_foc_critical_leave();
        out->park_offset_rad = d0_tot;
        out->park_lead_s = tau_tot;
        out->park_fit_rms_rad = rms;
        out->valid_mask |= ESP_FOC_MOTOR_ID_VALID_PARK;
    }
    return ESP_OK;
}

static esp_err_t sensored_stages(mid_t *m, esp_foc_motor_id_result_t *out)
{
    const esp_foc_motor_id_sensored_config_t *sc = &m->cfg.sensored;
    esp_foc_motor_id_seq_config_t gc;
    esp_foc_motor_id_seq_default_config(&gc);
    gc.vdc = m->cfg.vdc;
    gc.pwm_hz = m->pwm_hz;
    gc.i_bw_hz = sc->i_bw_hz;
    gc.tune_backoff = m->cfg.tune_backoff;
    float kp = 0.0f;
    float ki = 0.0f;
    esp_err_t err = esp_foc_motor_id_seq_gains_for(&gc, out->r_loop_ohm, out->ls_h, &kp, &ki);
    if (err != ESP_OK) {
        return err;
    }
    set_gains(m, kp, ki);

    go(m, ESP_FOC_MOTOR_ID_DIRECTION);
    err = bridge_arm(m);
    if (err != ESP_OK) {
        return err;
    }
    m->th_off = 0;
    m->tau_q32 = 0;
    m->on_rotor = true;
    m->run = true;
    if (!ramp_iq(m, 0.0f, sc->iq_dir_a, sc->dir_ramp_ms)) {
        return ESP_ERR_INVALID_STATE;
    }
    out->dir_fm_hz = meas_fm_hz(m, MID_DIR_MEAS_MS);
    emit(m, ESP_FOC_MOTOR_ID_EV_DIRECTION, 0, 0, ESP_OK, out->dir_fm_hz, sc->iq_dir_a, 0.0f,
         0.0f, NULL);
    if (out->dir_fm_hz < sc->fm_min_hz) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    out->valid_mask |= ESP_FOC_MOTOR_ID_VALID_DIR;

    float k = 0.0f;
    err = mech_stage(m, 0, out, &k);
    if (err != ESP_OK) {
        return err;
    }
    if (k > 0.0f) {
        out->k_raw_rad_s2_per_a = k;
        out->k_rad_s2_per_a = k;
        out->valid_mask |= ESP_FOC_MOTOR_ID_VALID_K;
    }
    if (sc->angle_comp) {
        go(m, ESP_FOC_MOTOR_ID_PARK_ANGLE);
        const float psi_hint =
            ((out->valid_mask & ESP_FOC_MOTOR_ID_VALID_PSI_F) != 0u) ? out->psi_f_wb : 0.0f;
        const float ripple = measure_ripple(m);
        err = hold_design(m, (k > 0.0f) ? k : sc->k_fallback, psi_hint, ripple);
        if (err != ESP_OK) {
            return err;
        }
        hold_arm(m);
        float psi_probe = 0.0f;
        err = angle_compensate(m, out, &psi_probe);
        m->hold_on = false;
        m->iq_ref = 0;
        if (err != ESP_OK) {
            return err;
        }
        if (((out->valid_mask & ESP_FOC_MOTOR_ID_VALID_PSI_F) == 0u) &&
            (psi_probe >= m->cfg.psi_min_wb) && (psi_probe <= m->cfg.psi_max_wb)) {
            out->psi_f_wb = psi_probe;
            out->valid_mask |= ESP_FOC_MOTOR_ID_VALID_PSI_F;
        }
        if ((out->valid_mask & ESP_FOC_MOTOR_ID_VALID_PARK) != 0u) {
            /* K scales with cos(Park error); the raw-angle K is not the plant. */
            err = mech_stage(m, 1, out, &k);
            if (err != ESP_OK) {
                return err;
            }
            if (k > 0.0f) {
                out->k_rad_s2_per_a = k;
                out->valid_mask |= ESP_FOC_MOTOR_ID_VALID_K;
            }
        }
    }
    bridge_off(m);
    if (((out->valid_mask & ESP_FOC_MOTOR_ID_VALID_K) != 0u) &&
        ((out->valid_mask & ESP_FOC_MOTOR_ID_VALID_PSI_F) != 0u)) {
        const float pp = (float)m->cfg.pole_pairs;
        out->j_kgm2 = 1.5f * pp * pp * out->psi_f_wb / out->k_rad_s2_per_a;
        out->valid_mask |= ESP_FOC_MOTOR_ID_VALID_J;
    }
    return ESP_OK;
}

static esp_err_t check_config(const esp_foc_motor_id_config_t *c, esp_foc_inverter_t *inv,
                              esp_foc_rotor_sensor_t *rotor)
{
    if ((c->pole_pairs <= 0) || !(c->vdc > 0.0f) || (c->tries == 0u) ||
        !(c->i_abort_a > 0.0f) || (c->probe_periods < 4u)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (rotor == NULL) {
        return ESP_OK;
    }
    const uint32_t pwm_hz = inv->get_pwm_rate_hz(inv);
    const esp_foc_motor_id_sensored_config_t *sc = &c->sensored;
    if ((sc->fetch_hz == 0u) || (sc->fetch_hz > pwm_hz) || ((pwm_hz % sc->fetch_hz) != 0u) ||
        !(sc->i_max_a > 0.0f) || !(sc->k_fallback > 0.0f) || !(sc->overspeed_hz > 0.0f) ||
        !(sc->band_hi_hz > sc->band_lo_hz) || (sc->probe_base_hz <= 0) ||
        !(sc->pll_sep_min > 0.0f) || !(sc->hold_zeta > 0.0f)) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static esp_err_t install(mid_t *m)
{
    esp_foc_inverter_t *inv = m->inv;
    const float ts = 1.0f / (float)m->pwm_hz;
    esp_err_t err = esp_foc_pid_init(&m->pi_d, MID_PI_SEED_KP, MID_PI_SEED_KI, 0.0f, 0.0f, ts);
    if (err == ESP_OK) {
        err = esp_foc_pid_init(&m->pi_q, MID_PI_SEED_KP, MID_PI_SEED_KI, 0.0f, 0.0f, ts);
    }
    if (err == ESP_OK) {
        err = esp_foc_motor_id_probe_init(&m->probe, inv, m->cfg.vdc, m->cfg.i_abort_a);
    }
    if (err != ESP_OK) {
        return err;
    }

    if (m->rotor != NULL) {
        const esp_foc_motor_id_sensored_config_t *sc = &m->cfg.sensored;
        esp_foc_rotor_pll_config_t pcfg;
        esp_foc_rotor_pll_config_default(&pcfg, sc->fetch_hz, sc->pll_bw_hz, sc->pll_zeta,
                                         0.0f);
        pcfg.domain = ESP_FOC_ROTOR_PLL_ELEC;
        err = esp_foc_rotor_pll_init(&m->pll, &pcfg);
        if (err != ESP_OK) {
            return err;
        }
        m->slow_div = m->pwm_hz / sc->fetch_hz;
        m->fetch_hz_q = q16_from_float((float)sc->fetch_hz);
        m->w_overspeed = q16_from_float(MID_TWO_PI * sc->overspeed_hz);
        m->vlead_q32 = (int32_t)lrintf(sc->pwm_delay_ts / (float)m->pwm_hz * 4294967296.0f);
        m->rotor_alive = true;
        if (esp_foc_task_spawn(mid_rotor_task, m, "foc_id_rotor", MID_ROTOR_STACK,
                               esp_foc_task_max_priority(), NULL) != 0) {
            m->rotor_alive = false;
            return ESP_ERR_NO_MEM;
        }
        for (uint32_t t = 0; (m->rotor_ev == NULL) && (t < MID_TASK_TIMEOUT_MS); t++) {
            esp_foc_sleep_ms(1);
        }
    }

    inv->set_duties(inv, Q16_HALF, Q16_HALF, Q16_HALF);
    esp_foc_critical_enter();
    inv->set_dma_callback(inv, NULL, NULL);
    inv->set_fault_callback(inv, mid_fault, m);
    inv->set_pwm_callback(inv, mid_tez, m);
    esp_foc_critical_leave();
    return ESP_OK;
}

static void release(mid_t *m)
{
    esp_foc_inverter_t *inv = m->inv;
    m->run = false;
    m->hold_on = false;
    esp_foc_critical_enter();
    inv->set_pwm_callback(inv, NULL, NULL);
    inv->set_dma_callback(inv, NULL, NULL);
    inv->set_fault_callback(inv, NULL, NULL);
    esp_foc_critical_leave();
    bridge_off(m);
    if (m->rotor_alive) {
        m->quit = true;
        esp_foc_event_post(m->rotor_ev);
        for (uint32_t t = 0; m->rotor_alive && (t < MID_TASK_TIMEOUT_MS); t++) {
            esp_foc_sleep_ms(1);
        }
    }
}

static void mid_worker(void *arg)
{
    mid_t *m = (mid_t *)arg;
    esp_foc_motor_id_result_t *out = m->out;
    esp_err_t err = install(m);
    if (err == ESP_OK) {
        err = run_elec(m, out);
    }
    if ((err == ESP_OK) && (m->rotor != NULL)) {
        err = sensored_stages(m, out);
        if (err != ESP_OK) {
            out->failed_at = m->phase;
        }
    }
    release(m);
    go(m, (err == ESP_OK) ? ESP_FOC_MOTOR_ID_DONE : ESP_FOC_MOTOR_ID_FAIL);
    m->err = err;
    m->worker_alive = false;
    esp_foc_task_delete_self();
}

esp_err_t esp_foc_motor_id_run(esp_foc_inverter_t *inv,
                               esp_foc_rotor_sensor_t *rotor,
                               const esp_foc_motor_id_config_t *cfg,
                               esp_foc_motor_id_result_t *out)
{
    if ((inv == NULL) || (cfg == NULL) || (out == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (esp_foc_in_isr_context()) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = check_config(cfg, inv, rotor);
    if (err != ESP_OK) {
        return err;
    }
    esp_foc_critical_enter();
    const bool busy = s_busy;
    s_busy = true;
    esp_foc_critical_leave();
    if (busy) {
        return ESP_ERR_INVALID_STATE;
    }

    mid_t *m = &s_mid;
    memset(m, 0, sizeof(*m));
    memset(out, 0, sizeof(*out));
    m->inv = inv;
    m->rotor = rotor;
    m->cfg = *cfg;
    m->pwm_hz = inv->get_pwm_rate_hz(inv);
    m->vdc_q = q16_from_float(cfg->vdc);
    m->phase = ESP_FOC_MOTOR_ID_IDLE;
    m->out = out;

    m->worker_alive = true;
    if (esp_foc_task_spawn(mid_worker, m, "foc_id", CONFIG_ESP_FOC_MOTOR_ID_TASK_STACK,
                           CONFIG_ESP_FOC_MOTOR_ID_TASK_PRIO, NULL) != 0) {
        m->worker_alive = false;
        s_busy = false;
        return ESP_ERR_NO_MEM;
    }
    /* Polled, not notified: a post landing after a timed-out wait would stay
     * pending on the caller's task and wake its next unrelated wait. */
    while (m->worker_alive) {
        esp_foc_sleep_ms(MID_JOIN_POLL_MS);
    }
    err = m->err;
    s_busy = false;
    return err;
}
