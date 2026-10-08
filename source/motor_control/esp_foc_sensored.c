/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Sensored FoC stack: TEZ fast path on the compensated encoder angle, the
 * per-sample slot (position P, speed PI, cogging feedforward, guards) and the
 * supervisor that feeds the encoder to the PLL and walks the state machine.
 */
#include "espFoC/motor_control/esp_foc_sensored.h"

#include <math.h>
#include <string.h>

#include "sdkconfig.h"
#include "espFoC/motor_control/esp_foc_rotor_pll.h"
#include "espFoC/osal/esp_foc_osal.h"
#include "espFoC/utils/esp_foc_angle.h"
#include "espFoC/utils/esp_foc_clarke.h"
#include "espFoC/utils/esp_foc_park.h"
#include "espFoC/utils/esp_foc_pid.h"
#include "espFoC/utils/esp_foc_svm.h"
#include "espFoC/utils/esp_foc_trig.h"
#include "espFoC/utils/esp_foc_vlim.h"

#define SD_TWO_PI          6.28318530718f
#define SD_INV_SQRT3       0.57735026919f
#define SD_BINS            ((uint32_t)CONFIG_ESP_FOC_SD_COG_BINS)
#define SD_POLL_MS         20u
#define SD_REQ_TIMEOUT_MS  5000u
#define SD_CAL_ROUNDS      32
#define SD_ARM_MS          50u
#define SD_STILL_HOLD_MS   300u
#define SD_Q32             4294967296.0

/*
 * Ripple sums are of the deviation from the window's first sample shifted
 * down before squaring: a window drifting to a stop at full Q16 wraps int64
 * and reads back as zero residual.
 */
#define SD_RIP_SHIFT 8
/* Window squares are of werr >> 6 for the same reason, over a 1 s merge. */
#define SD_WIN_SHIFT 6

#define SD_REQ_RUN   (1u << 0)
#define SD_REQ_STOP  (1u << 1)
#define SD_REQ_CLEAR (1u << 2)
#define SD_REQ_EXIT  (1u << 3)
#define SD_REQ_LEARN (1u << 4)

typedef enum {
    SD_OK = 0,
    SD_STOP,
    SD_EXIT,
    SD_FAILED,
    SD_ABORT,
    SD_FAULT,
    SD_TIMEOUT,
} sd_out_t;

typedef struct {
    /* --- TEZ fast path --- */
    volatile bool run;
    volatile bool slot_on;
    volatile bool guards_on;
    volatile bool fresh;
    volatile bool fetch_req;
    volatile uint8_t mode;
    uint32_t slow_div;
    uint32_t div;
    uint32_t since_fresh;
    uint32_t stale_limit;
    volatile q16_t th_off;
    volatile int32_t tau_q32;
    volatile int32_t vlead_q32;
    volatile q16_t iu;
    volatile q16_t iv;
    volatile q16_t iw;
    volatile q16_t id_user;
    volatile q16_t iq_user;
    volatile q16_t vd_ff;
    volatile q16_t vq_ff;
    volatile q16_t iq_ref;
    esp_foc_pid_t pi_d;
    esp_foc_pid_t pi_q;
    volatile q16_t theta;
    volatile q16_t id;
    volatile q16_t iq;
    volatile q16_t vd;
    volatile q16_t vq;
    volatile uint32_t tez;

    /* --- Slot --- */
    volatile int64_t kp_w_q32;
    volatile int64_t ki_ts_q32;
    int64_t wi_q32;
    int64_t wi_lim_q32;
    q16_t i_max;
    volatile q16_t w_target;
    volatile q16_t w_step;
    volatile q16_t w_ref;
    volatile int64_t pos_raw;     /* unwrapped mechanical, Q16 rad */
    volatile int64_t pos_org;
    volatile int64_t ref_abs;     /* position reference, same frame as pos_raw */
    volatile q16_t w_ff;          /* mechanical rad/s */
    volatile q16_t kp_pos;
    q16_t corr_max;
    q16_t wm_max;
    int32_t pp;
    volatile q16_t th_m;          /* last encoder mechanical angle, wrapped */
    volatile q16_t learn_dth;
    volatile int8_t map_dir;
    volatile bool cog_on;
    q16_t cog_k;                  /* bins per rad, Q16 */
    q16_t *cog;
    int32_t (*map_sum)[SD_BINS];
    uint16_t (*map_n)[SD_BINS];
    q16_t g_w_trip;
    uint32_t g_ov_n;
    uint32_t g_ov_need;
    volatile uint8_t abort;
    volatile uint8_t fault;

    /* --- Supervisor --- */
    esp_foc_inverter_t *inv;
    esp_foc_rotor_sensor_t *rotor;
    esp_foc_sensored_config_t cfg;
    esp_foc_rotor_pll_t pll;
    volatile bool pll_on;
    uint8_t axis;
    bool inited;
    volatile bool task_alive;
    volatile bool latched;
    bool bridge_on;
    bool have_th_m;
    bool have_seq;
    uint32_t seq_last;
    volatile esp_foc_event_handle_t sup;
    volatile uint32_t req;
    volatile esp_err_t req_err;
    volatile esp_foc_sensored_state_t state;
    esp_foc_sensored_fail_t fail;
    esp_foc_sensored_abort_t abort_seen;
    esp_foc_fault_reason_t fault_seen;
    uint32_t pwm_hz;
    float vdc_v;
    float slot_hz;
    q16_t th_m_prev;
    uint64_t idle_fetch_us;
    uint32_t fetch_n;
    uint32_t fetch_fail;
    uint32_t fail_run;
    esp_foc_sensored_tuning_t tune;

    uint32_t rip_budget;
    uint32_t rip_n;
    q16_t rip_seed;
    int64_t rip_sum;
    int64_t rip_sq;
    int64_t rip_ix;

    uint32_t win_n;
    int64_t win_w;
    int64_t win_e;
    int64_t win_e2;
    q16_t win_emin;
    q16_t win_emax;
    q16_t win_eprev;
    q16_t win_bias;
    uint32_t win_cross;
    int64_t win_iqr;
    q16_t win_iqr_min;
    q16_t win_iqr_max;
    int64_t win_iq;

    q16_t inpos_band;
    q16_t inpos_w;
    uint32_t inpos_need;
    uint32_t inpos_cnt;
    volatile bool inpos;
    volatile uint32_t inpos_toggles;

    uint32_t learn_timeout_ms;
    uint64_t learn_deadline_us;
    bool cog_valid;
    esp_foc_sensored_cogging_info_t cog_info;
} sd_t;

static sd_t s_sd[CONFIG_ESP_FOC_SD_MAX_AXES];
static q16_t s_cog[CONFIG_ESP_FOC_SD_MAX_AXES][SD_BINS];
static int32_t s_map_sum[CONFIG_ESP_FOC_SD_MAX_AXES][2][SD_BINS];
static uint16_t s_map_n[CONFIG_ESP_FOC_SD_MAX_AXES][2][SD_BINS];

#if CONFIG_ESP_FOC_SD_MAX_AXES > 4
#error "k_task_name covers four axes"
#endif
static const char *const k_task_name[] = {"foc_sd0", "foc_sd1", "foc_sd2", "foc_sd3"};

static inline sd_t *sd_axis(uint8_t axis)
{
    return (axis < CONFIG_ESP_FOC_SD_MAX_AXES) ? &s_sd[axis] : NULL;
}

static inline q16_t sd_abs(q16_t x)
{
    return (x < 0) ? q16_neg(x) : x;
}

static inline q16_t sd_slew(q16_t x, q16_t target, q16_t step)
{
    const q16_t d = q16_sub(target, x);
    if (d > step) {
        return q16_add(x, step);
    }
    if (d < q16_neg(step)) {
        return q16_sub(x, step);
    }
    return target;
}

static inline q16_t sd_sat64(int64_t x)
{
    if (x > (int64_t)INT32_MAX) {
        return INT32_MAX;
    }
    if (x < -(int64_t)INT32_MAX) {
        return -INT32_MAX;
    }
    return (q16_t)x;
}

static inline uint32_t sd_bin(const sd_t *sd, q16_t th_m)
{
    const q16_t m = (th_m < 0) ? q16_add(th_m, Q16_TWO_PI) : th_m;
    const uint32_t i = (uint32_t)(((int64_t)m * (int64_t)sd->cog_k) >> 32);
    return (i < SD_BINS) ? i : (SD_BINS - 1u);
}

/* ------------------------------------------------------------------------ */
/* Fast path                                                                 */
/* ------------------------------------------------------------------------ */

static void sd_dma(void *arg)
{
    sd_t *sd = (sd_t *)arg;
    esp_foc_inverter_t *inv = sd->inv;
    q16_t u;
    q16_t v;
    q16_t w;
    inv->fetch_currents(inv, &u, &v, &w);
    sd->iu = u;
    sd->iv = v;
    sd->iw = w;
}

static void sd_fault(void *arg, esp_foc_fault_reason_t reason)
{
    sd_t *sd = (sd_t *)arg;
    sd->run = false;
    sd->fault = (uint8_t)reason;
    esp_foc_event_post_auto(sd->sup);
}

/*
 * Positional PI: P never enters the state, and I only integrates when it
 * does not push the output further into the clamp. Integral gain in Q32:
 * Ki·Ts of a speed loop is under one Q16 LSB.
 */
static inline q16_t sd_speed_pi(sd_t *sd, q16_t e, q16_t ff)
{
    const int64_t kp = sd->kp_w_q32;
    const int64_t ki = sd->ki_ts_q32;
    const int64_t i_old = sd->wi_q32;
    const int64_t i_lim = sd->wi_lim_q32;
    const int64_t lim = (int64_t)sd->i_max;

    const int64_t inc = (ki * (int64_t)e) >> 16;
    int64_t i_new = i_old + inc;
    if (i_new > i_lim) {
        i_new = i_lim;
    } else if (i_new < -i_lim) {
        i_new = -i_lim;
    }
    int64_t u = ((kp * (int64_t)e) >> 32) + (i_new >> 16) + (int64_t)ff;
    bool hold = false;
    if (u > lim) {
        u = lim;
        hold = (inc > 0);
    } else if (u < -lim) {
        u = -lim;
        hold = (inc < 0);
    }
    if (!hold) {
        sd->wi_q32 = i_new;
    }
    return (q16_t)u;
}

static uint8_t sd_slot(sd_t *sd, q16_t w_e)
{
    const uint8_t mode = sd->mode;
    const q16_t iq_user = sd->iq_user;
    const q16_t th_m = sd->th_m;
    const int8_t dir = sd->map_dir;
    const bool cog_on = sd->cog_on;

    if (sd_abs(w_e) > sd->g_w_trip) {
        if (++sd->g_ov_n >= sd->g_ov_need) {
            return ESP_FOC_SD_ABORT_OVERSPEED;
        }
    } else {
        sd->g_ov_n = 0;
    }
    if (!sd->slot_on) {
        return ESP_FOC_SD_ABORT_NONE;
    }
    if (mode == ESP_FOC_SD_MODE_TORQUE) {
        sd->iq_ref = iq_user;
        return ESP_FOC_SD_ABORT_NONE;
    }

    uint32_t idx = 0;
    q16_t ff = iq_user;
    if (cog_on || (dir >= 0)) {
        idx = sd_bin(sd, th_m);
        if (cog_on) {
            ff = q16_add(ff, sd->cog[idx]);
        }
    }

    q16_t w_ref;
    if (mode == ESP_FOC_SD_MODE_POSITION) {
        const int64_t pos = sd->pos_raw;
        int64_t ref = sd->ref_abs;
        const q16_t dth = sd->learn_dth;
        const q16_t w_ff = sd->w_ff;
        const q16_t kp = sd->kp_pos;
        const q16_t cmax = sd->corr_max;
        const q16_t wmax = sd->wm_max;
        if (dth != 0) {
            ref += dth;
            sd->ref_abs = ref;
        }
        const q16_t corr = q16_clamp(q16_mul(kp, sd_sat64(ref - pos)), q16_neg(cmax), cmax);
        const q16_t wm = q16_clamp(q16_add(w_ff, corr), q16_neg(wmax), wmax);
        w_ref = (q16_t)(wm * sd->pp);
    } else {
        w_ref = sd_slew(sd->w_ref, sd->w_target, sd->w_step);
    }
    const q16_t u = sd_speed_pi(sd, q16_sub(w_ref, w_e), ff);

    sd->w_ref = w_ref;
    sd->iq_ref = u;
    if (dir >= 0) {
        sd->map_sum[dir][idx] += u;
        if (sd->map_n[dir][idx] < UINT16_MAX) {
            sd->map_n[dir][idx]++;
        }
    }
    return ESP_FOC_SD_ABORT_NONE;
}

static void sd_tez(void *arg)
{
    sd_t *sd = (sd_t *)arg;
    esp_foc_inverter_t *inv = sd->inv;
    esp_foc_rotor_sensor_t *rot = sd->rotor;
    sd->tez++;
    esp_foc_rotor_sensor_step(rot);
    if (++sd->div >= sd->slow_div) {
        sd->div = 0;
        sd->fetch_req = true;
        esp_foc_event_post_auto(sd->sup);
    }
    if (!sd->run) {
        return;
    }

    esp_foc_rotor_state_t snap;
    esp_foc_rotor_sensor_snapshot(rot, &snap);
    const q16_t iu = sd->iu;
    const q16_t iv = sd->iv;
    const q16_t iw = sd->iw;
    const q16_t idr = sd->id_user;
    const q16_t iqr = sd->iq_ref;
    const q16_t vd_ff = sd->vd_ff;
    const q16_t vq_ff = sd->vq_ff;
    const q16_t th_off = sd->th_off;
    const int64_t tau = sd->tau_q32;
    const int64_t vl = sd->vlead_q32;
    const q16_t w_e = esp_foc_rotor_pll_get_omega(&sd->pll);

    const q16_t lead = (q16_t)(((int64_t)w_e * tau) >> 32);
    const q16_t th = q16_wrap_pi(q16_add(snap.theta_e, q16_add(th_off, lead)));
    const q16_t vlead = (q16_t)(((int64_t)w_e * vl) >> 32);

    q16_t s;
    q16_t c;
    esp_foc_sincos(th, &s, &c);
    q16_t ia;
    q16_t ib;
    q16_t id;
    q16_t iq;
    esp_foc_clarke(iu, iv, iw, &ia, &ib);
    esp_foc_park(s, c, ia, ib, &id, &iq);

    q16_t vd = q16_add(esp_foc_pid_update(&sd->pi_d, idr, id), vd_ff);
    q16_t vq = q16_add(esp_foc_pid_update(&sd->pi_q, iqr, iq), vq_ff);
    esp_foc_vlim_dq(&vd, &vq, Q16_INV_SQRT3);
    esp_foc_pid_set_applied(&sd->pi_d, q16_sub(vd, vd_ff));
    esp_foc_pid_set_applied(&sd->pi_q, q16_sub(vq, vq_ff));

    /* The PWM-delay lead rotates (s, c) by a cubic instead of a second
     * CORDIC, which does not fit the period; |vlead| stays under 0.3 rad
     * at rated speed, where the cubic is 1e-4 off. */
    if (vlead != 0) {
        const q16_t d2 = q16_mul(vlead, vlead);
        const q16_t cd = q16_sub(Q16_ONE, d2 >> 1);
        const q16_t sd_ = q16_sub(vlead, q16_mul(q16_mul(d2, vlead), (q16_t)10923));
        const q16_t s0 = s;
        s = q16_add(q16_mul(s0, cd), q16_mul(c, sd_));
        c = q16_sub(q16_mul(c, cd), q16_mul(s0, sd_));
    }
    q16_t a;
    q16_t b;
    q16_t du;
    q16_t dv;
    q16_t dw;
    esp_foc_inv_park(s, c, vd, vq, &a, &b);
    esp_foc_svm(a, b, &du, &dv, &dw);
    inv->set_duties(inv, du, dv, dw);

    sd->theta = th;
    sd->id = id;
    sd->iq = iq;
    sd->vd = vd;
    sd->vq = vq;

    uint8_t why = ESP_FOC_SD_ABORT_NONE;
    if (sd->fresh) {
        sd->fresh = false;
        sd->since_fresh = 0;
        why = sd_slot(sd, w_e);
    } else if (sd->guards_on && (++sd->since_fresh > sd->stale_limit)) {
        why = ESP_FOC_SD_ABORT_SENSOR_STALE;
    }
    if ((why != ESP_FOC_SD_ABORT_NONE) && sd->guards_on) {
        sd->abort = why;
        sd->run = false;
        inv->set_duties(inv, Q16_HALF, Q16_HALF, Q16_HALF);
        esp_foc_event_post_auto(sd->sup);
    }
}

/* ------------------------------------------------------------------------ */
/* Encoder feed                                                              */
/* ------------------------------------------------------------------------ */

static void sd_window_add(sd_t *sd, q16_t w)
{
    const q16_t e = q16_sub(w, sd->w_ref);
    const q16_t iqr = sd->iq_ref;
    const q16_t iq = sd->iq;
    const int64_t es = (int64_t)(e >> SD_WIN_SHIFT);

    esp_foc_critical_enter();
    if (sd->win_n == 0u) {
        sd->win_emin = e;
        sd->win_emax = e;
        sd->win_iqr_min = iqr;
        sd->win_iqr_max = iqr;
    } else {
        sd->win_emin = q16_min(sd->win_emin, e);
        sd->win_emax = q16_max(sd->win_emax, e);
        sd->win_iqr_min = q16_min(sd->win_iqr_min, iqr);
        sd->win_iqr_max = q16_max(sd->win_iqr_max, iqr);
        if ((e < sd->win_bias) != (sd->win_eprev < sd->win_bias)) {
            sd->win_cross++;
        }
    }
    sd->win_eprev = e;
    sd->win_w += w;
    sd->win_e += e;
    sd->win_e2 += es * es;
    sd->win_iqr += iqr;
    sd->win_iq += iq;
    sd->win_n++;
    esp_foc_critical_leave();
}

static void sd_ripple_add(sd_t *sd, q16_t w)
{
    const uint32_t n = sd->rip_n;
    if (n >= sd->rip_budget) {
        return;
    }
    if (n == 0u) {
        sd->rip_seed = w;
    }
    const int64_t x = (int64_t)q16_sub(w, sd->rip_seed) >> SD_RIP_SHIFT;
    sd->rip_sum += x;
    sd->rip_sq += x * x;
    sd->rip_ix += (int64_t)n * x;
    sd->rip_n = n + 1u;
}

static void sd_inpos_add(sd_t *sd, q16_t w)
{
    if (sd->mode != ESP_FOC_SD_MODE_POSITION) {
        sd->inpos_cnt = 0;
        if (sd->inpos) {
            sd->inpos = false;
            sd->inpos_toggles++;
        }
        return;
    }
    esp_foc_critical_enter();
    const int64_t e64 = sd->ref_abs - sd->pos_raw;
    esp_foc_critical_leave();
    const q16_t e = sd_abs(sd_sat64(e64));
    bool in = sd->inpos;
    if ((e <= sd->inpos_band) && (sd_abs(w) <= sd->inpos_w) && (sd->learn_dth == 0)) {
        if (sd->inpos_cnt < sd->inpos_need) {
            sd->inpos_cnt++;
        } else {
            in = true;
        }
    } else {
        sd->inpos_cnt = 0;
        if (e > sd->inpos_band) {
            in = false;
        }
    }
    if (in != sd->inpos) {
        sd->inpos = in;
        sd->inpos_toggles++;
    }
}

static void sd_unwrap(sd_t *sd, q16_t th_m)
{
    if (!sd->have_th_m) {
        sd->have_th_m = true;
        sd->th_m_prev = th_m;
        sd->th_m = th_m;
        return;
    }
    const q16_t d = q16_angle_delta(sd->th_m_prev, th_m);
    sd->th_m_prev = th_m;
    esp_foc_critical_enter();
    sd->pos_raw += d;
    sd->th_m = th_m;
    esp_foc_critical_leave();
}

/* One encoder sample: read, PLL, unwrapped position, then the slot is told
 * a fresh sample is waiting. */
static void sd_fetch(sd_t *sd)
{
    esp_foc_rotor_sensor_t *rot = sd->rotor;
    const esp_err_t err = esp_foc_rotor_sensor_fetch(rot);
    sd->fetch_n++;
    if (err != ESP_OK) {
        sd->fetch_fail++;
        if ((++sd->fail_run >= sd->cfg.guard.sensor_fail_max) && sd->guards_on) {
            sd->abort = ESP_FOC_SD_ABORT_SENSOR_FAIL;
            sd->run = false;
            sd->inv->set_duties(sd->inv, Q16_HALF, Q16_HALF, Q16_HALF);
        }
    } else {
        sd->fail_run = 0;
    }
    esp_foc_rotor_state_t st;
    esp_foc_rotor_sensor_snapshot(rot, &st);
    const bool moved = st.valid && (!sd->have_seq || (st.seq != sd->seq_last));
    if (moved) {
        sd->have_seq = true;
        sd->seq_last = st.seq;
    }
    if (!sd->pll_on) {
        if (moved) {
            sd_unwrap(sd, st.theta_m);
        }
        return;
    }
    esp_foc_rotor_pll_update(&sd->pll, rot);
    if (!moved) {
        return;
    }
    sd_unwrap(sd, st.theta_m);
    const q16_t w = esp_foc_rotor_pll_get_omega(&sd->pll);
    sd->fresh = true;
    sd_ripple_add(sd, w);
    sd_window_add(sd, w);
    sd_inpos_add(sd, w);
}

/* Bridge off there is no TEZ to pace the fetch; the position keeps being
 * unwrapped at the poll rate so a shaft turned by hand is not lost. */
static void sd_service(sd_t *sd)
{
    if (sd->fetch_req) {
        sd->fetch_req = false;
        sd_fetch(sd);
        return;
    }
    if (!sd->bridge_on) {
        const uint64_t now = esp_foc_now_us();
        if (now - sd->idle_fetch_us >= (uint64_t)SD_POLL_MS * 1000u) {
            sd->idle_fetch_us = now;
            sd_fetch(sd);
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Supervisor helpers                                                        */
/* ------------------------------------------------------------------------ */

static esp_foc_sensored_event_t sd_event(sd_t *sd, esp_foc_sensored_ev_t ev)
{
    esp_foc_sensored_event_t e;
    memset(&e, 0, sizeof(e));
    e.ev = ev;
    e.axis = sd->axis;
    e.state = sd->state;
    e.mode = (esp_foc_sensored_mode_t)sd->mode;
    return e;
}

static void sd_emit(sd_t *sd, esp_foc_sensored_event_t *e)
{
    if (sd->cfg.on_event != NULL) {
        sd->cfg.on_event(sd->cfg.ctx, e);
    }
}

static void sd_emit_simple(sd_t *sd, esp_foc_sensored_ev_t ev)
{
    esp_foc_sensored_event_t e = sd_event(sd, ev);
    sd_emit(sd, &e);
}

static sd_out_t sd_pending(sd_t *sd)
{
    if (sd->fault != (uint8_t)ESP_FOC_FAULT_NONE) {
        return SD_FAULT;
    }
    if (sd->abort != (uint8_t)ESP_FOC_SD_ABORT_NONE) {
        return SD_ABORT;
    }
    const uint32_t req = sd->req;
    if ((req & SD_REQ_EXIT) != 0u) {
        return SD_EXIT;
    }
    if ((req & SD_REQ_STOP) != 0u) {
        return SD_STOP;
    }
    if ((sd->learn_deadline_us != 0u) && (esp_foc_now_us() >= sd->learn_deadline_us)) {
        return SD_TIMEOUT;
    }
    return SD_OK;
}

/* Every wait keeps feeding the encoder: the slot runs on its samples. */
static sd_out_t sd_wait(sd_t *sd, uint32_t ms)
{
    const uint64_t end = esp_foc_now_us() + (uint64_t)ms * 1000u;
    for (;;) {
        sd_service(sd);
        const sd_out_t o = sd_pending(sd);
        if (o != SD_OK) {
            return o;
        }
        const uint64_t now = esp_foc_now_us();
        if (now >= end) {
            return SD_OK;
        }
        uint32_t left = (uint32_t)((end - now + 999u) / 1000u);
        (void)esp_foc_event_wait_ms((left < SD_POLL_MS) ? left : SD_POLL_MS);
    }
}

static void sd_cut(sd_t *sd)
{
    esp_foc_inverter_t *inv = sd->inv;

    esp_foc_critical_enter();
    sd->run = false;
    sd->slot_on = false;
    sd->guards_on = false;
    sd->pll_on = false;
    sd->fresh = false;
    sd->map_dir = -1;
    sd->learn_dth = 0;
    sd->w_ff = 0;
    sd->iq_ref = 0;
    sd->w_ref = 0;
    sd->w_target = 0;
    esp_foc_critical_leave();
    sd->rip_budget = 0u;

    if (sd->bridge_on) {
        inv->set_sense_watchdog(inv, false);
        inv->set_duties(inv, Q16_HALF, Q16_HALF, Q16_HALF);
        inv->disable(inv);
        sd->bridge_on = false;
        sd->idle_fetch_us = esp_foc_now_us();
        sd_emit_simple(sd, ESP_FOC_SD_EV_CUT);
    }
}

static void sd_latch(sd_t *sd, sd_out_t out)
{
    esp_foc_sensored_event_t e = sd_event(sd, ESP_FOC_SD_EV_RUN_FAIL);
    if (out == SD_ABORT) {
        e.ev = ESP_FOC_SD_EV_ABORT;
        e.abort = (esp_foc_sensored_abort_t)sd->abort;
        sd->abort_seen = e.abort;
    } else if (out == SD_FAULT) {
        e.ev = ESP_FOC_SD_EV_FAULT;
        e.fault = (esp_foc_fault_reason_t)sd->fault;
        sd->fault_seen = e.fault;
    } else {
        e.fail = sd->fail;
    }
    sd->latched = (out == SD_ABORT) || (out == SD_FAULT);
    sd->state = sd->latched ? ESP_FOC_SD_STATE_FAULT : ESP_FOC_SD_STATE_IDLE;
    e.state = sd->state;
    sd_emit(sd, &e);
}

static void sd_req_done(sd_t *sd, uint32_t bit, esp_err_t err)
{
    sd->req_err = err;
    esp_foc_critical_enter();
    sd->req &= ~bit;
    esp_foc_critical_leave();
}

static esp_err_t sd_request(sd_t *sd, uint32_t bit, uint32_t timeout_ms)
{
    esp_foc_critical_enter();
    sd->req |= bit;
    esp_foc_critical_leave();
    esp_foc_event_post(sd->sup);
    if (esp_foc_event_handle_self() == sd->sup) {
        return ESP_OK;
    }
    for (uint32_t t = 0; t < timeout_ms; t++) {
        if ((sd->req & bit) == 0u) {
            return sd->req_err;
        }
        esp_foc_sleep_ms(1);
    }
    return ESP_ERR_TIMEOUT;
}

/* ------------------------------------------------------------------------ */
/* Design                                                                    */
/* ------------------------------------------------------------------------ */

static q16_t sd_step_q16(float per_s, float slot_hz)
{
    const q16_t step = q16_from_float(per_s / slot_hz);
    return (step > 0) ? step : 1;
}

static void sd_speed_gains(sd_t *sd, float kp, float ki)
{
    const int64_t kp_q32 = (int64_t)((double)kp * SD_Q32 + 0.5);
    const int64_t ki_q32 = (int64_t)((double)ki / (double)sd->slot_hz * SD_Q32 + 0.5);
    esp_foc_critical_enter();
    sd->kp_w_q32 = kp_q32;
    sd->ki_ts_q32 = ki_q32;
    esp_foc_critical_leave();
    sd->tune.kp_w = kp;
    sd->tune.ki_w = ki;
}

static void sd_position_kp(sd_t *sd, float kp)
{
    sd->kp_pos = q16_from_float(kp);
    sd->tune.kp_pos = kp;
}

/* Residual about a least-squares line, so a coast-down or a nudge during the
 * window is not counted as feedback noise. */
static float sd_ripple_sigma(sd_t *sd)
{
    const uint32_t n = sd->rip_n;
    if (n < 64u) {
        return 0.0f;
    }
    const double dn = (double)n;
    const double sx = (double)sd->rip_sum;
    const double sxx_c = (double)sd->rip_sq - sx * sx / dn;
    const double sii_c = dn * (dn * dn - 1.0) / 12.0;
    const double sxi_c = (double)sd->rip_ix - 0.5 * (dn - 1.0) * sx;
    double rss = sxx_c - ((sii_c > 0.0) ? (sxi_c * sxi_c / sii_c) : 0.0);
    if (rss < 0.0) {
        rss = 0.0;
    }
    return (float)(sqrt(rss / (dn - 2.0)) * (double)(1u << SD_RIP_SHIFT) / 65536.0);
}

/*
 * Both ceilings bound the crossover fc = 2·zeta·bw, not bw. Noise: Kp may
 * spend fb_iq_budget_a on one sigma of feedback, so K cancels out of Kp and
 * a wrong K moves only Ki and zeta. Phase: the PLL group delay is a pole at
 * about pll_bw/2, kept pll_sep_min above fc.
 */
static esp_err_t sd_speed_design(sd_t *sd)
{
    const esp_foc_sensored_config_t *c = &sd->cfg;
    const float two_z = 2.0f * c->speed_zeta;
    esp_foc_sensored_tuning_t *t = &sd->tune;

    float want = c->speed_bw_hz;
    if (!(want > 0.0f)) {
        want = c->speed_bw_frac * t->f_base_hz;
        want = fminf(fmaxf(want, c->speed_bw_min_hz), c->speed_bw_max_hz);
    }
    float bw = want;
    float noise = 0.0f;
    if (t->ripple_rads > 0.0f) {
        noise = (c->fb_iq_budget_a / t->ripple_rads) * c->k_rads2_a / (two_z * SD_TWO_PI);
        bw = fminf(bw, noise);
    }
    const float phase = 0.5f * c->pll_bw_hz / c->pll_sep_min / two_z;
    bw = fminf(bw, phase);

    float kp = c->kp_w;
    float ki = c->ki_w;
    if ((kp == 0.0f) && (ki == 0.0f)) {
        const esp_err_t err =
            esp_foc_pid_design_integrator(c->k_rads2_a, sd->slot_hz, bw, c->speed_zeta, &kp, &ki);
        if (err != ESP_OK) {
            return err;
        }
    }
    sd_speed_gains(sd, kp, ki);
    t->speed_bw_want_hz = want;
    t->speed_bw_noise_hz = noise;
    t->speed_bw_phase_hz = phase;
    t->speed_bw_hz = bw;
    t->speed_fc_hz = two_z * bw;
    sd_position_kp(sd, (c->kp_pos > 0.0f) ? c->kp_pos : SD_TWO_PI * t->speed_fc_hz / c->pos_sep);
    return ESP_OK;
}

static bool sd_cfg_ok(const esp_foc_sensored_config_t *c)
{
    const bool plant = (c->rs_ohm > 0.0f) && (c->ls_h > 0.0f) && (c->psi_wb > 0.0f) &&
                       (c->pole_pairs > 0u) && ((int)c->control >= 0) &&
                       (c->control <= ESP_FOC_SD_CONTROL_POSITION) &&
                       ((c->control == ESP_FOC_SD_CONTROL_TORQUE) || (c->k_rads2_a > 0.0f)) &&
                       (c->pwm_delay_ts >= 0.0f) && (fabsf(c->park_lead_s) < 0.01f);
    const bool sense = (c->fetch_hz > 0u) && (c->pll_bw_hz > 0.0f) && (c->pll_zeta > 0.0f);
    const bool cur = (c->i_max_a > 0.0f) && (c->i_bw_hz > 0.0f) && (c->i_tune_backoff > 0.0f) &&
                     (c->i_tune_backoff <= 1.0f) && (c->kp_i >= 0.0f) && (c->ki_i >= 0.0f);
    const bool speed = (c->speed_bw_hz >= 0.0f) && (c->speed_bw_frac > 0.0f) &&
                       (c->speed_bw_min_hz > 0.0f) &&
                       (c->speed_bw_max_hz >= c->speed_bw_min_hz) && (c->speed_zeta > 0.4f) &&
                       (c->speed_zeta < 3.0f) && (c->fb_iq_budget_a > 0.0f) &&
                       (c->pll_sep_min >= 1.0f) && (c->ripple_ms > 0u) && (c->still_hz > 0.0f) &&
                       (c->still_timeout_ms > 0u) && (c->kp_w >= 0.0f) && (c->ki_w >= 0.0f) &&
                       (c->wref_slew_hz_s > 0.0f);
    const bool pos = (c->pos_sep >= 1.0f) && (c->kp_pos >= 0.0f) && (c->corr_max_hz > 0.0f) &&
                     (c->wm_max_hz > 0.0f) && (c->inpos_rad > 0.0f) && (c->inpos_w_hz > 0.0f) &&
                     (c->inpos_ms > 0u) &&
                     (SD_TWO_PI * c->wm_max_hz * (float)c->pole_pairs < 30000.0f);
    const bool guard = (c->guard.overspeed_hz > 0.0f) &&
                       (SD_TWO_PI * c->guard.overspeed_hz < 15000.0f) &&
                       (c->guard.sensor_stale_slots >= 2u) && (c->guard.sensor_fail_max >= 1u);
    const bool cog = (c->cogging.sweep_hz > 0.0f) && (c->cogging.sweep_hz <= c->wm_max_hz) &&
                     (c->cogging.revs > 0.0f) && (c->cogging.passes_max >= 1u) &&
                     (c->cogging.conv_a > 0.0f) && ((c->cogging.smooth & 1u) == 1u) &&
                     (c->cogging.smooth < SD_BINS);
    return plant && sense && cur && speed && pos && guard && cog;
}

static esp_err_t sd_design(sd_t *sd)
{
    const esp_foc_sensored_config_t *c = &sd->cfg;
    const float pwm = (float)sd->pwm_hz;
    esp_err_t err;

    sd->slot_hz = (float)c->fetch_hz;
    sd->slow_div = sd->pwm_hz / c->fetch_hz;

    float kp_i = c->kp_i;
    float ki_i = c->ki_i;
    if ((kp_i == 0.0f) && (ki_i == 0.0f)) {
        err = esp_foc_pid_design_imc_zoh(sd->vdc_v / c->rs_ohm, c->ls_h / c->rs_ohm, pwm,
                                         c->i_bw_hz, &kp_i, &ki_i);
        if (err != ESP_OK) {
            return err;
        }
        kp_i *= c->i_tune_backoff;
        ki_i *= c->i_tune_backoff;
    }
    err = esp_foc_pid_init(&sd->pi_d, kp_i, ki_i, 0.0f, 0.0f, 1.0f / pwm);
    if (err == ESP_OK) {
        err = esp_foc_pid_init(&sd->pi_q, kp_i, ki_i, 0.0f, 0.0f, 1.0f / pwm);
    }
    if (err != ESP_OK) {
        return err;
    }

    esp_foc_rotor_pll_config_t pc;
    esp_foc_rotor_pll_config_default(&pc, c->fetch_hz, c->pll_bw_hz, c->pll_zeta,
                                     2.0f * SD_TWO_PI * c->guard.overspeed_hz);
    pc.domain = ESP_FOC_ROTOR_PLL_ELEC;
    err = esp_foc_rotor_pll_init(&sd->pll, &pc);
    if (err != ESP_OK) {
        return err;
    }

    sd->tune.kp_i = kp_i;
    sd->tune.ki_i = ki_i;
    sd->tune.slot_hz = sd->slot_hz;
    sd->tune.f_base_hz = sd->vdc_v * SD_INV_SQRT3 / (SD_TWO_PI * c->psi_wb);

    sd->th_off = q16_from_float(c->park_offset_rad);
    sd->tau_q32 = (int32_t)lrintf(c->park_lead_s * (float)SD_Q32);
    sd->vlead_q32 = (int32_t)lrintf(c->pwm_delay_ts / pwm * (float)SD_Q32);

    sd->i_max = q16_from_float(c->i_max_a);
    sd->wi_lim_q32 = (int64_t)sd->i_max << 16;
    sd->w_step = sd_step_q16(SD_TWO_PI * c->wref_slew_hz_s, sd->slot_hz);
    sd->corr_max = q16_from_float(SD_TWO_PI * c->corr_max_hz);
    sd->wm_max = q16_from_float(SD_TWO_PI * c->wm_max_hz);
    sd->pp = (int32_t)c->pole_pairs;
    sd->cog_k = q16_from_float((float)SD_BINS / SD_TWO_PI);
    sd->g_w_trip = q16_from_float(SD_TWO_PI * c->guard.overspeed_hz);
    sd->g_ov_need = (uint32_t)((float)c->guard.overspeed_hold_ms * sd->slot_hz * 0.001f + 0.5f);
    if (sd->g_ov_need == 0u) {
        sd->g_ov_need = 1u;
    }
    sd->stale_limit = c->guard.sensor_stale_slots * sd->slow_div;
    sd->inpos_band = q16_from_float(c->inpos_rad);
    sd->inpos_w = q16_from_float(SD_TWO_PI * c->inpos_w_hz);
    sd->inpos_need = (uint32_t)((float)c->inpos_ms * sd->slot_hz * 0.001f + 0.5f);
    if (c->kp_pos > 0.0f) {
        sd_position_kp(sd, c->kp_pos);
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */
/* Modes                                                                     */
/* ------------------------------------------------------------------------ */

/* Under the critical section: the slot must see the whole switch at once. */
static void sd_enter_mode(sd_t *sd, esp_foc_sensored_mode_t mode)
{
    const q16_t iq = sd->iq_ref;
    const q16_t w = esp_foc_rotor_pll_get_omega(&sd->pll);
    switch (mode) {
    case ESP_FOC_SD_MODE_TORQUE:
        sd->iq_user = iq;
        break;
    case ESP_FOC_SD_MODE_VELOCITY:
        sd->w_ref = w;
        sd->w_target = w;
        sd->wi_q32 = (int64_t)q16_sub(iq, sd->iq_user) << 16;
        break;
    case ESP_FOC_SD_MODE_POSITION:
        sd->ref_abs = sd->pos_raw;
        sd->w_ff = 0;
        sd->w_ref = w;
        sd->wi_q32 = (int64_t)q16_sub(iq, sd->iq_user) << 16;
        break;
    default:
        break;
    }
    sd->mode = (uint8_t)mode;
}

/* ------------------------------------------------------------------------ */
/* State machine                                                             */
/* ------------------------------------------------------------------------ */

static sd_out_t sd_wait_still(sd_t *sd)
{
    const q16_t still = q16_from_float(SD_TWO_PI * sd->cfg.still_hz);
    uint32_t calm = 0;
    for (uint32_t t = 0; t < sd->cfg.still_timeout_ms; t += SD_POLL_MS) {
        const sd_out_t o = sd_wait(sd, SD_POLL_MS);
        if (o != SD_OK) {
            return o;
        }
        calm = (sd_abs(esp_foc_rotor_pll_get_omega(&sd->pll)) < still) ? calm + SD_POLL_MS : 0u;
        if (calm >= SD_STILL_HOLD_MS) {
            return SD_OK;
        }
    }
    sd->fail = ESP_FOC_SD_FAIL_NOT_STILL;
    return SD_FAILED;
}

static sd_out_t sd_measure_ripple(sd_t *sd)
{
    sd->rip_n = 0u;
    sd->rip_sum = 0;
    sd->rip_sq = 0;
    sd->rip_ix = 0;
    sd->rip_budget = (uint32_t)((float)sd->cfg.ripple_ms * sd->slot_hz * 0.001f);
    const sd_out_t o = sd_wait(sd, sd->cfg.ripple_ms + 2u * SD_POLL_MS);
    sd->rip_budget = 0u;
    sd->tune.ripple_rads = sd_ripple_sigma(sd);
    return o;
}

static sd_out_t sd_start(sd_t *sd)
{
    esp_foc_inverter_t *inv = sd->inv;
    const esp_foc_sensored_config_t *c = &sd->cfg;

    sd->fail = ESP_FOC_SD_FAIL_NONE;
    sd->state = ESP_FOC_SD_STATE_ARMED;
    sd_emit_simple(sd, ESP_FOC_SD_EV_ARMED);

    inv->set_duties(inv, Q16_HALF, Q16_HALF, Q16_HALF);
    if (inv->enable(inv) != ESP_OK) {
        sd->fail = ESP_FOC_SD_FAIL_ENABLE;
        return SD_FAILED;
    }
    sd->bridge_on = true;
    /* The DMA hook clears sample_ready, which calibrate waits on. */
    inv->set_dma_callback(inv, NULL, NULL);
    inv->calibrate_currents(inv, SD_CAL_ROUNDS);
    inv->set_dma_callback(inv, sd_dma, sd);
    inv->set_duties(inv, Q16_HALF, Q16_HALF, Q16_HALF);
    sd_out_t o = sd_wait(sd, SD_ARM_MS);
    if (o != SD_OK) {
        return o;
    }

    esp_foc_rotor_state_t st;
    esp_foc_rotor_sensor_snapshot(sd->rotor, &st);
    esp_foc_critical_enter();
    esp_foc_pid_reset(&sd->pi_d);
    esp_foc_pid_reset(&sd->pi_q);
    esp_foc_rotor_pll_seed(&sd->pll, st.theta_e, 0);
    sd->wi_q32 = 0;
    sd->iq_ref = 0;
    sd->w_ref = 0;
    sd->w_target = 0;
    sd->w_ff = 0;
    sd->g_ov_n = 0;
    sd->since_fresh = 0;
    sd->fail_run = 0;
    sd->mode = (uint8_t)c->control;
    sd->ref_abs = sd->pos_raw;
    sd->pll_on = true;
    sd->run = true;
    sd->guards_on = true;
    esp_foc_critical_leave();
    inv->set_sense_watchdog(inv, true);

    if (c->control != ESP_FOC_SD_CONTROL_TORQUE) {
        o = sd_wait_still(sd);
        if (o == SD_OK) {
            o = sd_measure_ripple(sd);
        }
        if (o != SD_OK) {
            return o;
        }
        if (sd_speed_design(sd) != ESP_OK) {
            sd->fail = ESP_FOC_SD_FAIL_DESIGN;
            return SD_FAILED;
        }
        esp_foc_critical_enter();
        sd_enter_mode(sd, (esp_foc_sensored_mode_t)c->control);
        sd->slot_on = true;
        esp_foc_critical_leave();
    } else {
        esp_foc_critical_enter();
        sd->slot_on = true;
        esp_foc_critical_leave();
    }
    sd->state = ESP_FOC_SD_STATE_RUNNING;
    sd_emit_simple(sd, ESP_FOC_SD_EV_RUNNING);
    return SD_OK;
}

/* ------------------------------------------------------------------------ */
/* Cogging learn                                                             */
/* ------------------------------------------------------------------------ */

static sd_out_t sd_sweep_leg(sd_t *sd, int8_t dir)
{
    const esp_foc_sensored_config_t *c = &sd->cfg;
    const float w = ((dir == 0) ? SD_TWO_PI : -SD_TWO_PI) * c->cogging.sweep_hz;
    const q16_t dth = q16_from_float(w / sd->slot_hz);
    const uint32_t ms = (uint32_t)(1000.0f * c->cogging.revs / c->cogging.sweep_hz);

    esp_foc_critical_enter();
    sd->ref_abs = sd->pos_raw;
    sd->w_ff = q16_from_float(w);
    sd->learn_dth = (dth != 0) ? dth : ((dir == 0) ? 1 : -1);
    esp_foc_critical_leave();
    sd_out_t o = sd_wait(sd, c->cogging.skip_ms);
    if (o == SD_OK) {
        sd->map_dir = dir;
        o = sd_wait(sd, ms);
    }
    esp_foc_critical_enter();
    sd->map_dir = -1;
    sd->learn_dth = 0;
    sd->w_ff = 0;
    esp_foc_critical_leave();
    if (o != SD_OK) {
        return o;
    }
    return sd_wait(sd, c->cogging.rest_ms);
}

/*
 * (+map + −map)/2 per bin, gaps filled linearly, mean removed, box-smoothed.
 * The averaged map is kept in map_sum[0] and the smoothed table in
 * map_sum[1], so the build needs no scratch beyond the sweep buffers.
 */
static bool sd_cog_build(sd_t *sd, esp_foc_sensored_cogging_info_t *info)
{
    int32_t *avg = sd->map_sum[0];
    int32_t *out = sd->map_sum[1];
    uint16_t *have = sd->map_n[0];
    const uint16_t *n1 = sd->map_n[1];
    uint32_t mapped = 0;

    for (uint32_t i = 0; i < SD_BINS; i++) {
        const bool ok = (have[i] > 0u) && (n1[i] > 0u);
        if (ok) {
            const int32_t a = avg[i] / (int32_t)have[i];
            const int32_t b = out[i] / (int32_t)n1[i];
            avg[i] = (int32_t)(((int64_t)a + (int64_t)b) / 2);
            mapped++;
        }
        have[i] = ok ? 1u : 0u;
    }
    info->bins = SD_BINS;
    info->bins_mapped = mapped;
    if (mapped < (SD_BINS / 2u)) {
        return false;
    }
    for (uint32_t i = 0; i < SD_BINS; i++) {
        if (have[i] != 0u) {
            continue;
        }
        uint32_t lo = 1;
        uint32_t hi = 1;
        while (have[(i + SD_BINS - lo) % SD_BINS] == 0u) {
            lo++;
        }
        while (have[(i + hi) % SD_BINS] == 0u) {
            hi++;
        }
        const int64_t a = avg[(i + SD_BINS - lo) % SD_BINS];
        const int64_t b = avg[(i + hi) % SD_BINS];
        avg[i] = (int32_t)(a + ((b - a) * (int64_t)lo) / (int64_t)(lo + hi));
    }
    int64_t mean = 0;
    for (uint32_t i = 0; i < SD_BINS; i++) {
        mean += avg[i];
    }
    mean /= (int64_t)SD_BINS;

    const uint32_t w = sd->cfg.cogging.smooth;
    int64_t acc = 0;
    for (uint32_t k = 0; k < w; k++) {
        acc += avg[(SD_BINS + k - (w / 2u)) % SD_BINS];
    }
    int64_t sq = 0;
    q16_t lo_v = 0;
    q16_t hi_v = 0;
    for (uint32_t i = 0; i < SD_BINS; i++) {
        const q16_t v = (q16_t)(acc / (int64_t)w - mean);
        out[i] = v;
        const int64_t d = (int64_t)v - (int64_t)sd->cog[i];
        sq += d * d;
        lo_v = q16_min(lo_v, v);
        hi_v = q16_max(hi_v, v);
        acc += avg[(i + w - (w / 2u)) % SD_BINS];
        acc -= avg[(i + SD_BINS - (w / 2u)) % SD_BINS];
    }
    /* Word stores: the slot may read a mix of two passes for one sample,
     * never a torn entry. */
    for (uint32_t i = 0; i < SD_BINS; i++) {
        sd->cog[i] = out[i];
    }
    sd->cog_valid = true;
    sd->cog_on = true;
    info->change_rms_a = (float)(sqrt((double)sq / (double)SD_BINS) / 65536.0);
    info->p2p_a = q16_to_float(q16_sub(hi_v, lo_v));
    return true;
}

static sd_out_t sd_learn(sd_t *sd, esp_err_t *err)
{
    const esp_foc_sensored_config_t *c = &sd->cfg;
    esp_foc_sensored_cogging_info_t info;
    memset(&info, 0, sizeof(info));
    sd_out_t o = SD_OK;

    esp_foc_critical_enter();
    sd->iq_user = 0;
    esp_foc_critical_leave();
    *err = ESP_OK;
    for (uint32_t pass = 1; pass <= c->cogging.passes_max; pass++) {
        memset(sd->map_sum, 0, sizeof(int32_t) * 2u * SD_BINS);
        memset(sd->map_n, 0, sizeof(uint16_t) * 2u * SD_BINS);
        o = sd_sweep_leg(sd, 0);
        if (o == SD_OK) {
            o = sd_sweep_leg(sd, 1);
        }
        if (o != SD_OK) {
            break;
        }
        info.passes = pass;
        if (!sd_cog_build(sd, &info)) {
            *err = ESP_FAIL;
            break;
        }
        esp_foc_sensored_event_t e = sd_event(sd, ESP_FOC_SD_EV_LEARN_PASS);
        e.pass = (uint8_t)pass;
        e.change_rms_a = info.change_rms_a;
        sd_emit(sd, &e);
        if ((pass >= 2u) && (info.change_rms_a <= c->cogging.conv_a)) {
            info.converged = true;
            break;
        }
    }
    sd->cog_info = info;
    esp_foc_critical_enter();
    sd->ref_abs = sd->pos_raw;
    esp_foc_critical_leave();

    esp_foc_sensored_event_t e = sd_event(sd, ESP_FOC_SD_EV_LEARN_DONE);
    e.pass = (uint8_t)info.passes;
    e.change_rms_a = info.change_rms_a;
    if ((o != SD_OK) || (*err != ESP_OK)) {
        e.ev = ESP_FOC_SD_EV_LEARN_FAIL;
        if (o == SD_TIMEOUT) {
            *err = ESP_ERR_TIMEOUT;
            o = SD_OK;
        } else if (*err == ESP_OK) {
            *err = (o == SD_STOP) ? ESP_ERR_INVALID_STATE : ESP_FAIL;
        }
    }
    sd_emit(sd, &e);
    return o;
}

/* ------------------------------------------------------------------------ */
/* Supervisor                                                                */
/* ------------------------------------------------------------------------ */

static void sd_finish(sd_t *sd, sd_out_t out)
{
    switch (out) {
    case SD_STOP:
    case SD_EXIT:
        sd_cut(sd);
        sd->abort = (uint8_t)ESP_FOC_SD_ABORT_NONE;
        if (sd->state != ESP_FOC_SD_STATE_IDLE) {
            sd->state = ESP_FOC_SD_STATE_IDLE;
            sd_emit_simple(sd, ESP_FOC_SD_EV_STOPPED);
        }
        break;
    case SD_FAILED:
    case SD_ABORT:
    case SD_FAULT:
        sd_cut(sd);
        sd_latch(sd, out);
        break;
    default:
        break;
    }
}

static esp_err_t sd_clear(sd_t *sd)
{
    esp_foc_inverter_t *inv = sd->inv;
    if (!sd->latched) {
        return ESP_ERR_INVALID_STATE;
    }
    if (inv->is_faulted(inv)) {
        const esp_err_t err = inv->clear_fault(inv);
        if (err != ESP_OK) {
            return err;
        }
    }
    esp_foc_critical_enter();
    sd->fault = (uint8_t)ESP_FOC_FAULT_NONE;
    sd->abort = (uint8_t)ESP_FOC_SD_ABORT_NONE;
    esp_foc_critical_leave();
    sd->fail = ESP_FOC_SD_FAIL_NONE;
    sd->abort_seen = ESP_FOC_SD_ABORT_NONE;
    sd->fault_seen = ESP_FOC_FAULT_NONE;
    sd->latched = false;
    sd->state = ESP_FOC_SD_STATE_IDLE;
    sd_emit_simple(sd, ESP_FOC_SD_EV_FAULT_CLEARED);
    return ESP_OK;
}

static void sd_do_run(sd_t *sd)
{
    if ((sd->state != ESP_FOC_SD_STATE_IDLE) || sd->latched) {
        sd_req_done(sd, SD_REQ_RUN, ESP_ERR_INVALID_STATE);
        return;
    }
    const sd_out_t o = sd_start(sd);
    if (o != SD_OK) {
        sd_finish(sd, o);
    }
    sd_req_done(sd, SD_REQ_RUN, (o == SD_OK) ? ESP_OK : ESP_FAIL);
}

static void sd_do_learn(sd_t *sd)
{
    if ((sd->state != ESP_FOC_SD_STATE_RUNNING) || (sd->mode != ESP_FOC_SD_MODE_POSITION)) {
        sd_req_done(sd, SD_REQ_LEARN, ESP_ERR_INVALID_STATE);
        return;
    }
    sd->state = ESP_FOC_SD_STATE_LEARNING;
    sd->learn_deadline_us = esp_foc_now_us() + (uint64_t)sd->learn_timeout_ms * 1000u;
    esp_err_t err = ESP_OK;
    const sd_out_t o = sd_learn(sd, &err);
    sd->learn_deadline_us = 0u;
    if (o == SD_OK) {
        sd->state = ESP_FOC_SD_STATE_RUNNING;
    } else {
        sd_finish(sd, o);
    }
    sd_req_done(sd, SD_REQ_LEARN, err);
}

static void sd_task(void *arg)
{
    sd_t *sd = (sd_t *)arg;
    sd->sup = esp_foc_event_handle_self();
    sd->task_alive = true;

    for (;;) {
        sd_service(sd);
        const uint32_t req = sd->req;
        if ((req & SD_REQ_EXIT) != 0u) {
            break;
        }
        if ((req & SD_REQ_STOP) != 0u) {
            if (!sd->latched) {
                sd_finish(sd, SD_STOP);
            }
            sd_req_done(sd, SD_REQ_STOP, ESP_OK);
            continue;
        }
        if (sd->bridge_on) {
            const sd_out_t o = sd_pending(sd);
            if ((o == SD_ABORT) || (o == SD_FAULT)) {
                sd_finish(sd, o);
                continue;
            }
        }
        if ((req & SD_REQ_CLEAR) != 0u) {
            sd_req_done(sd, SD_REQ_CLEAR, sd_clear(sd));
            continue;
        }
        if ((req & SD_REQ_RUN) != 0u) {
            sd_do_run(sd);
            continue;
        }
        if ((req & SD_REQ_LEARN) != 0u) {
            sd_do_learn(sd);
            continue;
        }
        (void)esp_foc_event_wait_ms(SD_POLL_MS);
    }

    sd_cut(sd);
    sd->task_alive = false;
    esp_foc_task_delete_self();
}

/* ------------------------------------------------------------------------ */
/* Public API                                                                */
/* ------------------------------------------------------------------------ */

void esp_foc_sensored_default_config(esp_foc_sensored_config_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));
    cfg->control = ESP_FOC_SD_CONTROL_POSITION;
    cfg->pwm_delay_ts = (float)CONFIG_ESP_FOC_SD_PWM_DELAY_PERMIL * 1.0e-3f;
    cfg->fetch_hz = CONFIG_ESP_FOC_SD_FETCH_HZ;
    cfg->pll_bw_hz = (float)CONFIG_ESP_FOC_SD_PLL_BW_HZ;
    cfg->pll_zeta = (float)CONFIG_ESP_FOC_SD_PLL_ZETA_PERMIL * 1.0e-3f;
    cfg->i_max_a = (float)CONFIG_ESP_FOC_SD_I_MAX_MA * 1.0e-3f;
    cfg->i_bw_hz = (float)CONFIG_ESP_FOC_SD_I_BW_HZ;
    cfg->i_tune_backoff = (float)CONFIG_ESP_FOC_SD_I_BACKOFF_PERMIL * 1.0e-3f;
    cfg->speed_bw_frac = (float)CONFIG_ESP_FOC_SD_SPEED_BW_E4 * 1.0e-4f;
    cfg->speed_bw_min_hz = (float)CONFIG_ESP_FOC_SD_SPEED_BW_MIN_HZ;
    cfg->speed_bw_max_hz = (float)CONFIG_ESP_FOC_SD_SPEED_BW_MAX_HZ;
    cfg->speed_zeta = (float)CONFIG_ESP_FOC_SD_SPEED_ZETA_PERMIL * 1.0e-3f;
    cfg->fb_iq_budget_a = (float)CONFIG_ESP_FOC_SD_FB_IQ_BUDGET_MA * 1.0e-3f;
    cfg->pll_sep_min = (float)CONFIG_ESP_FOC_SD_PLL_SEP_PERMIL * 1.0e-3f;
    cfg->ripple_ms = CONFIG_ESP_FOC_SD_RIPPLE_MS;
    cfg->still_hz = (float)CONFIG_ESP_FOC_SD_STILL_HZ;
    cfg->still_timeout_ms = CONFIG_ESP_FOC_SD_STILL_TIMEOUT_MS;
    cfg->wref_slew_hz_s = (float)CONFIG_ESP_FOC_SD_WREF_SLEW_HZ_S;
    cfg->pos_sep = (float)CONFIG_ESP_FOC_SD_POS_SEP_PERMIL * 1.0e-3f;
    cfg->corr_max_hz = (float)CONFIG_ESP_FOC_SD_CORR_MAX_MHZ * 1.0e-3f;
    cfg->wm_max_hz = (float)CONFIG_ESP_FOC_SD_WM_MAX_HZ;
    cfg->inpos_rad = (float)CONFIG_ESP_FOC_SD_INPOS_MDEG * 1.0e-3f * (SD_TWO_PI / 360.0f);
    cfg->inpos_w_hz = (float)CONFIG_ESP_FOC_SD_INPOS_W_HZ;
    cfg->inpos_ms = CONFIG_ESP_FOC_SD_INPOS_MS;
    cfg->guard.overspeed_hz = (float)CONFIG_ESP_FOC_SD_OVERSPEED_HZ;
    cfg->guard.overspeed_hold_ms = CONFIG_ESP_FOC_SD_OVERSPEED_HOLD_MS;
    cfg->guard.sensor_stale_slots = CONFIG_ESP_FOC_SD_STALE_SLOTS;
    cfg->guard.sensor_fail_max = CONFIG_ESP_FOC_SD_FAIL_MAX;
    cfg->cogging.sweep_hz = (float)CONFIG_ESP_FOC_SD_COG_SWEEP_MHZ * 1.0e-3f;
    cfg->cogging.revs = (float)CONFIG_ESP_FOC_SD_COG_REVS_PERMIL * 1.0e-3f;
    cfg->cogging.passes_max = CONFIG_ESP_FOC_SD_COG_PASSES_MAX;
    cfg->cogging.conv_a = (float)CONFIG_ESP_FOC_SD_COG_CONV_UA * 1.0e-6f;
    cfg->cogging.smooth = CONFIG_ESP_FOC_SD_COG_SMOOTH;
    cfg->cogging.skip_ms = CONFIG_ESP_FOC_SD_COG_SKIP_MS;
    cfg->cogging.rest_ms = CONFIG_ESP_FOC_SD_COG_REST_MS;
    esp_foc_phase_map_identity(&cfg->map);
}

#if CONFIG_ESP_FOC_ENABLE_MOTOR_ID
void esp_foc_sensored_config_from_motor_id(esp_foc_sensored_config_t *cfg,
                                           const esp_foc_motor_id_result_t *r)
{
    if ((cfg == NULL) || (r == NULL)) {
        return;
    }
    const uint32_t v = r->valid_mask;
    if ((v & ESP_FOC_MOTOR_ID_VALID_R_LOOP) != 0u) {
        cfg->rs_ohm = r->r_loop_ohm;
    } else if ((v & ESP_FOC_MOTOR_ID_VALID_RS) != 0u) {
        cfg->rs_ohm = r->rs_ohm;
    }
    if ((v & ESP_FOC_MOTOR_ID_VALID_LS) != 0u) {
        cfg->ls_h = r->ls_h;
    }
    if ((v & ESP_FOC_MOTOR_ID_VALID_PSI_F) != 0u) {
        cfg->psi_wb = r->psi_f_wb;
    }
    if ((v & ESP_FOC_MOTOR_ID_VALID_PP) != 0u) {
        cfg->pole_pairs = (uint8_t)r->pole_pairs;
    }
    if ((v & ESP_FOC_MOTOR_ID_VALID_K) != 0u) {
        cfg->k_rads2_a = r->k_rad_s2_per_a;
    }
    if ((v & ESP_FOC_MOTOR_ID_VALID_J) != 0u) {
        cfg->j_kgm2 = r->j_kgm2;
    }
    if ((v & ESP_FOC_MOTOR_ID_VALID_PARK) != 0u) {
        cfg->park_offset_rad = r->park_offset_rad;
        cfg->park_lead_s = r->park_lead_s;
    }
}
#endif

void esp_foc_sensored_config_from_phase_map(esp_foc_sensored_config_t *cfg,
                                            const esp_foc_phase_discover_result_t *r)
{
    if ((cfg == NULL) || (r == NULL) || !esp_foc_phase_map_valid(&r->map)) {
        return;
    }
    cfg->map = r->map;
    cfg->map_valid = true;
}

esp_err_t esp_foc_sensored_init(esp_foc_inverter_t *inv, esp_foc_rotor_sensor_t *rotor,
                                const esp_foc_sensored_config_t *cfg)
{
    if ((inv == NULL) || (rotor == NULL) || (cfg == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    sd_t *sd = sd_axis(cfg->axis);
    if (sd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (sd->inited || sd->task_alive || !esp_foc_in_task_context()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!sd_cfg_ok(cfg) || (cfg->map_valid && !esp_foc_phase_map_valid(&cfg->map))) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint32_t pwm_hz = inv->get_pwm_rate_hz(inv);
    const float vdc = q16_to_float(inv->get_dc_link_voltage(inv));
    if ((pwm_hz == 0u) || !(vdc > 0.0f) || (cfg->fetch_hz > pwm_hz) ||
        ((pwm_hz % cfg->fetch_hz) != 0u)) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(sd, 0, sizeof(*sd));
    sd->axis = cfg->axis;
    sd->inv = inv;
    sd->rotor = rotor;
    sd->cfg = *cfg;
    sd->pwm_hz = pwm_hz;
    sd->vdc_v = vdc;
    sd->state = ESP_FOC_SD_STATE_IDLE;
    sd->mode = (uint8_t)cfg->control;
    sd->map_dir = -1;
    sd->cog = s_cog[sd->axis];
    sd->map_sum = s_map_sum[sd->axis];
    sd->map_n = s_map_n[sd->axis];
    memset(sd->cog, 0, sizeof(q16_t) * SD_BINS);
    esp_err_t err = sd_design(sd);
    if (err != ESP_OK) {
        return err;
    }
    if (cfg->map_valid) {
        err = inv->set_phase_map(inv, &cfg->map);
        if (err != ESP_OK) {
            return err;
        }
    }

    if (esp_foc_task_spawn(sd_task, sd, k_task_name[sd->axis], CONFIG_ESP_FOC_SD_TASK_STACK,
                           esp_foc_task_max_priority() - CONFIG_ESP_FOC_SD_TASK_PRIO_BELOW_MAX,
                           NULL) != 0) {
        return ESP_ERR_NO_MEM;
    }
    while (!sd->task_alive) {
        esp_foc_sleep_ms(1);
    }

    esp_foc_critical_enter();
    inv->set_pwm_callback(inv, sd_tez, sd);
    inv->set_dma_callback(inv, sd_dma, sd);
    inv->set_fault_callback(inv, sd_fault, sd);
    esp_foc_critical_leave();
    sd->inited = true;
    return ESP_OK;
}

void esp_foc_sensored_deinit(uint8_t axis)
{
    sd_t *sd = sd_axis(axis);
    if ((sd == NULL) || !sd->inited) {
        return;
    }
    esp_foc_inverter_t *inv = sd->inv;
    esp_foc_critical_enter();
    sd->req |= SD_REQ_EXIT;
    esp_foc_critical_leave();
    esp_foc_event_post(sd->sup);
    while (sd->task_alive) {
        esp_foc_sleep_ms(1);
    }
    esp_foc_critical_enter();
    inv->set_pwm_callback(inv, NULL, NULL);
    inv->set_dma_callback(inv, NULL, NULL);
    inv->set_fault_callback(inv, NULL, NULL);
    esp_foc_critical_leave();
    sd->inited = false;
}

esp_err_t esp_foc_sensored_run(uint8_t axis)
{
    sd_t *sd = sd_axis(axis);
    if (sd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sd->inited || sd->latched || (sd->state != ESP_FOC_SD_STATE_IDLE)) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint32_t ms = sd->cfg.still_timeout_ms + sd->cfg.ripple_ms + SD_REQ_TIMEOUT_MS;
    return sd_request(sd, SD_REQ_RUN, ms);
}

esp_err_t esp_foc_sensored_stop(uint8_t axis)
{
    sd_t *sd = sd_axis(axis);
    if (sd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sd->inited) {
        return ESP_ERR_INVALID_STATE;
    }
    return sd_request(sd, SD_REQ_STOP, SD_REQ_TIMEOUT_MS);
}

esp_err_t esp_foc_sensored_clear_fault(uint8_t axis)
{
    sd_t *sd = sd_axis(axis);
    if (sd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sd->inited || !sd->latched) {
        return ESP_ERR_INVALID_STATE;
    }
    return sd_request(sd, SD_REQ_CLEAR, SD_REQ_TIMEOUT_MS);
}

static sd_t *sd_ready(uint8_t axis, esp_err_t *err)
{
    sd_t *sd = sd_axis(axis);
    if (sd == NULL) {
        *err = ESP_ERR_INVALID_ARG;
        return NULL;
    }
    if (!sd->inited) {
        *err = ESP_ERR_INVALID_STATE;
        return NULL;
    }
    *err = ESP_OK;
    return sd;
}

static bool sd_mode_live(const sd_t *sd, esp_foc_sensored_mode_t mode)
{
    return (sd->mode == (uint8_t)mode) && (sd->state != ESP_FOC_SD_STATE_LEARNING);
}

esp_err_t esp_foc_sensored_set_mode(uint8_t axis, esp_foc_sensored_mode_t mode)
{
    esp_err_t err;
    sd_t *sd = sd_ready(axis, &err);
    if (sd == NULL) {
        return err;
    }
    if (((int)mode < 0) || (mode > ESP_FOC_SD_MODE_POSITION)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (((int)mode > (int)sd->cfg.control) || (sd->state == ESP_FOC_SD_STATE_LEARNING) ||
        (sd->state == ESP_FOC_SD_STATE_ARMED)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (sd->mode == (uint8_t)mode) {
        return ESP_OK;
    }
    esp_foc_critical_enter();
    sd_enter_mode(sd, mode);
    esp_foc_critical_leave();
    sd->inpos = false;
    sd->inpos_cnt = 0;
    sd_emit_simple(sd, ESP_FOC_SD_EV_MODE);
    return ESP_OK;
}

esp_err_t esp_foc_sensored_set_iq(uint8_t axis, float a)
{
    esp_err_t err;
    sd_t *sd = sd_ready(axis, &err);
    if (sd == NULL) {
        return err;
    }
    if (sd->state == ESP_FOC_SD_STATE_LEARNING) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!(fabsf(a) <= sd->cfg.i_max_a)) {
        return ESP_ERR_INVALID_ARG;
    }
    sd->iq_user = q16_from_float(a);
    return ESP_OK;
}

esp_err_t esp_foc_sensored_set_id(uint8_t axis, float a)
{
    esp_err_t err;
    sd_t *sd = sd_ready(axis, &err);
    if (sd == NULL) {
        return err;
    }
    if (!(fabsf(a) <= sd->cfg.i_max_a)) {
        return ESP_ERR_INVALID_ARG;
    }
    sd->id_user = q16_from_float(a);
    return ESP_OK;
}

esp_err_t esp_foc_sensored_set_vdq_ff(uint8_t axis, float vd_v, float vq_v)
{
    esp_err_t err;
    sd_t *sd = sd_ready(axis, &err);
    if (sd == NULL) {
        return err;
    }
    const float v_max = sd->vdc_v * SD_INV_SQRT3;
    if (!(fabsf(vd_v) <= v_max) || !(fabsf(vq_v) <= v_max)) {
        return ESP_ERR_INVALID_ARG;
    }
    const q16_t vd = q16_from_float(vd_v / sd->vdc_v);
    const q16_t vq = q16_from_float(vq_v / sd->vdc_v);
    esp_foc_critical_enter();
    sd->vd_ff = vd;
    sd->vq_ff = vq;
    esp_foc_critical_leave();
    return ESP_OK;
}

esp_err_t esp_foc_sensored_set_speed_ref_hz(uint8_t axis, float fe_hz)
{
    esp_err_t err;
    sd_t *sd = sd_ready(axis, &err);
    if (sd == NULL) {
        return err;
    }
    if (!sd_mode_live(sd, ESP_FOC_SD_MODE_VELOCITY)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!(fabsf(fe_hz) <= sd->cfg.guard.overspeed_hz)) {
        return ESP_ERR_INVALID_ARG;
    }
    sd->w_target = q16_from_float(SD_TWO_PI * fe_hz);
    return ESP_OK;
}

esp_err_t esp_foc_sensored_set_speed_slew(uint8_t axis, float hz_per_s)
{
    esp_err_t err;
    sd_t *sd = sd_ready(axis, &err);
    if (sd == NULL) {
        return err;
    }
    if ((int)sd->cfg.control < (int)ESP_FOC_SD_CONTROL_VELOCITY) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!(hz_per_s > 0.0f) || (SD_TWO_PI * hz_per_s / sd->slot_hz > 30000.0f)) {
        return ESP_ERR_INVALID_ARG;
    }
    sd->w_step = sd_step_q16(SD_TWO_PI * hz_per_s, sd->slot_hz);
    return ESP_OK;
}

static int64_t sd_pos_q(float rad)
{
    return (int64_t)llround((double)rad * 65536.0);
}

esp_err_t esp_foc_sensored_set_position_ref_rad(uint8_t axis, float theta_rad)
{
    esp_err_t err;
    sd_t *sd = sd_ready(axis, &err);
    if (sd == NULL) {
        return err;
    }
    if (!sd_mode_live(sd, ESP_FOC_SD_MODE_POSITION)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!(fabsf(theta_rad) < 1.0e7f)) {
        return ESP_ERR_INVALID_ARG;
    }
    const int64_t q = sd_pos_q(theta_rad);
    esp_foc_critical_enter();
    sd->ref_abs = q + sd->pos_org;
    sd->w_ff = 0;
    esp_foc_critical_leave();
    return ESP_OK;
}

esp_err_t esp_foc_sensored_set_origin(uint8_t axis)
{
    esp_err_t err;
    sd_t *sd = sd_ready(axis, &err);
    if (sd == NULL) {
        return err;
    }
    if (sd->state == ESP_FOC_SD_STATE_LEARNING) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_foc_critical_enter();
    sd->pos_org = sd->pos_raw;
    esp_foc_critical_leave();
    return ESP_OK;
}

esp_err_t esp_foc_sensored_set_joint(uint8_t axis, float theta_rad, float w_ff_rads, float iq_ff_a)
{
    esp_err_t err;
    sd_t *sd = sd_ready(axis, &err);
    if (sd == NULL) {
        return err;
    }
    if (!sd_mode_live(sd, ESP_FOC_SD_MODE_POSITION)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!(fabsf(theta_rad) < 1.0e7f) || !(fabsf(w_ff_rads) <= SD_TWO_PI * sd->cfg.wm_max_hz) ||
        !(fabsf(iq_ff_a) <= sd->cfg.i_max_a)) {
        return ESP_ERR_INVALID_ARG;
    }
    const int64_t q = sd_pos_q(theta_rad);
    const q16_t w = q16_from_float(w_ff_rads);
    const q16_t iq = q16_from_float(iq_ff_a);
    esp_foc_critical_enter();
    sd->ref_abs = q + sd->pos_org;
    sd->w_ff = w;
    sd->iq_user = iq;
    esp_foc_critical_leave();
    return ESP_OK;
}

static void sd_pid_take(esp_foc_pid_t *dst, const esp_foc_pid_t *src)
{
    dst->b0 = src->b0;
    dst->b1 = src->b1;
    dst->b2 = src->b2;
    dst->a1 = src->a1;
    dst->a2 = src->a2;
    dst->kp = src->kp;
    dst->ki = src->ki;
}

esp_err_t esp_foc_sensored_set_current_pi(uint8_t axis, float kp, float ki)
{
    esp_err_t err;
    sd_t *sd = sd_ready(axis, &err);
    if (sd == NULL) {
        return err;
    }
    if (!(kp > 0.0f) || !(ki >= 0.0f) || (kp >= 32000.0f) || (ki / (float)sd->pwm_hz >= 32000.0f)) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_foc_pid_t tmp = sd->pi_d;
    err = esp_foc_pid_set_kp(&tmp, kp);
    if (err == ESP_OK) {
        err = esp_foc_pid_set_ki(&tmp, ki);
    }
    if (err != ESP_OK) {
        return err;
    }
    esp_foc_critical_enter();
    sd_pid_take(&sd->pi_d, &tmp);
    sd_pid_take(&sd->pi_q, &tmp);
    esp_foc_critical_leave();
    sd->tune.kp_i = kp;
    sd->tune.ki_i = ki;
    return ESP_OK;
}

esp_err_t esp_foc_sensored_set_speed_pi(uint8_t axis, float kp, float ki)
{
    esp_err_t err;
    sd_t *sd = sd_ready(axis, &err);
    if (sd == NULL) {
        return err;
    }
    if ((int)sd->cfg.control < (int)ESP_FOC_SD_CONTROL_VELOCITY) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!(kp >= 0.0f) || !(ki >= 0.0f) || (kp >= 100.0f) || (ki / sd->slot_hz >= 100.0f)) {
        return ESP_ERR_INVALID_ARG;
    }
    sd_speed_gains(sd, kp, ki);
    return ESP_OK;
}

esp_err_t esp_foc_sensored_set_speed_bw(uint8_t axis, float hz)
{
    esp_err_t err;
    sd_t *sd = sd_ready(axis, &err);
    if (sd == NULL) {
        return err;
    }
    if ((int)sd->cfg.control < (int)ESP_FOC_SD_CONTROL_VELOCITY) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!(hz > 0.0f) || (hz >= 0.25f * sd->slot_hz)) {
        return ESP_ERR_INVALID_ARG;
    }
    float kp = 0.0f;
    float ki = 0.0f;
    err = esp_foc_pid_design_integrator(sd->cfg.k_rads2_a, sd->slot_hz, hz, sd->cfg.speed_zeta,
                                        &kp, &ki);
    if (err != ESP_OK) {
        return err;
    }
    sd_speed_gains(sd, kp, ki);
    sd->tune.speed_bw_hz = hz;
    sd->tune.speed_fc_hz = 2.0f * sd->cfg.speed_zeta * hz;
    return ESP_OK;
}

esp_err_t esp_foc_sensored_set_position_kp(uint8_t axis, float kp)
{
    esp_err_t err;
    sd_t *sd = sd_ready(axis, &err);
    if (sd == NULL) {
        return err;
    }
    if (sd->cfg.control != ESP_FOC_SD_CONTROL_POSITION) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!(kp >= 0.0f) || (kp >= 30000.0f)) {
        return ESP_ERR_INVALID_ARG;
    }
    sd_position_kp(sd, kp);
    return ESP_OK;
}

esp_err_t esp_foc_sensored_cogging_learn(uint8_t axis, uint32_t timeout_ms,
                                         esp_foc_sensored_cogging_info_t *info)
{
    esp_err_t err;
    sd_t *sd = sd_ready(axis, &err);
    if (sd == NULL) {
        return err;
    }
    if ((sd->state != ESP_FOC_SD_STATE_RUNNING) || (sd->mode != ESP_FOC_SD_MODE_POSITION) ||
        (esp_foc_event_handle_self() == sd->sup)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (timeout_ms == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    sd->learn_timeout_ms = timeout_ms;
    err = sd_request(sd, SD_REQ_LEARN, timeout_ms + SD_REQ_TIMEOUT_MS);
    if (info != NULL) {
        *info = sd->cog_info;
    }
    return err;
}

esp_err_t esp_foc_sensored_cogging_enable(uint8_t axis, bool on)
{
    esp_err_t err;
    sd_t *sd = sd_ready(axis, &err);
    if (sd == NULL) {
        return err;
    }
    if (on && !sd->cog_valid) {
        return ESP_ERR_INVALID_STATE;
    }
    sd->cog_on = on;
    return ESP_OK;
}

void esp_foc_sensored_get_cogging_info(uint8_t axis, esp_foc_sensored_cogging_info_t *info)
{
    sd_t *sd = sd_axis(axis);
    if ((sd == NULL) || (info == NULL)) {
        return;
    }
    *info = sd->cog_info;
}

esp_foc_sensored_state_t esp_foc_sensored_get_state(uint8_t axis)
{
    sd_t *sd = sd_axis(axis);
    return (sd != NULL) ? sd->state : ESP_FOC_SD_STATE_IDLE;
}

void esp_foc_sensored_get_status(uint8_t axis, esp_foc_sensored_status_t *st)
{
    sd_t *sd = sd_axis(axis);
    if ((sd == NULL) || (st == NULL)) {
        return;
    }
    esp_foc_critical_enter();
    const q16_t th = sd->theta;
    const q16_t we = esp_foc_rotor_pll_get_omega(&sd->pll);
    const q16_t w_ref = sd->w_ref;
    const int64_t pos = sd->pos_raw - sd->pos_org;
    const int64_t ref = sd->ref_abs - sd->pos_org;
    const q16_t w_ff = sd->w_ff;
    const q16_t id = sd->id;
    const q16_t iq = sd->iq;
    const q16_t id_ref = sd->id_user;
    const q16_t iq_ref = sd->iq_ref;
    const q16_t vd = sd->vd;
    const q16_t vq = sd->vq;
    const uint32_t tez = sd->tez;
    esp_foc_critical_leave();

    memset(st, 0, sizeof(*st));
    st->state = sd->state;
    st->mode = (esp_foc_sensored_mode_t)sd->mode;
    st->theta_e_rad = q16_to_float(th);
    st->we_rads = q16_to_float(we);
    st->w_ref_rads = q16_to_float(w_ref);
    st->theta_m_rad = (float)((double)pos / 65536.0);
    st->theta_ref_rad = (float)((double)ref / 65536.0);
    st->w_ff_rads = q16_to_float(w_ff);
    st->id_a = q16_to_float(id);
    st->iq_a = q16_to_float(iq);
    st->id_ref_a = q16_to_float(id_ref);
    st->iq_ref_a = q16_to_float(iq_ref);
    st->vd_v = q16_to_float(vd) * sd->vdc_v;
    st->vq_v = q16_to_float(vq) * sd->vdc_v;
    st->vdc_v = sd->vdc_v;
    st->inpos = sd->inpos;
    st->inpos_toggles = sd->inpos_toggles;
    st->cogging_on = sd->cog_on;
    st->tez = tez;
    st->fetch_n = sd->fetch_n;
    st->fetch_fail = sd->fetch_fail;
    st->fail = sd->fail;
    st->abort = sd->abort_seen;
    st->fault = sd->fault_seen;
}

void esp_foc_sensored_get_window(uint8_t axis, esp_foc_sensored_window_t *w)
{
    sd_t *sd = sd_axis(axis);
    if ((sd == NULL) || (w == NULL)) {
        return;
    }
    esp_foc_critical_enter();
    const uint32_t n = sd->win_n;
    const int64_t sw = sd->win_w;
    const int64_t se = sd->win_e;
    const int64_t se2 = sd->win_e2;
    const q16_t emin = sd->win_emin;
    const q16_t emax = sd->win_emax;
    const uint32_t cross = sd->win_cross;
    const int64_t siqr = sd->win_iqr;
    const q16_t iqr_min = sd->win_iqr_min;
    const q16_t iqr_max = sd->win_iqr_max;
    const int64_t siq = sd->win_iq;
    const q16_t w_ref = sd->w_ref;
    if (n > 0u) {
        sd->win_bias = (q16_t)(se / (int64_t)n);
    }
    sd->win_n = 0u;
    sd->win_w = 0;
    sd->win_e = 0;
    sd->win_e2 = 0;
    sd->win_cross = 0u;
    sd->win_iqr = 0;
    sd->win_iq = 0;
    esp_foc_critical_leave();

    const double hz = 1.0 / (65536.0 * (double)SD_TWO_PI);
    const double dn = (n > 0u) ? (double)n : 1.0;
    const double es = (double)(1u << SD_WIN_SHIFT);
    memset(w, 0, sizeof(*w));
    w->n = n;
    w->t_ms = (uint32_t)(esp_foc_now_us() / 1000u);
    w->w_ref_hz = (float)((double)w_ref * hz);
    if (n == 0u) {
        return;
    }
    w->w_mean_hz = (float)((double)sw / dn * hz);
    w->werr_sum_hz = (float)((double)se * hz);
    w->werr_sq_hz2 = (float)((double)se2 * es * es * hz * hz);
    w->werr_min_hz = (float)((double)emin * hz);
    w->werr_max_hz = (float)((double)emax * hz);
    w->werr_cross = cross;
    w->iq_ref_mean_a = (float)((double)siqr / dn / 65536.0);
    w->iq_ref_min_a = q16_to_float(iqr_min);
    w->iq_ref_max_a = q16_to_float(iqr_max);
    w->iq_mean_a = (float)((double)siq / dn / 65536.0);
}

void esp_foc_sensored_get_tuning(uint8_t axis, esp_foc_sensored_tuning_t *t)
{
    sd_t *sd = sd_axis(axis);
    if ((sd == NULL) || (t == NULL)) {
        return;
    }
    *t = sd->tune;
}
