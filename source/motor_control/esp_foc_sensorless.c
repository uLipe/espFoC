/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Sensorless FoC stack: TEZ fast path, decimated speed/guard slot, and the
 * supervisor that walks the startup into a caught speed or torque loop.
 */
#include "espFoC/motor_control/esp_foc_sensorless.h"

#include <math.h>
#include <string.h>

#include "sdkconfig.h"
#include "espFoC/motor_control/esp_foc_if.h"
#include "espFoC/motor_control/esp_foc_observer_flux.h"
#include "espFoC/osal/esp_foc_osal.h"
#include "espFoC/utils/esp_foc_angle.h"
#include "espFoC/utils/esp_foc_clarke.h"
#include "espFoC/utils/esp_foc_park.h"
#include "espFoC/utils/esp_foc_pid.h"
#include "espFoC/utils/esp_foc_svm.h"
#include "espFoC/utils/esp_foc_trig.h"
#include "espFoC/utils/esp_foc_vlim.h"

#define SL_TWO_PI          6.28318530718f
#define SL_INV_SQRT3       0.57735026919f
#define SL_SLOW_DIV        ((uint32_t)CONFIG_ESP_FOC_SL_SLOW_DIV)
#define SL_POLL_MS         20u
#define SL_REQ_TIMEOUT_MS  5000u
/* Collapse is only judged with a real torque request on q. */
#define SL_COLLAPSE_IQREF_A 0.10f
/* Hunting this long under we_min during the settle refuses the commit. */
#define SL_SETTLE_BELOW_MS 60u

#define SL_REQ_RUN   (1u << 0)
#define SL_REQ_STOP  (1u << 1)
#define SL_REQ_CLEAR (1u << 2)
#define SL_REQ_EXIT  (1u << 3)

#define SL_M_REF   (1u << 0)
#define SL_M_SIGN  (1u << 1)
#define SL_M_GUARD (1u << 2)
#define SL_M_ALL   (SL_M_REF | SL_M_SIGN | SL_M_GUARD)

typedef enum {
    SL_OK = 0,
    SL_CUT,
    SL_SIGN,
    SL_STOP,
    SL_EXIT,
    SL_FAILED,
    SL_ABORT,
    SL_FAULT,
} sl_out_t;

enum {
    SL_SRC_OL = 0,
    SL_SRC_OBS = 1,
};

typedef struct {
    /* --- TEZ fast path --- */
    volatile bool run;
    volatile bool obs_run;
    volatile bool vf_on;
    volatile bool blend_on;
    volatile uint8_t theta_src;
    volatile q16_t theta_ol;
    volatile q16_t dth;
    volatile q16_t blend_u;
    q16_t blend_du;
    volatile q16_t id_ref;
    volatile q16_t iq_ref;
    volatile q16_t vd_ff;
    volatile q16_t vq_ff;
    volatile q16_t vf_vd;
    volatile q16_t vf_vq;
    q16_t vdc;
    volatile q16_t iu;
    volatile q16_t iv;
    volatile q16_t iw;
    q16_t valpha;
    q16_t vbeta;
    volatile q16_t theta;
    volatile q16_t th_obs;
    volatile q16_t w_obs;
    volatile bool lock_raw;
    volatile q16_t id;
    volatile q16_t iq;
    volatile q16_t vd;
    volatile q16_t vq;
    volatile uint32_t tez;
    uint32_t slow_div;
    esp_foc_pid_t pi_d;
    esp_foc_pid_t pi_q;
    esp_foc_observer_flux_t obs;

    /* --- decimated slot --- */
    volatile bool slew_on;
    volatile bool speed_on;
    volatile bool guards_on;
    volatile q16_t w_cmd;
    volatile q16_t w_ref;
    volatile q16_t w_target;
    volatile q16_t w_step;
    volatile q16_t w_ctrl;
    q16_t w_filt_b0;
    volatile q16_t iq_lo;
    volatile q16_t iq_hi;
    volatile q16_t iq_user;
    volatile q16_t iq_target;
    volatile q16_t iq_step;
    volatile q16_t id_target;
    volatile q16_t id_step;
    esp_foc_pid_t pi_w;
    q16_t g_bemf_k;
    q16_t g_w_fmin;
    q16_t g_w_trip;
    q16_t g_i_col;
    q16_t g_iq_col_ref;
    uint32_t g_bemf_need;
    uint32_t g_ov_need;
    uint32_t g_col_need;
    uint32_t g_loss_need;
    uint32_t g_bemf_n;
    uint32_t g_ov_n;
    uint32_t g_col_n;
    uint32_t g_loss_n;
    bool g_loss_abort;
    volatile uint8_t abort;
    volatile uint8_t fault;
    esp_foc_event_handle_t sup;

    /* --- supervisor --- */
    uint8_t axis;
    esp_foc_inverter_t *inv;
    esp_foc_sensorless_config_t cfg;
    esp_foc_if_t ifs;
    volatile esp_foc_sensorless_state_t state;
    volatile uint32_t req;
    volatile esp_err_t req_err;
    volatile q16_t w_user;
    volatile q16_t w_slew_user;
    volatile q16_t id_user;
    volatile bool track;
    volatile int8_t dir;
    volatile bool inited;
    volatile bool task_alive;
    bool bridge_on;
    bool latched;
    sl_out_t if_out;
    esp_foc_sensorless_fail_t fail;
    esp_foc_sensorless_abort_t abort_seen;
    esp_foc_fault_reason_t fault_seen;
    uint64_t coast_until_us;
    uint32_t follow_ms;
    uint32_t follow_wait_ms;
    uint32_t id_err_ms;
    bool no_follow;
    volatile q16_t fe_ol;

    uint32_t pwm_hz;
    float slot_hz;
    float vdc_v;
    float f_base_hz;
    int plateau_hz;
    float we_min_hz;
    float kp_lim;
    esp_foc_sensorless_tuning_t tune;
    q16_t inv_vdc;
    q16_t rs_pu;
    q16_t psi;
    q16_t follow_k2;
    q16_t ramp_id_err;
    q16_t acq_band;
    q16_t acq_hunt;
    q16_t iq_start;
    q16_t w_plateau;
    q16_t we_min;
    q16_t w_hold_max;
    q16_t catch_floor;
    q16_t brake;
    q16_t i_max;
    q16_t close_frac;
    q16_t close_min;
    q16_t w_cut;
    q16_t iq_cut;
    q16_t rev_w_step;
    q16_t rev_iq_step;
    q16_t catch_step;
} sl_t;

static sl_t s_sl[CONFIG_ESP_FOC_SL_MAX_AXES];

#if CONFIG_ESP_FOC_SL_MAX_AXES > 4
#error "k_task_name covers four axes"
#endif
static const char *const k_task_name[] = {"foc_sl0", "foc_sl1", "foc_sl2", "foc_sl3"};

static inline sl_t *sl_axis(uint8_t axis)
{
    return (axis < CONFIG_ESP_FOC_SL_MAX_AXES) ? &s_sl[axis] : NULL;
}

static inline q16_t sl_abs(q16_t x)
{
    return (x < 0) ? q16_neg(x) : x;
}

static inline q16_t sl_slew(q16_t x, q16_t target, q16_t step)
{
    if (step == 0) {
        return target;
    }
    const q16_t d = q16_sub(target, x);
    if (d > step) {
        return q16_add(x, step);
    }
    if (d < q16_neg(step)) {
        return q16_sub(x, step);
    }
    return target;
}

/* ------------------------------------------------------------------------ */
/* Fast path                                                                 */
/* ------------------------------------------------------------------------ */

static void sl_dma(void *arg)
{
    sl_t *sl = (sl_t *)arg;
    esp_foc_inverter_t *inv = sl->inv;
    q16_t u;
    q16_t v;
    q16_t w;
    inv->fetch_currents(inv, &u, &v, &w);
    sl->iu = u;
    sl->iv = v;
    sl->iw = w;
}

static void sl_fault(void *arg, esp_foc_fault_reason_t reason)
{
    sl_t *sl = (sl_t *)arg;
    sl->run = false;
    sl->fault = (uint8_t)reason;
    esp_foc_event_post_auto(sl->sup);
}

static uint8_t sl_guards(sl_t *sl, q16_t w_obs, q16_t id, q16_t iq, bool lock)
{
    uint8_t why = ESP_FOC_SL_ABORT_NONE;

    if (sl->obs_run) {
        const q16_t w_pl = sl_abs((sl->theta_src == SL_SRC_OBS) ? w_obs : sl->w_cmd);
        if (w_pl >= sl->g_w_fmin) {
            const esp_foc_observer_t *o = &sl->obs.iface;
            const q16_t ea = o->get_e_alpha(o);
            const q16_t eb = o->get_e_beta(o);
            const q16_t e_min = q16_mul(sl->g_bemf_k, w_pl);
            if (q16_add(q16_mul(ea, ea), q16_mul(eb, eb)) < q16_mul(e_min, e_min)) {
                if (++sl->g_bemf_n >= sl->g_bemf_need) {
                    why = ESP_FOC_SL_ABORT_BEMF;
                }
            } else {
                sl->g_bemf_n = 0;
            }
        } else {
            sl->g_bemf_n = 0;
        }
    }
    if (!sl->guards_on) {
        return why;
    }

    if (lock) {
        sl->g_loss_n = 0;
    } else if (++sl->g_loss_n >= sl->g_loss_need && sl->g_loss_abort) {
        why = ESP_FOC_SL_ABORT_LOCK_LOSS;
    }

    if ((sl_abs(sl->iq_ref) > sl->g_iq_col_ref) && (sl_abs(id) < sl->g_i_col) &&
        (sl_abs(iq) < sl->g_i_col)) {
        if (++sl->g_col_n >= sl->g_col_need) {
            why = ESP_FOC_SL_ABORT_COLLAPSE;
        }
    } else {
        sl->g_col_n = 0;
    }

    if (sl_abs(w_obs) > sl->g_w_trip) {
        if (++sl->g_ov_n >= sl->g_ov_need) {
            why = ESP_FOC_SL_ABORT_OVERSPEED;
        }
    } else {
        sl->g_ov_n = 0;
    }
    return why;
}

static void sl_slot(sl_t *sl, q16_t w_obs)
{
    if (sl->obs_run) {
        const q16_t w_ctrl = sl->w_ctrl;
        sl->w_ctrl = q16_add(w_ctrl, q16_mul(sl->w_filt_b0, q16_sub(w_obs, w_ctrl)));
    }
    if (!sl->slew_on) {
        return;
    }
    sl->id_ref = sl_slew(sl->id_ref, sl->id_target, sl->id_step);
    if (sl->speed_on) {
        const q16_t w_ref = sl_slew(sl->w_ref, sl->w_target, sl->w_step);
        const q16_t iq_user = sl->iq_user;
        const q16_t lo = sl->iq_lo;
        const q16_t hi = sl->iq_hi;
        q16_t u = q16_add(esp_foc_pid_update(&sl->pi_w, w_ref, sl->w_ctrl), iq_user);
        u = q16_clamp(u, lo, hi);
        esp_foc_pid_set_applied(&sl->pi_w, q16_sub(u, iq_user));
        sl->w_ref = w_ref;
        sl->iq_ref = u;
    } else {
        sl->iq_ref = sl_slew(sl->iq_ref, sl->iq_target, sl->iq_step);
    }
}

static void sl_tez(void *arg)
{
    sl_t *sl = (sl_t *)arg;
    esp_foc_inverter_t *inv = sl->inv;
    sl->tez++;
    if (!sl->run) {
        return;
    }

    const q16_t th_ol = q16_wrap_pi(q16_add(sl->theta_ol, sl->dth));
    const q16_t iu = sl->iu;
    const q16_t iv = sl->iv;
    const q16_t iw = sl->iw;
    const q16_t vdc = sl->vdc;
    const q16_t va = q16_mul(sl->valpha, vdc);
    const q16_t vb = q16_mul(sl->vbeta, vdc);
    const q16_t idr = sl->id_ref;
    const q16_t iqr = sl->iq_ref;
    /* The user's feedforward belongs to the caught loop, not to the align. */
    const bool caught = sl->slew_on;
    const q16_t vd_ff = caught ? sl->vd_ff : 0;
    const q16_t vq_ff = caught ? sl->vq_ff : 0;
    q16_t th_obs = sl->th_obs;
    q16_t w_obs = sl->w_obs;
    bool lock = sl->lock_raw;

    q16_t ia;
    q16_t ib;
    esp_foc_clarke(iu, iv, iw, &ia, &ib);

    if (sl->obs_run) {
        esp_foc_observer_t *o = &sl->obs.iface;
        o->update(o, ia, ib, va, vb);
        th_obs = o->get_theta(o);
        w_obs = o->get_omega(o);
        lock = o->is_locked(o);
    }

    q16_t th;
    if (sl->blend_on) {
        q16_t u = q16_add(sl->blend_u, sl->blend_du);
        if (u >= Q16_ONE) {
            u = Q16_ONE;
            sl->blend_on = false;
            sl->theta_src = SL_SRC_OBS;
            sl->dth = 0;
        }
        sl->blend_u = u;
        th = q16_wrap_pi(q16_add(th_ol, q16_mul(u, q16_angle_delta(th_ol, th_obs))));
    } else {
        th = (sl->theta_src == SL_SRC_OBS) ? th_obs : th_ol;
    }

    q16_t s;
    q16_t c;
    esp_foc_sincos(th, &s, &c);
    q16_t id;
    q16_t iq;
    esp_foc_park(s, c, ia, ib, &id, &iq);

    q16_t vd;
    q16_t vq;
    if (sl->vf_on) {
        /* Open loop at standstill: the PIs would only wind up against the
         * dead zone, so they sit out until to_current_mode seeds them. */
        vd = sl->vf_vd;
        vq = sl->vf_vq;
        esp_foc_vlim_dq(&vd, &vq, Q16_INV_SQRT3);
    } else {
        vd = q16_add(esp_foc_pid_update(&sl->pi_d, idr, id), vd_ff);
        vq = q16_add(esp_foc_pid_update(&sl->pi_q, iqr, iq), vq_ff);
        esp_foc_vlim_dq(&vd, &vq, Q16_INV_SQRT3);
        esp_foc_pid_set_applied(&sl->pi_d, q16_sub(vd, vd_ff));
        esp_foc_pid_set_applied(&sl->pi_q, q16_sub(vq, vq_ff));
    }

    q16_t a;
    q16_t b;
    q16_t du;
    q16_t dv;
    q16_t dw;
    esp_foc_inv_park(s, c, vd, vq, &a, &b);
    esp_foc_svm(a, b, &du, &dv, &dw);
    inv->set_duties(inv, du, dv, dw);

    sl->theta_ol = th_ol;
    sl->theta = th;
    sl->th_obs = th_obs;
    sl->w_obs = w_obs;
    sl->lock_raw = lock;
    sl->id = id;
    sl->iq = iq;
    sl->vd = vd;
    sl->vq = vq;
    sl->valpha = a;
    sl->vbeta = b;

    if (++sl->slow_div < SL_SLOW_DIV) {
        return;
    }
    sl->slow_div = 0;
    sl_slot(sl, w_obs);
    const uint8_t why = sl_guards(sl, w_obs, id, iq, lock);
    if (why != ESP_FOC_SL_ABORT_NONE) {
        sl->abort = why;
        sl->run = false;
        inv->set_duties(inv, Q16_HALF, Q16_HALF, Q16_HALF);
        /* Only a trip wakes the supervisor. A post per slot is a context
         * switch per slot on a core the fast path already nearly fills, and
         * it starved IDLE into the task watchdog. */
        esp_foc_event_post_auto(sl->sup);
    }
}

/* ------------------------------------------------------------------------ */
/* Supervisor helpers                                                        */
/* ------------------------------------------------------------------------ */

static void sl_emit(sl_t *sl, esp_foc_sensorless_event_t *e)
{
    if (sl->cfg.on_event != NULL) {
        sl->cfg.on_event(sl->cfg.ctx, e);
    }
}

static esp_foc_sensorless_event_t sl_event(sl_t *sl, esp_foc_sensorless_ev_t ev)
{
    esp_foc_sensorless_event_t e;
    memset(&e, 0, sizeof(e));
    e.ev = ev;
    e.axis = sl->axis;
    e.state = sl->state;
    e.dir = sl->dir;
    e.speed_mode = sl->cfg.speed_loop;
    return e;
}

static void sl_emit_simple(sl_t *sl, esp_foc_sensorless_ev_t ev)
{
    esp_foc_sensorless_event_t e = sl_event(sl, ev);
    sl_emit(sl, &e);
}

static void sl_emit_angle(sl_t *sl, esp_foc_sensorless_ev_t ev)
{
    esp_foc_sensorless_event_t e = sl_event(sl, ev);
    e.we_rads = q16_to_float(sl->w_obs);
    e.dang_rad = q16_to_float(q16_angle_delta(sl->theta_ol, sl->th_obs));
    sl_emit(sl, &e);
}

static int8_t sl_ref_dir(sl_t *sl)
{
    const q16_t ref = sl->cfg.speed_loop ? sl->w_user : sl->iq_user;
    const q16_t cut = sl->cfg.speed_loop ? sl->w_cut : sl->iq_cut;
    if (sl_abs(ref) < cut) {
        return 0;
    }
    return (ref < 0) ? -1 : 1;
}

static void sl_track_user(sl_t *sl)
{
    sl->id_target = sl->id_user;
    if (sl->cfg.speed_loop) {
        sl->w_target = sl->w_user;
        sl->w_step = sl->w_slew_user;
    } else {
        sl->iq_target = sl->iq_user;
    }
}

static sl_out_t sl_pending(sl_t *sl, uint32_t mask)
{
    if ((mask & SL_M_GUARD) != 0u) {
        if (sl->fault != (uint8_t)ESP_FOC_FAULT_NONE) {
            return SL_FAULT;
        }
        if (sl->abort != (uint8_t)ESP_FOC_SL_ABORT_NONE) {
            return SL_ABORT;
        }
    }
    const uint32_t req = sl->req;
    if ((req & SL_REQ_EXIT) != 0u) {
        return SL_EXIT;
    }
    if ((req & SL_REQ_STOP) != 0u) {
        return SL_STOP;
    }
    if ((mask & (SL_M_REF | SL_M_SIGN)) != 0u) {
        const int8_t d = sl_ref_dir(sl);
        if (((mask & SL_M_REF) != 0u) && (d == 0)) {
            return SL_CUT;
        }
        if (((mask & SL_M_SIGN) != 0u) && (d != 0) && (d != sl->dir)) {
            return SL_SIGN;
        }
    }
    return SL_OK;
}

/* Time comes from the clock, so a bridge that stopped ticking cannot stretch
 * a wait. Wake-ups come from the deadline, an API post, a fault or a trip. */
static sl_out_t sl_wait(sl_t *sl, uint32_t ms, uint32_t mask)
{
    const uint64_t end = esp_foc_now_us() + (uint64_t)ms * 1000u;
    for (;;) {
        if (sl->track) {
            sl_track_user(sl);
        }
        const sl_out_t o = sl_pending(sl, mask);
        if (o != SL_OK) {
            return o;
        }
        const uint64_t now = esp_foc_now_us();
        if (now >= end) {
            return SL_OK;
        }
        (void)esp_foc_event_wait_ms((uint32_t)((end - now + 999u) / 1000u));
    }
}

static void sl_set_fe(sl_t *sl, q16_t fe_hz)
{
    sl->fe_ol = fe_hz;
    sl->w_cmd = q16_mul(fe_hz, Q16_TWO_PI);
    sl->dth = (q16_t)(((int64_t)fe_hz * (int64_t)Q16_TWO_PI) /
                       ((int64_t)sl->pwm_hz << 16));
}

static void sl_iq_band(sl_t *sl, int8_t dir, q16_t floor_q)
{
    if (dir > 0) {
        sl->iq_lo = floor_q;
        sl->iq_hi = sl->i_max;
    } else {
        sl->iq_lo = q16_neg(sl->i_max);
        sl->iq_hi = q16_neg(floor_q);
    }
}

static void sl_reset_guards(sl_t *sl)
{
    sl->g_bemf_n = 0;
    sl->g_ov_n = 0;
    sl->g_col_n = 0;
    sl->g_loss_n = 0;
}

/* Mid duty, bridge off, everything the ISRs read back to rest. Disables only
 * a bridge this stack enabled: disable() on an idle inverter is not free. */
static void sl_cut(sl_t *sl)
{
    esp_foc_inverter_t *inv = sl->inv;

    esp_foc_critical_enter();
    sl->run = false;
    sl->track = false;
    sl->speed_on = false;
    sl->slew_on = false;
    sl->guards_on = false;
    sl->obs_run = false;
    sl->vf_on = false;
    sl->blend_on = false;
    sl->theta_src = SL_SRC_OL;
    sl->dth = 0;
    sl->w_cmd = 0;
    sl->fe_ol = 0;
    sl->id_ref = 0;
    sl->iq_ref = 0;
    esp_foc_critical_leave();

    if (sl->bridge_on) {
        inv->set_sense_watchdog(inv, false);
        inv->set_duties(inv, Q16_HALF, Q16_HALF, Q16_HALF);
        inv->disable(inv);
        sl->bridge_on = false;
        sl->coast_until_us = esp_foc_now_us() + (uint64_t)sl->cfg.coast_ms * 1000u;
    }
}

/* ------------------------------------------------------------------------ */
/* esp_foc_if ops                                                            */
/* ------------------------------------------------------------------------ */

static void sl_if_set_idq(void *ctx, q16_t id, q16_t iq)
{
    sl_t *sl = (sl_t *)ctx;
    if (sl->if_out != SL_OK) {
        return;
    }
    sl->id_ref = id;
    sl->iq_ref = iq;
}

static void sl_if_set_fe(void *ctx, q16_t fe_hz)
{
    sl_t *sl = (sl_t *)ctx;
    if (sl->if_out != SL_OK) {
        return;
    }
    sl_set_fe(sl, fe_hz);
}

static void sl_if_sleep(void *ctx, uint32_t ms)
{
    sl_t *sl = (sl_t *)ctx;
    if (sl->if_out != SL_OK) {
        return;
    }
    sl->if_out = sl_wait(sl, ms, SL_M_ALL);
}

static void sl_if_poll(void *ctx)
{
    sl_t *sl = (sl_t *)ctx;
    const esp_foc_if_phase_t ph = sl->ifs.phase;
    if (ph <= ESP_FOC_IF_PHASE_ALIGN_SETTLE) {
        sl->state = ESP_FOC_SL_STATE_ALIGN;
    } else if (ph <= ESP_FOC_IF_PHASE_CREEP) {
        sl->state = ESP_FOC_SL_STATE_LOCKIN;
    }
}

static q16_t sl_if_dang(void *ctx)
{
    sl_t *sl = (sl_t *)ctx;
    return sl->obs_run ? q16_angle_delta(sl->theta_ol, sl->th_obs) : 0;
}

static void sl_if_on_plateau(void *ctx)
{
    sl_t *sl = (sl_t *)ctx;
    esp_foc_observer_t *o = &sl->obs.iface;
    esp_foc_critical_enter();
    const q16_t th_ol = sl->theta_ol;
    const q16_t w = sl->w_cmd;
    esp_foc_observer_reset(o);
    esp_foc_observer_set_theta(o, th_ol);
    esp_foc_observer_set_omega(o, w);
    esp_foc_observer_set_extract(o, ESP_FOC_ANGLE_PLL);
    esp_foc_observer_set_pll_enable(o, true);
    sl->th_obs = th_ol;
    sl->w_obs = w;
    sl->w_ctrl = w;
    sl->lock_raw = false;
    sl->g_bemf_n = 0;
    sl->obs_run = true;
    esp_foc_critical_leave();
}

/*
 * Lock-in proof. Every other witness here is derived from the estimator, so a
 * frame that is wrong but self-consistent passes them all; |v - Rs i| is the
 * voltage the current loop needed, and only a turning magnet forces it.
 */
static bool sl_if_lock_ok(void *ctx)
{
    sl_t *sl = (sl_t *)ctx;
    if ((sl->if_out != SL_OK) || sl->no_follow) {
        return true;
    }
    if (!sl->obs_run) {
        return false;
    }
    if (sl->follow_k2 == 0) {
        return true;
    }

    esp_foc_critical_enter();
    const q16_t vd = sl->vd;
    const q16_t vq = sl->vq;
    const q16_t id = sl->id;
    const q16_t iq = sl->iq;
    esp_foc_critical_leave();

    const uint32_t step = sl->cfg.startup.step_ms;
    const q16_t w_abs = sl_abs(sl->w_cmd);
    if (w_abs < sl->g_w_fmin) {
        sl->follow_ms = 0;
        return false;
    }
    const q16_t e_exp = q16_mul(q16_mul(sl->psi, w_abs), sl->inv_vdc);
    const q16_t ed = q16_sub(vd, q16_mul(sl->rs_pu, id));
    const q16_t eq = q16_sub(vq, q16_mul(sl->rs_pu, iq));
    const q16_t got2 = q16_add(q16_mul(ed, ed), q16_mul(eq, eq));
    const q16_t need2 = q16_mul(sl->follow_k2, q16_mul(e_exp, e_exp));

    if (got2 < need2) {
        sl->follow_ms = 0;
        sl->follow_wait_ms += step;
        if (sl->follow_wait_ms >= sl->cfg.startup.follow_timeout_ms) {
            sl->no_follow = true;
            return true;
        }
        return false;
    }
    sl->follow_ms += step;
    return sl->follow_ms >= sl->cfg.startup.follow_hold_ms;
}

static bool sl_if_align_ok(void *ctx)
{
    sl_t *sl = (sl_t *)ctx;
    return sl->if_out == SL_OK;
}

/* A loop that cannot put the lock-in current into the winding is sweeping
 * the frame over a machine it is not driving. */
static bool sl_if_ramp_ok(void *ctx)
{
    sl_t *sl = (sl_t *)ctx;
    if (sl->if_out != SL_OK) {
        return false;
    }
    if (sl_abs(q16_sub(sl->id_ref, sl->id)) <= sl->ramp_id_err) {
        sl->id_err_ms = 0;
        return true;
    }
    sl->id_err_ms += sl->cfg.startup.step_ms;
    if (sl->id_err_ms < sl->cfg.startup.ramp_id_err_ms) {
        return true;
    }
    sl->fail = ESP_FOC_SL_FAIL_NO_CURRENT;
    return false;
}

static void sl_if_set_vdq(void *ctx, q16_t vd, q16_t vq)
{
    sl_t *sl = (sl_t *)ctx;
    esp_foc_critical_enter();
    sl->vf_vd = vd;
    sl->vf_vq = vq;
    sl->vf_on = true;
    esp_foc_critical_leave();
}

/* Bumpless: the PIs never ran during V/f, so they start from the voltage the
 * bridge is already at instead of a zeroed integrator. */
static void sl_if_to_current_mode(void *ctx)
{
    sl_t *sl = (sl_t *)ctx;
    esp_foc_critical_enter();
    esp_foc_pid_reset(&sl->pi_d);
    esp_foc_pid_reset(&sl->pi_q);
    esp_foc_pid_set_applied(&sl->pi_d, sl->vf_vd);
    esp_foc_pid_set_applied(&sl->pi_q, sl->vf_vq);
    sl->vf_on = false;
    esp_foc_critical_leave();
}

/* Band on |we - w_if| and on the step-to-step hunt, held. */
static bool sl_acquire(sl_t *sl)
{
    const q16_t w_if = sl->w_cmd;
    const uint32_t hold_need = sl->cfg.handoff.hold_ms;
    const uint32_t timeout = sl->cfg.handoff.timeout_ms;
    uint32_t hold = 0;
    uint32_t elapsed = 0;
    bool have = false;
    q16_t prev = 0;

    for (;;) {
        const q16_t we = sl->w_obs;
        const q16_t err = sl_abs(q16_sub(we, w_if));
        const q16_t dw = have ? sl_abs(q16_sub(we, prev)) : 0;
        prev = we;
        have = true;
        hold = ((err <= sl->acq_band) && (dw <= sl->acq_hunt)) ? hold + SL_POLL_MS : 0u;
        if (hold >= hold_need) {
            return true;
        }
        if (elapsed >= timeout) {
            sl->fail = ESP_FOC_SL_FAIL_PLL_TIMEOUT;
            return false;
        }
        sl->if_out = sl_wait(sl, SL_POLL_MS, SL_M_ALL);
        if (sl->if_out != SL_OK) {
            return false;
        }
        elapsed += SL_POLL_MS;
    }
}

static bool sl_we_ok(sl_t *sl, q16_t we)
{
    return (sl->dir > 0) ? (we >= sl->we_min) : (we <= q16_neg(sl->we_min));
}

static bool sl_if_handoff(void *ctx)
{
    sl_t *sl = (sl_t *)ctx;
    esp_foc_observer_t *o = &sl->obs.iface;

    if (sl->if_out != SL_OK) {
        return false;
    }
    if (sl->no_follow) {
        sl->fail = ESP_FOC_SL_FAIL_NO_FOLLOW;
        return false;
    }

    sl->state = ESP_FOC_SL_STATE_PLL_ACQUIRE;
    if (!sl_acquire(sl)) {
        return false;
    }
    sl_emit_angle(sl, ESP_FOC_SL_EV_LOCKED);
    sl->state = ESP_FOC_SL_STATE_HANDOFF;

    /*
     * One critical section. Sampling theta_obs and writing it into the live
     * Park frame across a preemptible gap steps the frame by we * (sample
     * age), and the current loop answers a frame step with rail voltage. The
     * VCO is frozen at w_if so the blend does not inherit its ring.
     */
    const q16_t w_if = sl->w_cmd;
    const q16_t floor_q = (sl->dir > 0) ? sl->iq_start : q16_neg(sl->iq_start);
    q16_t id1;
    q16_t iq1;
    esp_foc_critical_enter();
    esp_foc_observer_set_omega(o, w_if);
    esp_foc_observer_set_pll_enable(o, false);
    sl->w_obs = w_if;
    const q16_t th_obs = sl->th_obs;
    const q16_t dang = q16_angle_delta(sl->theta_ol, th_obs);
    const q16_t id0 = sl->id_ref;
    const q16_t iq0 = sl->iq_ref;
    q16_t sn;
    q16_t cs;
    esp_foc_sincos(dang, &sn, &cs);
    id1 = q16_add(q16_mul(id0, cs), q16_mul(iq0, sn));
    iq1 = q16_sub(q16_mul(iq0, cs), q16_mul(id0, sn));
    if ((sl->dir > 0) ? (iq1 < floor_q) : (iq1 > floor_q)) {
        iq1 = floor_q;
    }
    sl->theta_ol = th_obs;
    sl->id_ref = id1;
    sl->iq_ref = iq1;
    sl->blend_u = 0;
    sl->blend_on = true;
    esp_foc_critical_leave();

    sl->if_out = sl_wait(sl, sl->cfg.handoff.blend_ms + 5u, SL_M_ALL);
    if (sl->if_out != SL_OK) {
        return false;
    }
    if (!sl_we_ok(sl, sl->w_obs)) {
        sl->fail = ESP_FOC_SL_FAIL_WE_LOW;
        return false;
    }

    esp_foc_critical_enter();
    sl->blend_on = false;
    sl->theta_src = SL_SRC_OBS;
    sl->dth = 0;
    sl->id_ref = id1;
    sl->iq_ref = iq1;
    esp_foc_observer_set_pll_enable(o, true);
    esp_foc_critical_leave();

    uint32_t below = 0;
    for (uint32_t t = 0; t < sl->cfg.handoff.settle_ms; t += SL_POLL_MS) {
        below = sl_we_ok(sl, sl->w_obs) ? 0u : below + SL_POLL_MS;
        if (below >= SL_SETTLE_BELOW_MS) {
            sl->fail = ESP_FOC_SL_FAIL_WE_DROP;
            return false;
        }
        sl->if_out = sl_wait(sl, SL_POLL_MS, SL_M_ALL);
        if (sl->if_out != SL_OK) {
            return false;
        }
    }

    esp_foc_critical_enter();
    sl_reset_guards(sl);
    sl->guards_on = true;
    esp_foc_critical_leave();
    /* Park is on theta_obs from here, so a frozen sense frame would let the
     * regulators wind into the ceiling against a still picture. */
    sl->inv->set_sense_watchdog(sl->inv, true);
    sl_emit_angle(sl, ESP_FOC_SL_EV_HANDOFF);
    return true;
}

/* ------------------------------------------------------------------------ */
/* State machine                                                             */
/* ------------------------------------------------------------------------ */

static sl_out_t sl_launch(sl_t *sl, int8_t dir)
{
    esp_foc_inverter_t *inv = sl->inv;

    sl->dir = dir;
    sl->fail = ESP_FOC_SL_FAIL_NONE;
    sl->if_out = SL_OK;
    sl->follow_ms = 0;
    sl->follow_wait_ms = 0;
    sl->id_err_ms = 0;
    sl->no_follow = false;
    sl->state = ESP_FOC_SL_STATE_ALIGN;
    sl_emit_simple(sl, ESP_FOC_SL_EV_STARTUP);

    inv->set_duties(inv, Q16_HALF, Q16_HALF, Q16_HALF);
    if (inv->enable(inv) != ESP_OK) {
        sl->fail = ESP_FOC_SL_FAIL_ENABLE;
        return SL_FAILED;
    }
    sl->bridge_on = true;
    /* The DMA hook clears sample_ready, which calibrate waits on. Calibrate
     * also arms the inverter's current limit, so it runs even at 0 rounds. */
    inv->set_dma_callback(inv, NULL, NULL);
    inv->calibrate_currents(inv, sl->cfg.startup.cal_rounds);
    inv->set_dma_callback(inv, sl_dma, sl);
    inv->set_duties(inv, Q16_HALF, Q16_HALF, Q16_HALF);
    sl_out_t out = sl_wait(sl, sl->cfg.startup.arm_ms, SL_M_ALL);
    if (out != SL_OK) {
        return out;
    }

    esp_foc_critical_enter();
    esp_foc_pid_reset(&sl->pi_d);
    esp_foc_pid_reset(&sl->pi_q);
    esp_foc_pid_reset(&sl->pi_w);
    esp_foc_observer_reset(&sl->obs.iface);
    sl->theta_ol = 0;
    sl->th_obs = 0;
    sl->w_obs = 0;
    sl->w_ctrl = 0;
    sl->w_ref = 0;
    sl->lock_raw = false;
    sl->valpha = 0;
    sl->vbeta = 0;
    sl->slow_div = 0;
    sl_reset_guards(sl);
    sl->run = true;
    esp_foc_critical_leave();

    sl->ifs.phase = ESP_FOC_IF_PHASE_ALIGN;
    sl->ifs.fe_hz = 0;
    sl->ifs.id = 0;
    sl->ifs.iq = 0;
    const esp_err_t err = esp_foc_if_run(&sl->ifs, dir);
    if (sl->if_out != SL_OK) {
        return sl->if_out;
    }
    if (err != ESP_OK) {
        if (sl->fail == ESP_FOC_SL_FAIL_NONE) {
            sl->fail = ESP_FOC_SL_FAIL_SEQUENCE;
        }
        return SL_FAILED;
    }
    return SL_OK;
}

static sl_out_t sl_wait_until(sl_t *sl, volatile q16_t *x, volatile q16_t *target)
{
    while (*x != *target) {
        const sl_out_t out = sl_wait(sl, SL_POLL_MS, SL_M_ALL);
        if (out != SL_OK) {
            return out;
        }
    }
    return SL_OK;
}

static sl_out_t sl_catch_speed(sl_t *sl)
{
    const int8_t dir = sl->dir;
    q16_t w0 = q16_clamp(sl_abs(sl->w_obs), sl->w_plateau, sl->w_hold_max);
    if (dir < 0) {
        w0 = q16_neg(w0);
    }

    /* Bumpless: the speed PI starts from the iq the reseed left on q. */
    esp_foc_critical_enter();
    sl->w_ctrl = sl->w_obs;
    sl->w_ref = w0;
    sl->w_target = w0;
    sl->w_step = sl->w_slew_user;
    esp_foc_pid_reset(&sl->pi_w);
    esp_foc_pid_set_applied(&sl->pi_w, q16_sub(sl->iq_ref, sl->iq_user));
    sl_iq_band(sl, dir, sl->catch_floor);
    sl->id_target = sl->id_user;
    sl->id_step = sl->catch_step;
    sl->speed_on = true;
    sl->slew_on = true;
    esp_foc_critical_leave();

    sl_out_t out = sl_wait(sl, sl->cfg.catch_up.hold_ms, SL_M_ALL);
    if (out != SL_OK) {
        return out;
    }
    sl->track = true;
    for (;;) {
        out = sl_wait(sl, SL_POLL_MS, SL_M_ALL);
        if (out != SL_OK) {
            return out;
        }
        if ((sl->w_ref == sl->w_target) && (sl->w_target == sl->w_user)) {
            break;
        }
    }

    /* Caught: let the loop brake, or a step down in w_ref has no torque to
     * shed speed with. */
    esp_foc_critical_enter();
    sl_iq_band(sl, dir, q16_neg(sl->brake));
    esp_foc_critical_leave();

    for (uint32_t t = 0; t < sl->cfg.catch_up.close_timeout_ms; t += SL_POLL_MS) {
        const q16_t w_ref = sl->w_ref;
        const q16_t band = q16_max(q16_mul(sl->close_frac, sl_abs(w_ref)), sl->close_min);
        if (sl_abs(q16_sub(sl->w_ctrl, w_ref)) <= band) {
            break;
        }
        out = sl_wait(sl, SL_POLL_MS, SL_M_ALL);
        if (out != SL_OK) {
            return out;
        }
    }
    return SL_OK;
}

static sl_out_t sl_catch_torque(sl_t *sl)
{
    esp_foc_critical_enter();
    sl->iq_target = sl->iq_user;
    sl->iq_step = sl->catch_step;
    sl->id_target = sl->id_user;
    sl->id_step = sl->catch_step;
    sl->speed_on = false;
    sl->slew_on = true;
    esp_foc_critical_leave();
    sl->track = true;

    const sl_out_t out = sl_wait_until(sl, &sl->iq_ref, &sl->iq_target);
    if (out != SL_OK) {
        return out;
    }
    sl->iq_step = 0;
    return SL_OK;
}

static sl_out_t sl_catch_and_run(sl_t *sl)
{
    sl->state = ESP_FOC_SL_STATE_CATCH;
    sl_out_t out = sl->cfg.speed_loop ? sl_catch_speed(sl) : sl_catch_torque(sl);
    if (out != SL_OK) {
        return out;
    }
    sl->state = ESP_FOC_SL_STATE_RUNNING;
    sl_emit_simple(sl, ESP_FOC_SL_EV_RUNNING);
    for (;;) {
        out = sl_wait(sl, SL_POLL_MS, SL_M_ALL);
        if (out != SL_OK) {
            return out;
        }
    }
}

/* Decelerate to the handoff speed (speed) or to zero torque, still on
 * theta_obs, then hand back for the coast and the relaunch. */
static sl_out_t sl_reverse(sl_t *sl)
{
    esp_foc_sensorless_event_t e;

    sl->track = false;
    sl->state = ESP_FOC_SL_STATE_REVERSING;
    e = sl_event(sl, ESP_FOC_SL_EV_REVERSING);
    e.dir = (int8_t)-sl->dir;
    sl_emit(sl, &e);

    volatile q16_t *x;
    volatile q16_t *target;
    q16_t span;
    q16_t rate;
    esp_foc_critical_enter();
    if (sl->cfg.speed_loop) {
        sl->w_target = (sl->dir > 0) ? sl->w_plateau : q16_neg(sl->w_plateau);
        sl->w_step = sl->rev_w_step;
        x = &sl->w_ref;
        target = &sl->w_target;
        rate = sl->rev_w_step;
    } else {
        sl->iq_target = 0;
        sl->iq_step = sl->rev_iq_step;
        x = &sl->iq_ref;
        target = &sl->iq_target;
        rate = sl->rev_iq_step;
    }
    span = sl_abs(q16_sub(*target, *x));
    esp_foc_critical_leave();

    /* Slot ticks the ramp needs, as wall time, plus margin for a slow
     * observer: the deadline only exists so a stuck ramp cannot hold the
     * bridge on. */
    const uint32_t ticks = (uint32_t)(span / ((rate > 0) ? rate : 1)) + 1u;
    const uint32_t cap_ms =
        (uint32_t)((uint64_t)ticks * 1000u * SL_SLOW_DIV / sl->pwm_hz) + 2000u;
    for (uint32_t t = 0; (t < cap_ms) && (*x != *target); t += SL_POLL_MS) {
        const sl_out_t out = sl_wait(sl, SL_POLL_MS, SL_M_REF | SL_M_GUARD);
        if (out != SL_OK) {
            return out;
        }
    }
    return SL_OK;
}

static void sl_latch(sl_t *sl, sl_out_t out)
{
    esp_foc_sensorless_event_t e = sl_event(sl, ESP_FOC_SL_EV_STARTUP_FAILED);
    if (out == SL_ABORT) {
        e.ev = ESP_FOC_SL_EV_ABORT;
        e.abort = (esp_foc_sensorless_abort_t)sl->abort;
        sl->abort_seen = e.abort;
    } else if (out == SL_FAULT) {
        e.ev = ESP_FOC_SL_EV_FAULT;
        e.fault = (esp_foc_fault_reason_t)sl->fault;
        sl->fault_seen = e.fault;
    } else {
        e.fail = sl->fail;
    }
    sl->latched = true;
    sl->state = ESP_FOC_SL_STATE_FAULT;
    sl_emit(sl, &e);
}

static void sl_req_done(sl_t *sl, uint32_t bit, esp_err_t err)
{
    sl->req_err = err;
    esp_foc_critical_enter();
    sl->req &= ~bit;
    esp_foc_critical_leave();
}

static void sl_finish(sl_t *sl, sl_out_t out)
{
    sl_cut(sl);
    switch (out) {
    case SL_CUT:
        sl_emit_simple(sl, ESP_FOC_SL_EV_CUT);
        sl->state = ESP_FOC_SL_STATE_ARMED;
        sl_emit_simple(sl, ESP_FOC_SL_EV_ARMED);
        break;
    case SL_STOP:
        sl->state = ESP_FOC_SL_STATE_IDLE;
        sl_emit_simple(sl, ESP_FOC_SL_EV_STOPPED);
        sl_req_done(sl, SL_REQ_STOP, ESP_OK);
        break;
    case SL_FAILED:
    case SL_ABORT:
    case SL_FAULT:
        sl_latch(sl, out);
        break;
    default:
        break;
    }
}

static void sl_drive(sl_t *sl, int8_t dir)
{
    for (;;) {
        sl_out_t out = sl_launch(sl, dir);
        const bool launched = (out == SL_OK);
        if (launched) {
            out = sl_catch_and_run(sl);
        }
        if (out == SL_SIGN) {
            if (launched) {
                out = sl_reverse(sl);
            } else {
                esp_foc_sensorless_event_t e = sl_event(sl, ESP_FOC_SL_EV_REVERSING);
                e.dir = (int8_t)-dir;
                sl_emit(sl, &e);
                out = SL_OK;
            }
            if (out == SL_OK) {
                sl_cut(sl);
                out = sl_wait(sl, sl->cfg.coast_ms, 0u);
                if (out == SL_OK) {
                    dir = sl_ref_dir(sl);
                    if (dir != 0) {
                        continue;
                    }
                    out = SL_CUT;
                }
            }
        }
        sl_finish(sl, out);
        return;
    }
}

static esp_err_t sl_clear(sl_t *sl)
{
    esp_foc_inverter_t *inv = sl->inv;
    if (!sl->latched) {
        return ESP_ERR_INVALID_STATE;
    }
    if (inv->is_faulted(inv)) {
        const esp_err_t err = inv->clear_fault(inv);
        if (err != ESP_OK) {
            return err;
        }
    }
    esp_foc_critical_enter();
    sl->fault = (uint8_t)ESP_FOC_FAULT_NONE;
    sl->abort = (uint8_t)ESP_FOC_SL_ABORT_NONE;
    esp_foc_critical_leave();
    sl->fail = ESP_FOC_SL_FAIL_NONE;
    sl->abort_seen = ESP_FOC_SL_ABORT_NONE;
    sl->fault_seen = ESP_FOC_FAULT_NONE;
    sl->latched = false;
    sl_emit_simple(sl, ESP_FOC_SL_EV_FAULT_CLEARED);
    if (sl->state == ESP_FOC_SL_STATE_FAULT) {
        sl->state = ESP_FOC_SL_STATE_ARMED;
        sl_emit_simple(sl, ESP_FOC_SL_EV_ARMED);
    }
    return ESP_OK;
}

static void sl_task(void *arg)
{
    sl_t *sl = (sl_t *)arg;
    sl->sup = esp_foc_event_handle_self();
    sl->task_alive = true;

    for (;;) {
        const uint32_t req = sl->req;
        if ((req & SL_REQ_EXIT) != 0u) {
            break;
        }
        if ((req & SL_REQ_STOP) != 0u) {
            sl_cut(sl);
            if (sl->state != ESP_FOC_SL_STATE_IDLE) {
                sl->state = ESP_FOC_SL_STATE_IDLE;
                sl_emit_simple(sl, ESP_FOC_SL_EV_STOPPED);
            }
            sl_req_done(sl, SL_REQ_STOP, ESP_OK);
            continue;
        }
        if ((req & SL_REQ_CLEAR) != 0u) {
            sl_req_done(sl, SL_REQ_CLEAR, sl_clear(sl));
            continue;
        }
        if ((req & SL_REQ_RUN) != 0u) {
            esp_err_t err = ESP_ERR_INVALID_STATE;
            if ((sl->state == ESP_FOC_SL_STATE_IDLE) && !sl->latched) {
                sl->state = ESP_FOC_SL_STATE_ARMED;
                sl_emit_simple(sl, ESP_FOC_SL_EV_ARMED);
                err = ESP_OK;
            }
            sl_req_done(sl, SL_REQ_RUN, err);
            continue;
        }
        if ((sl->state == ESP_FOC_SL_STATE_ARMED) &&
            (esp_foc_now_us() >= sl->coast_until_us)) {
            const int8_t d = sl_ref_dir(sl);
            if (d != 0) {
                sl_drive(sl, d);
                continue;
            }
        }
        (void)esp_foc_event_wait_ms(SL_POLL_MS);
    }

    sl_cut(sl);
    sl->task_alive = false;
    esp_foc_task_delete_self();
}

static esp_err_t sl_request(sl_t *sl, uint32_t bit)
{
    esp_foc_critical_enter();
    sl->req |= bit;
    esp_foc_critical_leave();
    esp_foc_event_post(sl->sup);
    if (esp_foc_event_handle_self() == sl->sup) {
        return ESP_OK;
    }
    for (uint32_t t = 0; t < SL_REQ_TIMEOUT_MS; t++) {
        if ((sl->req & bit) == 0u) {
            return sl->req_err;
        }
        esp_foc_sleep_ms(1);
    }
    return ESP_ERR_TIMEOUT;
}

/* ------------------------------------------------------------------------ */
/* Design                                                                    */
/* ------------------------------------------------------------------------ */

static q16_t sl_step_q16(float per_s, float slot_hz)
{
    const q16_t step = q16_from_float(per_s / slot_hz);
    return (step > 0) ? step : 1;
}

static uint32_t sl_slot_ticks(sl_t *sl, uint32_t ms)
{
    const uint32_t n = (uint32_t)((float)ms * sl->slot_hz * 0.001f + 0.5f);
    return (n > 0u) ? n : 1u;
}

static bool sl_cfg_ok(const esp_foc_sensorless_config_t *c)
{
    const bool plant = (c->rs_ohm > 0.0f) && (c->ls_h > 0.0f) && (c->psi_wb > 0.0f) &&
                       (c->pole_pairs > 0u) && (c->fe_rated_hz > 0.0f) &&
                       (c->fe_rated_hz < 3000.0f);
    const bool limits = (c->i_max_a > 0.0f) && (c->i_align_a > 0.0f) &&
                        (fabsf(c->id_run_a) <= c->i_max_a) && (c->i_bw_hz > 0.0f) &&
                        (c->kp_i >= 0.0f) && (c->ki_i >= 0.0f);
    const bool trig = (c->w_min_hz > 0.0f) && (c->iq_min_a > 0.0f) &&
                      (c->iq_min_a <= c->i_max_a) && (c->rev_decel_hz_s > 0.0f) &&
                      (c->rev_decel_a_s > 0.0f) && (c->wref_slew_hz_s > 0.0f);
    const bool speed = (c->speed_bw_hz >= 0.0f) && (c->speed_zeta > 0.4f) &&
                       (c->speed_zeta < 1.2f) && (c->kp_w >= 0.0f) && (c->ki_w >= 0.0f) &&
                       (c->speed_filt_frac > 0.0f) && (c->speed_err_sat_frac > 0.0f);
    const bool start = (c->startup.step_ms > 0u) && (c->startup.accel_rads2 > 0.0f) &&
                       (c->startup.align_timeout_ms > 0u) && (c->startup.plateau_hz >= 0.0f) &&
                       (c->startup.plateau_fbase_frac > 0.0f) &&
                       (c->startup.plateau_min_hz > 0.0f) &&
                       (c->startup.plateau_max_hz >= c->startup.plateau_min_hz) &&
                       (c->startup.vf_hz >= 0.0f) &&
                       ((c->startup.vf_hz == 0.0f) || (c->startup.vf_ramp_ms > 0u)) &&
                       (c->startup.follow_frac >= 0.0f) && (c->startup.follow_frac <= 1.0f) &&
                       (c->startup.ramp_id_err_a > 0.0f);
    const bool hand = (c->handoff.band_hz > 0.0f) && (c->handoff.hunt_hz > 0.0f) &&
                      (c->handoff.hold_ms > 0u) && (c->handoff.timeout_ms > 0u) &&
                      (c->handoff.blend_ms > 0u) && (c->handoff.we_min_frac > 0.0f) &&
                      (c->handoff.we_min_frac <= 1.0f) && (c->handoff.iq_start_a >= 0.0f) &&
                      (c->handoff.iq_start_a <= c->i_max_a);
    const bool catch_ok = (c->catch_up.floor_a >= 0.0f) && (c->catch_up.floor_a < c->i_max_a) &&
                          (c->catch_up.brake_a >= 0.0f) && (c->catch_up.brake_a < c->i_max_a) &&
                          (c->catch_up.close_frac > 0.0f) && (c->catch_up.close_min_hz > 0.0f) &&
                          (c->catch_up.slew_a_s > 0.0f) &&
                          (c->catch_up.hold_fbase_frac > 0.0f);
    const bool guard = (c->guard.bemf_frac >= 0.0f) && (c->guard.bemf_fmin_hz > 0.0f) &&
                       (c->guard.overspeed_frac > 0.0f) && (c->guard.collapse_a > 0.0f) &&
                       (c->observer.w_max_frac > c->guard.overspeed_frac) &&
                       (c->observer.w_max_frac * c->fe_rated_hz * SL_TWO_PI < 30000.0f);
    return plant && limits && trig && speed && start && hand && catch_ok && guard;
}

static esp_err_t sl_speed_design(sl_t *sl, float bw_hz, float *kp, float *ki)
{
    const float zeta = sl->cfg.speed_zeta;
    const float k_w = 2.0f * zeta * SL_TWO_PI * bw_hz / sl->kp_lim;
    return esp_foc_pid_design_integrator(k_w, sl->slot_hz, bw_hz, zeta, kp, ki);
}

static esp_err_t sl_design(sl_t *sl)
{
    const esp_foc_sensorless_config_t *c = &sl->cfg;
    const float pwm = (float)sl->pwm_hz;
    const float ts = 1.0f / pwm;
    const float vdc = sl->vdc_v;
    esp_err_t err;

    sl->slot_hz = pwm / (float)SL_SLOW_DIV;

    float kp_i = c->kp_i;
    float ki_i = c->ki_i;
    if ((kp_i == 0.0f) && (ki_i == 0.0f)) {
        err = esp_foc_pid_design_imc_zoh(vdc / c->rs_ohm, c->ls_h / c->rs_ohm, pwm, c->i_bw_hz,
                                         &kp_i, &ki_i);
        if (err != ESP_OK) {
            return err;
        }
    }
    err = esp_foc_pid_init(&sl->pi_d, kp_i, ki_i, 0.0f, 0.0f, ts);
    if (err == ESP_OK) {
        err = esp_foc_pid_init(&sl->pi_q, kp_i, ki_i, 0.0f, 0.0f, ts);
    }
    if (err != ESP_OK) {
        return err;
    }

    const float fe_r = c->fe_rated_hz;
    const float track = (c->observer.track_bw_hz > 0.0f)
                            ? c->observer.track_bw_hz
                            : fe_r * (float)CONFIG_ESP_FOC_SL_TRACK_BW_E4 * 1.0e-4f;
    const float blend = (c->observer.blend_hz > 0.0f)
                            ? c->observer.blend_hz
                            : fe_r * (float)CONFIG_ESP_FOC_SL_BLEND_E4 * 1.0e-4f;
    const float speed_bw = (c->speed_bw_hz > 0.0f)
                               ? c->speed_bw_hz
                               : fe_r * (float)CONFIG_ESP_FOC_SL_SPEED_BW_E4 * 1.0e-4f;

    /* SVM phase-peak ceiling Vdc/sqrt(3) over psi_f: the speed the BEMF alone
     * fills the linear range. */
    sl->f_base_hz = vdc * SL_INV_SQRT3 / (SL_TWO_PI * c->psi_wb);
    int plateau = (int)(c->startup.plateau_hz + 0.5f);
    if (plateau <= 0) {
        plateau = (int)(sl->f_base_hz * c->startup.plateau_fbase_frac + 0.5f);
        /* Under the BEMF gate a stalled rotor reads as healthy. */
        if (plateau < (int)(c->startup.plateau_min_hz + 0.5f)) {
            plateau = (int)(c->startup.plateau_min_hz + 0.5f);
        }
        if ((float)plateau < c->guard.bemf_fmin_hz) {
            plateau = (int)(c->guard.bemf_fmin_hz + 0.5f);
        }
        if (plateau > (int)(c->startup.plateau_max_hz + 0.5f)) {
            plateau = (int)(c->startup.plateau_max_hz + 0.5f);
        }
    }
    if ((plateau <= 0) || ((c->startup.vf_hz > 0.0f) && (c->startup.vf_hz >= (float)plateau))) {
        return ESP_ERR_INVALID_ARG;
    }
    sl->plateau_hz = plateau;
    sl->we_min_hz = (float)plateau * c->handoff.we_min_frac;

    uint32_t unlock = c->guard.lock_loss_ms * sl->pwm_hz / 1000u;
    if (unlock > 65535u) {
        unlock = 65535u;
    }
    const esp_foc_observer_flux_config_t ocfg = {
        .rs_ohm = c->rs_ohm,
        .ls_h = c->ls_h,
        .psi_f_wb = c->psi_wb,
        .ts_s = ts,
        .obs_bw_hz = c->observer.obs_bw_hz,
        .obs_zeta = c->observer.obs_zeta,
        .track_bw_hz = track,
        .track_zeta = c->observer.track_zeta,
        .blend_hz = blend,
        .psi_lock_frac = c->observer.psi_lock_frac,
        .w_max_rads = SL_TWO_PI * fe_r * c->observer.w_max_frac,
        .lock_count = c->observer.lock_count,
        .unlock_count = (uint16_t)((unlock > 0u) ? unlock : 1u),
    };
    err = esp_foc_observer_flux_init(&sl->obs, &ocfg);
    if (err != ESP_OK) {
        return err;
    }

    /* Kp from the limits: the speed error that saturates iq. The integrator
     * plant gain is then whatever puts that Kp at the requested bandwidth. */
    sl->kp_lim = c->i_max_a / (SL_TWO_PI * c->speed_err_sat_frac * fe_r);
    const float filt = speed_bw * c->speed_filt_frac;
    if (c->speed_loop && ((speed_bw >= track) || (filt > 0.25f * sl->slot_hz))) {
        return ESP_ERR_INVALID_ARG;
    }
    float kp_w = c->kp_w;
    float ki_w = c->ki_w;
    if ((kp_w == 0.0f) && (ki_w == 0.0f)) {
        err = sl_speed_design(sl, speed_bw, &kp_w, &ki_w);
        if (err != ESP_OK) {
            return err;
        }
    }
    err = esp_foc_pid_init(&sl->pi_w, kp_w, ki_w, 0.0f, 0.0f, 1.0f / sl->slot_hz);
    if (err != ESP_OK) {
        return err;
    }
    sl->w_filt_b0 = q16_from_float(1.0f - expf(-SL_TWO_PI * filt / sl->slot_hz));

    sl->tune.kp_i = kp_i;
    sl->tune.ki_i = ki_i;
    sl->tune.speed_bw_hz = speed_bw;
    sl->tune.kp_w = kp_w;
    sl->tune.ki_w = ki_w;
    sl->tune.speed_filt_hz = filt;
    sl->tune.track_bw_hz = track;
    sl->tune.blend_hz = blend;
    sl->tune.slot_hz = sl->slot_hz;

    sl->vdc = q16_from_float(vdc);
    sl->inv_vdc = q16_from_float(1.0f / vdc);
    sl->rs_pu = q16_from_float(c->rs_ohm / vdc);
    sl->psi = q16_from_float(c->psi_wb);
    sl->follow_k2 = q16_from_float(c->startup.follow_frac * c->startup.follow_frac);
    sl->ramp_id_err = q16_from_float(c->startup.ramp_id_err_a);
    sl->acq_band = q16_from_float(SL_TWO_PI * c->handoff.band_hz);
    sl->acq_hunt = q16_from_float(SL_TWO_PI * c->handoff.hunt_hz);
    sl->iq_start = q16_from_float(c->handoff.iq_start_a);
    sl->w_plateau = q16_from_float(SL_TWO_PI * (float)plateau);
    sl->we_min = q16_from_float(SL_TWO_PI * sl->we_min_hz);
    sl->w_hold_max = q16_max(q16_from_float(SL_TWO_PI * c->catch_up.hold_fbase_frac *
                                             sl->f_base_hz),
                              sl->w_plateau);
    sl->catch_floor = q16_from_float(c->catch_up.floor_a);
    sl->brake = q16_from_float(c->catch_up.brake_a);
    sl->i_max = q16_from_float(c->i_max_a);
    sl->close_frac = q16_from_float(c->catch_up.close_frac);
    sl->close_min = q16_from_float(SL_TWO_PI * c->catch_up.close_min_hz);
    sl->w_cut = q16_from_float(SL_TWO_PI * c->w_min_hz);
    sl->iq_cut = q16_from_float(c->iq_min_a);
    sl->rev_w_step = sl_step_q16(SL_TWO_PI * c->rev_decel_hz_s, sl->slot_hz);
    sl->rev_iq_step = sl_step_q16(c->rev_decel_a_s, sl->slot_hz);
    sl->catch_step = sl_step_q16(c->catch_up.slew_a_s, sl->slot_hz);
    sl->w_slew_user = sl_step_q16(SL_TWO_PI * c->wref_slew_hz_s, sl->slot_hz);
    sl->blend_du = q16_from_float(1.0f / (pwm * 0.001f * (float)c->handoff.blend_ms));
    sl->id_user = q16_from_float(c->id_run_a);

    sl->g_bemf_k = q16_from_float(c->guard.bemf_frac * c->psi_wb);
    sl->g_w_fmin = q16_from_float(SL_TWO_PI * c->guard.bemf_fmin_hz);
    sl->g_w_trip = q16_from_float(SL_TWO_PI * fe_r * c->guard.overspeed_frac);
    sl->g_i_col = q16_from_float(c->guard.collapse_a);
    sl->g_iq_col_ref = q16_from_float(SL_COLLAPSE_IQREF_A);
    sl->g_bemf_need = sl_slot_ticks(sl, c->guard.bemf_hold_ms);
    sl->g_ov_need = sl_slot_ticks(sl, c->guard.overspeed_hold_ms);
    sl->g_col_need = sl_slot_ticks(sl, c->guard.collapse_hold_ms);
    sl->g_loss_need = sl_slot_ticks(sl, c->guard.lock_loss_ms);
    sl->g_loss_abort = c->guard.lock_loss_abort;

    esp_foc_if_config_t *ic = &sl->ifs.cfg;
    memset(&sl->ifs, 0, sizeof(sl->ifs));
    ic->i_align_a = c->i_align_a;
    ic->align_ms = c->startup.align_ms;
    ic->align_settle_ms = c->startup.align_settle_ms;
    ic->align_timeout_ms = c->startup.align_timeout_ms;
    ic->f_min_hz = plateau;
    ic->f_max_hz = plateau;
    ic->lock_timeout_ms = c->startup.follow_timeout_ms;
    ic->dt_ms = c->startup.step_ms;
    ic->accel_rads2 = c->startup.accel_rads2;
    ic->vf_hz = c->startup.vf_hz;
    ic->vf_ramp_ms = c->startup.vf_ramp_ms;
    /* What it takes to put i_align through the winding after the bridge has
     * eaten its dead zone; the slope is the BEMF a following rotor makes. */
    ic->vf_boost = (c->v_deadzone_v + c->rs_ohm * c->i_align_a + c->startup.vf_margin_v) / vdc;
    ic->vf_per_hz = SL_TWO_PI * c->psi_wb / vdc;

    esp_foc_if_ops_t *op = &sl->ifs.ops;
    op->ctx = sl;
    op->set_idq = sl_if_set_idq;
    op->set_fe_hz = sl_if_set_fe;
    op->sleep_ms = sl_if_sleep;
    op->poll = sl_if_poll;
    op->get_dang = sl_if_dang;
    op->lock_ok = sl_if_lock_ok;
    op->do_handoff = sl_if_handoff;
    op->on_plateau = sl_if_on_plateau;
    op->align_ok = sl_if_align_ok;
    op->ramp_ok = sl_if_ramp_ok;
    if (c->startup.vf_hz > 0.0f) {
        op->set_vdq = sl_if_set_vdq;
        op->to_current_mode = sl_if_to_current_mode;
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */
/* Public API                                                                */
/* ------------------------------------------------------------------------ */

void esp_foc_sensorless_default_config(esp_foc_sensorless_config_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));
    cfg->i_max_a = (float)CONFIG_ESP_FOC_SL_I_MAX_MA * 1.0e-3f;
    cfg->i_align_a = (float)CONFIG_ESP_FOC_SL_ALIGN_MA * 1.0e-3f;
    cfg->id_run_a = (float)CONFIG_ESP_FOC_SL_ID_RUN_MA * 1.0e-3f;
    cfg->i_bw_hz = (float)CONFIG_ESP_FOC_SL_I_BW_HZ;
    cfg->speed_loop = true;
    cfg->speed_zeta = (float)CONFIG_ESP_FOC_SL_SPEED_ZETA_PERMIL * 1.0e-3f;
    cfg->speed_filt_frac = (float)CONFIG_ESP_FOC_SL_SPEED_FILT_PERMIL * 1.0e-3f;
    cfg->speed_err_sat_frac = (float)CONFIG_ESP_FOC_SL_SPEED_ERR_SAT_PERMIL * 1.0e-3f;
    cfg->wref_slew_hz_s = (float)CONFIG_ESP_FOC_SL_WREF_SLEW_HZ_S;
    cfg->w_min_hz = (float)CONFIG_ESP_FOC_SL_W_MIN_HZ;
    cfg->iq_min_a = (float)CONFIG_ESP_FOC_SL_IQ_MIN_MA * 1.0e-3f;
    cfg->rev_decel_hz_s = (float)CONFIG_ESP_FOC_SL_REV_DECEL_HZ_S;
    cfg->rev_decel_a_s = (float)CONFIG_ESP_FOC_SL_REV_DECEL_MA_S * 1.0e-3f;
    cfg->coast_ms = CONFIG_ESP_FOC_SL_COAST_MS;

    cfg->startup.cal_rounds = CONFIG_ESP_FOC_SL_CAL_ROUNDS;
    cfg->startup.arm_ms = CONFIG_ESP_FOC_SL_ARM_MS;
    cfg->startup.align_ms = CONFIG_ESP_FOC_SL_ALIGN_MS;
    cfg->startup.align_settle_ms = CONFIG_ESP_FOC_SL_ALIGN_SETTLE_MS;
    cfg->startup.align_timeout_ms = CONFIG_ESP_FOC_SL_ALIGN_TIMEOUT_MS;
    cfg->startup.step_ms = CONFIG_ESP_FOC_SL_STEP_MS;
    cfg->startup.accel_rads2 = (float)CONFIG_ESP_FOC_SL_ACCEL_RADS2;
    cfg->startup.plateau_fbase_frac = (float)CONFIG_ESP_FOC_SL_PLATEAU_FBASE_PERMIL * 1.0e-3f;
    cfg->startup.plateau_min_hz = (float)CONFIG_ESP_FOC_SL_PLATEAU_MIN_HZ;
    cfg->startup.plateau_max_hz = (float)CONFIG_ESP_FOC_SL_PLATEAU_MAX_HZ;
    cfg->startup.vf_hz = (float)CONFIG_ESP_FOC_SL_VF_HZ;
    cfg->startup.vf_ramp_ms = CONFIG_ESP_FOC_SL_VF_RAMP_MS;
    cfg->startup.vf_margin_v = (float)CONFIG_ESP_FOC_SL_VF_MARGIN_MV * 1.0e-3f;
    cfg->startup.follow_frac = (float)CONFIG_ESP_FOC_SL_FOLLOW_PERMIL * 1.0e-3f;
    cfg->startup.follow_hold_ms = CONFIG_ESP_FOC_SL_FOLLOW_HOLD_MS;
    cfg->startup.follow_timeout_ms = CONFIG_ESP_FOC_SL_FOLLOW_TIMEOUT_MS;
    cfg->startup.ramp_id_err_a = (float)CONFIG_ESP_FOC_SL_RAMP_ID_ERR_MA * 1.0e-3f;
    cfg->startup.ramp_id_err_ms = CONFIG_ESP_FOC_SL_RAMP_ID_ERR_MS;

    cfg->handoff.band_hz = (float)CONFIG_ESP_FOC_SL_ACQ_BAND_HZ;
    cfg->handoff.hunt_hz = (float)CONFIG_ESP_FOC_SL_ACQ_HUNT_HZ;
    cfg->handoff.hold_ms = CONFIG_ESP_FOC_SL_ACQ_HOLD_MS;
    cfg->handoff.timeout_ms = CONFIG_ESP_FOC_SL_ACQ_TIMEOUT_MS;
    cfg->handoff.blend_ms = CONFIG_ESP_FOC_SL_BLEND_MS;
    cfg->handoff.settle_ms = CONFIG_ESP_FOC_SL_SETTLE_MS;
    cfg->handoff.iq_start_a = (float)CONFIG_ESP_FOC_SL_IQ_START_MA * 1.0e-3f;
    cfg->handoff.we_min_frac = (float)CONFIG_ESP_FOC_SL_WE_MIN_PERMIL * 1.0e-3f;

    cfg->catch_up.floor_a = (float)CONFIG_ESP_FOC_SL_CATCH_FLOOR_MA * 1.0e-3f;
    cfg->catch_up.brake_a = (float)CONFIG_ESP_FOC_SL_BRAKE_MA * 1.0e-3f;
    cfg->catch_up.hold_ms = CONFIG_ESP_FOC_SL_CATCH_HOLD_MS;
    cfg->catch_up.hold_fbase_frac = (float)CONFIG_ESP_FOC_SL_CATCH_HOLD_FBASE_PERMIL * 1.0e-3f;
    cfg->catch_up.close_frac = (float)CONFIG_ESP_FOC_SL_CLOSE_PERMIL * 1.0e-3f;
    cfg->catch_up.close_min_hz = (float)CONFIG_ESP_FOC_SL_CLOSE_MIN_HZ;
    cfg->catch_up.close_timeout_ms = CONFIG_ESP_FOC_SL_CLOSE_TIMEOUT_MS;
    cfg->catch_up.slew_a_s = (float)CONFIG_ESP_FOC_SL_CATCH_SLEW_MA_S * 1.0e-3f;

    cfg->observer.obs_bw_hz = (float)CONFIG_ESP_FOC_SL_OBS_BW_HZ;
    cfg->observer.obs_zeta = (float)CONFIG_ESP_FOC_SL_OBS_ZETA_PERMIL * 1.0e-3f;
    cfg->observer.track_zeta = (float)CONFIG_ESP_FOC_SL_TRACK_ZETA_PERMIL * 1.0e-3f;
    cfg->observer.psi_lock_frac = (float)CONFIG_ESP_FOC_SL_PSI_LOCK_PERMIL * 1.0e-3f;
    cfg->observer.w_max_frac = (float)CONFIG_ESP_FOC_SL_W_MAX_PERMIL * 1.0e-3f;
    cfg->observer.lock_count = CONFIG_ESP_FOC_SL_LOCK_COUNT;

    cfg->guard.bemf_frac = (float)CONFIG_ESP_FOC_SL_BEMF_PERMIL * 1.0e-3f;
    cfg->guard.bemf_hold_ms = CONFIG_ESP_FOC_SL_BEMF_HOLD_MS;
    cfg->guard.bemf_fmin_hz = (float)CONFIG_ESP_FOC_SL_BEMF_FMIN_HZ;
    cfg->guard.overspeed_frac = (float)CONFIG_ESP_FOC_SL_OVERSPEED_PERMIL * 1.0e-3f;
    cfg->guard.overspeed_hold_ms = CONFIG_ESP_FOC_SL_OVERSPEED_HOLD_MS;
    cfg->guard.collapse_a = (float)CONFIG_ESP_FOC_SL_COLLAPSE_MA * 1.0e-3f;
    cfg->guard.collapse_hold_ms = CONFIG_ESP_FOC_SL_COLLAPSE_HOLD_MS;
    cfg->guard.lock_loss_ms = CONFIG_ESP_FOC_SL_LOCK_LOSS_MS;
#if CONFIG_ESP_FOC_SL_LOCK_LOSS_ABORT
    cfg->guard.lock_loss_abort = true;
#endif
    esp_foc_phase_map_identity(&cfg->map);
}

#if CONFIG_ESP_FOC_ENABLE_MOTOR_ID
void esp_foc_sensorless_config_from_motor_id(esp_foc_sensorless_config_t *cfg,
                                             const esp_foc_motor_id_result_t *r)
{
    if ((cfg == NULL) || (r == NULL)) {
        return;
    }
    if ((r->valid_mask & ESP_FOC_MOTOR_ID_VALID_R_LOOP) != 0u) {
        cfg->rs_ohm = r->r_loop_ohm;
    } else if ((r->valid_mask & ESP_FOC_MOTOR_ID_VALID_RS) != 0u) {
        cfg->rs_ohm = r->rs_ohm;
    }
    if ((r->valid_mask & ESP_FOC_MOTOR_ID_VALID_LS) != 0u) {
        cfg->ls_h = r->ls_h;
    }
    if ((r->valid_mask & ESP_FOC_MOTOR_ID_VALID_PSI_F) != 0u) {
        cfg->psi_wb = r->psi_f_wb;
    }
    if ((r->valid_mask & ESP_FOC_MOTOR_ID_VALID_PP) != 0u) {
        cfg->pole_pairs = r->pole_pairs;
    }
    if (r->v_deadzone_v > 0.0f) {
        cfg->v_deadzone_v = r->v_deadzone_v;
    }
}
#endif

void esp_foc_sensorless_config_from_phase_map(esp_foc_sensorless_config_t *cfg,
                                              const esp_foc_phase_discover_result_t *r)
{
    if ((cfg == NULL) || (r == NULL) || !esp_foc_phase_map_valid(&r->map)) {
        return;
    }
    cfg->map = r->map;
    cfg->map_valid = true;
}

esp_err_t esp_foc_sensorless_init(esp_foc_inverter_t *inv,
                                  const esp_foc_sensorless_config_t *cfg)
{
    if ((inv == NULL) || (cfg == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    sl_t *sl = sl_axis(cfg->axis);
    if (sl == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (sl->inited || sl->task_alive || !esp_foc_in_task_context()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!sl_cfg_ok(cfg) || (cfg->map_valid && !esp_foc_phase_map_valid(&cfg->map))) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint32_t pwm_hz = inv->get_pwm_rate_hz(inv);
    const float vdc = q16_to_float(inv->get_dc_link_voltage(inv));
    if ((pwm_hz == 0u) || !(vdc > 0.0f)) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(sl, 0, sizeof(*sl));
    sl->axis = cfg->axis;
    sl->inv = inv;
    sl->cfg = *cfg;
    sl->pwm_hz = pwm_hz;
    sl->vdc_v = vdc;
    sl->state = ESP_FOC_SL_STATE_IDLE;
    esp_err_t err = sl_design(sl);
    if (err != ESP_OK) {
        return err;
    }
    if (cfg->map_valid) {
        err = inv->set_phase_map(inv, &cfg->map);
        if (err != ESP_OK) {
            return err;
        }
    }

    if (esp_foc_task_spawn(sl_task, sl, k_task_name[sl->axis], CONFIG_ESP_FOC_SL_TASK_STACK,
                           esp_foc_task_max_priority() - CONFIG_ESP_FOC_SL_TASK_PRIO_BELOW_MAX,
                           NULL) != 0) {
        return ESP_ERR_NO_MEM;
    }
    while (!sl->task_alive) {
        esp_foc_sleep_ms(1);
    }

    esp_foc_critical_enter();
    inv->set_pwm_callback(inv, sl_tez, sl);
    inv->set_dma_callback(inv, sl_dma, sl);
    inv->set_fault_callback(inv, sl_fault, sl);
    esp_foc_critical_leave();
    sl->inited = true;
    return ESP_OK;
}

void esp_foc_sensorless_deinit(uint8_t axis)
{
    sl_t *sl = sl_axis(axis);
    if (sl == NULL) {
        return;
    }
    if (!sl->inited) {
        return;
    }
    esp_foc_inverter_t *inv = sl->inv;
    esp_foc_critical_enter();
    sl->req |= SL_REQ_EXIT;
    esp_foc_critical_leave();
    esp_foc_event_post(sl->sup);
    while (sl->task_alive) {
        esp_foc_sleep_ms(1);
    }
    esp_foc_critical_enter();
    inv->set_pwm_callback(inv, NULL, NULL);
    inv->set_dma_callback(inv, NULL, NULL);
    inv->set_fault_callback(inv, NULL, NULL);
    esp_foc_critical_leave();
    sl->inited = false;
}

esp_err_t esp_foc_sensorless_run(uint8_t axis)
{
    sl_t *sl = sl_axis(axis);
    if (sl == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sl->inited || sl->latched || (sl->state != ESP_FOC_SL_STATE_IDLE)) {
        return ESP_ERR_INVALID_STATE;
    }
    return sl_request(sl, SL_REQ_RUN);
}

esp_err_t esp_foc_sensorless_stop(uint8_t axis)
{
    sl_t *sl = sl_axis(axis);
    if (sl == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sl->inited) {
        return ESP_ERR_INVALID_STATE;
    }
    return sl_request(sl, SL_REQ_STOP);
}

esp_err_t esp_foc_sensorless_clear_fault(uint8_t axis)
{
    sl_t *sl = sl_axis(axis);
    if (sl == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sl->inited || !sl->latched) {
        return ESP_ERR_INVALID_STATE;
    }
    return sl_request(sl, SL_REQ_CLEAR);
}

esp_err_t esp_foc_sensorless_set_speed_ref_hz(uint8_t axis, float fe_hz)
{
    sl_t *sl = sl_axis(axis);
    if (sl == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sl->inited || !sl->cfg.speed_loop) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!(fabsf(fe_hz) <= sl->cfg.fe_rated_hz * sl->cfg.guard.overspeed_frac)) {
        return ESP_ERR_INVALID_ARG;
    }
    sl->w_user = q16_from_float(SL_TWO_PI * fe_hz);
    esp_foc_event_post(sl->sup);
    return ESP_OK;
}

esp_err_t esp_foc_sensorless_set_speed_slew(uint8_t axis, float hz_per_s)
{
    sl_t *sl = sl_axis(axis);
    if (sl == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sl->inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!(hz_per_s > 0.0f) || (SL_TWO_PI * hz_per_s / sl->slot_hz > 30000.0f)) {
        return ESP_ERR_INVALID_ARG;
    }
    sl->w_slew_user = sl_step_q16(SL_TWO_PI * hz_per_s, sl->slot_hz);
    return ESP_OK;
}

esp_err_t esp_foc_sensorless_set_iq(uint8_t axis, float a)
{
    sl_t *sl = sl_axis(axis);
    if (sl == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sl->inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!(fabsf(a) <= sl->cfg.i_max_a)) {
        return ESP_ERR_INVALID_ARG;
    }
    sl->iq_user = q16_from_float(a);
    esp_foc_event_post(sl->sup);
    return ESP_OK;
}

esp_err_t esp_foc_sensorless_set_id(uint8_t axis, float a)
{
    sl_t *sl = sl_axis(axis);
    if (sl == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sl->inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!(fabsf(a) <= sl->cfg.i_max_a)) {
        return ESP_ERR_INVALID_ARG;
    }
    sl->id_user = q16_from_float(a);
    return ESP_OK;
}

esp_err_t esp_foc_sensorless_set_vdq_ff(uint8_t axis, float vd_v, float vq_v)
{
    sl_t *sl = sl_axis(axis);
    if (sl == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sl->inited) {
        return ESP_ERR_INVALID_STATE;
    }
    const float v_max = sl->vdc_v * SL_INV_SQRT3;
    if (!(fabsf(vd_v) <= v_max) || !(fabsf(vq_v) <= v_max)) {
        return ESP_ERR_INVALID_ARG;
    }
    const q16_t vd = q16_from_float(vd_v / sl->vdc_v);
    const q16_t vq = q16_from_float(vq_v / sl->vdc_v);
    esp_foc_critical_enter();
    sl->vd_ff = vd;
    sl->vq_ff = vq;
    esp_foc_critical_leave();
    return ESP_OK;
}

static void sl_pid_take(esp_foc_pid_t *dst, const esp_foc_pid_t *src)
{
    dst->b0 = src->b0;
    dst->b1 = src->b1;
    dst->b2 = src->b2;
    dst->a1 = src->a1;
    dst->a2 = src->a2;
    dst->kp = src->kp;
    dst->ki = src->ki;
}

/* Coefficients are redesigned on a copy so the ISR never runs a half-written
 * set; the swap itself is five words under the critical section. */
static esp_err_t sl_pid_retune(esp_foc_pid_t *p, esp_foc_pid_t *p2, float kp, float ki)
{
    esp_foc_pid_t tmp = *p;
    esp_err_t err = esp_foc_pid_set_kp(&tmp, kp);
    if (err == ESP_OK) {
        err = esp_foc_pid_set_ki(&tmp, ki);
    }
    if (err != ESP_OK) {
        return err;
    }
    esp_foc_critical_enter();
    sl_pid_take(p, &tmp);
    if (p2 != NULL) {
        sl_pid_take(p2, &tmp);
    }
    esp_foc_critical_leave();
    return ESP_OK;
}

esp_err_t esp_foc_sensorless_set_current_pi(uint8_t axis, float kp, float ki)
{
    sl_t *sl = sl_axis(axis);
    if (sl == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sl->inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!(kp > 0.0f) || !(ki >= 0.0f) || (kp >= 32000.0f) || (ki / (float)sl->pwm_hz >= 32000.0f)) {
        return ESP_ERR_INVALID_ARG;
    }
    const esp_err_t err = sl_pid_retune(&sl->pi_d, &sl->pi_q, kp, ki);
    if (err == ESP_OK) {
        sl->tune.kp_i = kp;
        sl->tune.ki_i = ki;
    }
    return err;
}

esp_err_t esp_foc_sensorless_set_speed_pi(uint8_t axis, float kp, float ki)
{
    sl_t *sl = sl_axis(axis);
    if (sl == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sl->inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!(kp > 0.0f) || !(ki >= 0.0f) || (kp >= 32000.0f) || (ki / sl->slot_hz >= 32000.0f)) {
        return ESP_ERR_INVALID_ARG;
    }
    const esp_err_t err = sl_pid_retune(&sl->pi_w, NULL, kp, ki);
    if (err == ESP_OK) {
        sl->tune.kp_w = kp;
        sl->tune.ki_w = ki;
    }
    return err;
}

esp_err_t esp_foc_sensorless_set_speed_bw(uint8_t axis, float hz)
{
    sl_t *sl = sl_axis(axis);
    if (sl == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sl->inited) {
        return ESP_ERR_INVALID_STATE;
    }
    const float filt = hz * sl->cfg.speed_filt_frac;
    if (!(hz > 0.0f) || (hz >= sl->tune.track_bw_hz) || (filt > 0.25f * sl->slot_hz)) {
        return ESP_ERR_INVALID_ARG;
    }
    float kp = 0.0f;
    float ki = 0.0f;
    esp_err_t err = sl_speed_design(sl, hz, &kp, &ki);
    if (err != ESP_OK) {
        return err;
    }
    const q16_t b0 = q16_from_float(1.0f - expf(-SL_TWO_PI * filt / sl->slot_hz));
    err = sl_pid_retune(&sl->pi_w, NULL, kp, ki);
    if (err != ESP_OK) {
        return err;
    }
    sl->w_filt_b0 = b0;
    sl->tune.speed_bw_hz = hz;
    sl->tune.kp_w = kp;
    sl->tune.ki_w = ki;
    sl->tune.speed_filt_hz = filt;
    return ESP_OK;
}

esp_foc_sensorless_state_t esp_foc_sensorless_get_state(uint8_t axis)
{
    sl_t *sl = sl_axis(axis);
    if (sl == NULL) {
        return ESP_FOC_SL_STATE_IDLE;
    }
    return sl->state;
}

void esp_foc_sensorless_get_status(uint8_t axis, esp_foc_sensorless_status_t *st)
{
    sl_t *sl = sl_axis(axis);
    if (sl == NULL) {
        return;
    }
    if (st == NULL) {
        return;
    }
    esp_foc_critical_enter();
    const q16_t th = sl->theta;
    const q16_t we = sl->w_obs;
    const q16_t w_ctrl = sl->w_ctrl;
    const q16_t w_ref = sl->w_ref;
    const q16_t fe_ol = sl->fe_ol;
    const q16_t id = sl->id;
    const q16_t iq = sl->iq;
    const q16_t id_ref = sl->id_ref;
    const q16_t iq_ref = sl->iq_ref;
    const q16_t vd = sl->vd;
    const q16_t vq = sl->vq;
    const bool on_obs = (sl->theta_src == SL_SRC_OBS);
    const bool lock = sl->lock_raw;
    const uint32_t tez = sl->tez;
    esp_foc_critical_leave();

    memset(st, 0, sizeof(*st));
    st->state = sl->state;
    st->dir = sl->dir;
    st->park_on_observer = on_obs;
    st->observer_locked = lock;
    st->theta_e_rad = q16_to_float(th);
    st->we_rads = q16_to_float(we);
    st->w_ctrl_rads = q16_to_float(w_ctrl);
    st->w_ref_rads = q16_to_float(w_ref);
    st->fe_ol_hz = q16_to_float(fe_ol);
    st->id_a = q16_to_float(id);
    st->iq_a = q16_to_float(iq);
    st->id_ref_a = q16_to_float(id_ref);
    st->iq_ref_a = q16_to_float(iq_ref);
    st->vd_v = q16_to_float(vd) * sl->vdc_v;
    st->vq_v = q16_to_float(vq) * sl->vdc_v;
    st->vdc_v = sl->vdc_v;
    st->f_base_hz = sl->f_base_hz;
    st->plateau_hz = (float)sl->plateau_hz;
    st->we_min_hz = sl->we_min_hz;
    st->tez = tez;
    st->fail = sl->fail;
    st->abort = sl->abort_seen;
    st->fault = sl->fault_seen;
}

void esp_foc_sensorless_get_tuning(uint8_t axis, esp_foc_sensorless_tuning_t *t)
{
    sl_t *sl = sl_axis(axis);
    if (sl == NULL) {
        return;
    }
    if (t != NULL) {
        *t = sl->tune;
    }
}
