/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#include <string.h>

#include "esp_foc_motor_id_probe.h"
#include "espFoC/utils/esp_foc_angle.h"
#include "espFoC/utils/esp_foc_clarke.h"
#include "espFoC/utils/esp_foc_park.h"
#include "espFoC/utils/esp_foc_svm.h"
#include "espFoC/utils/esp_foc_trig.h"

/* Long enough for the slowest window the sequencer asks for, short enough that
 * a disabled bridge reports instead of hanging the run. */
#define PROBE_TIMEOUT_MS 2000u
/* Wait granularity, and therefore how fast a stopped carrier is noticed. */
#define PROBE_POLL_MS 50u
/*
 * DC injection produces torque, so the rotor snaps to the injected axis and its
 * BEMF rides on the reading while it moves. That is mechanical settling, not
 * electrical: the winding is done in a fraction of a millisecond, the shaft
 * takes on the order of a hundred. A 10 ms settle read a healthy machine as a
 * 251 per-mil asymmetry purely because the first injection happened to land on
 * the axis the rotor was already parked on.
 */
#define PROBE_DC_SETTLE 4000u
#define PROBE_DC_AVG 4000u
/* EMA over 64 samples: 3.2 ms at 20 kHz, far below the thermal time constant of
 * anything the bridge drives and far above the switching ripple. */
#define PROBE_GUARD_SHIFT 6
/*
 * Step repetition: 24 samples of window, then 16 to decay.
 *
 * The decay tail has to be several L over R so each repetition starts from the same
 * zero — 16 samples is 8 time constants on this bench and 2 on a winding ten times
 * more inductive, which is where the baseline scatter starts to grow. 96 of them
 * costs 192 ms and divides the sense floor by about ten.
 */
#define PROBE_STEP_PERIOD 40u
#define PROBE_STEP_REPS 96u
/* How much looser the DC and terminal windows are guarded; see i_abort_dc. */
#define PROBE_DC_ABORT_RATIO 2.0f

static inline q16_t q16_abs(q16_t v)
{
    return (v < 0) ? -v : v;
}

static void park_idle(esp_foc_motor_id_probe_t *p)
{
    p->mode = ESP_FOC_MOTOR_ID_PROBE_OFF;
    p->inv->set_duties(p->inv, Q16_HALF, Q16_HALF, Q16_HALF);
}

static void finish(esp_foc_motor_id_probe_t *p, esp_foc_motor_id_probe_abort_t why)
{
    p->abort_why = why;
    p->aborted = (why != ESP_FOC_MOTOR_ID_ABORT_NONE);
    park_idle(p);
    /* Auto: a test inverter delivers the PWM callback from a task. */
    esp_foc_event_post_auto(p->waiter);
}

/**
 * Ramp a DC excitation in over the settle window.
 *
 * A step of DC current is a torque impulse into an undamped spring: the rotor
 * rang at roughly eight times the settled current on a free shaft, which both
 * tripped the guard and skewed the mean over any window shorter than several
 * ring periods. Ramping excites almost none of that mode, so the rotor walks to
 * the injected axis instead of being kicked at it.
 */
static q16_t ramped(const esp_foc_motor_id_probe_t *p, q16_t v)
{
    if (p->settle_samples == 0u || p->n_win >= p->settle_samples) {
        return v;
    }
    return (q16_t)(((int64_t)v * (int64_t)p->n_win) / (int64_t)p->settle_samples);
}

static void drive_dq(esp_foc_motor_id_probe_t *p, q16_t vd_pu)
{
    q16_t d = q16_clamp(vd_pu, -p->v_max_pu, p->v_max_pu);
    q16_t q = 0;
    q16_t a;
    q16_t b;
    q16_t du;
    q16_t dv;
    q16_t dw;
    esp_foc_inv_park(p->sin_th, p->cos_th, d, q, &a, &b);
    esp_foc_svm(a, b, &du, &dv, &dw);
    p->inv->set_duties(p->inv, du, dv, dw);
}

static void drive_terminal(esp_foc_motor_id_probe_t *p)
{
    /*
     * One terminal against the other two. The star point floats, so the driven
     * phase sees R + R/2 and the reading is 2/3 of a phase-to-phase injection;
     * that scale is common to all three and the symmetry check only compares
     * them against each other.
     */
    q16_t k = q16_clamp(ramped(p, p->v_pu), 0, p->v_max_pu);
    q16_t hi = q16_clamp(q16_add(Q16_HALF, k), 0, Q16_ONE);
    q16_t lo = q16_clamp(q16_sub(Q16_HALF, k / 2), 0, Q16_ONE);
    q16_t d[3] = {lo, lo, lo};
    d[p->terminal] = hi;
    p->inv->set_duties(p->inv, d[0], d[1], d[2]);
}

bool esp_foc_motor_id_probe_isr(esp_foc_motor_id_probe_t *p)
{
    const esp_foc_motor_id_probe_mode_t mode = p->mode;
    if (mode == ESP_FOC_MOTOR_ID_PROBE_OFF) {
        return false;
    }

    esp_foc_inverter_t *inv = p->inv;
    if (inv->is_faulted(inv)) {
        finish(p, ESP_FOC_MOTOR_ID_ABORT_FAULT);
        return true;
    }

    q16_t iu;
    q16_t iv;
    q16_t iw;
    inv->fetch_currents_raw(inv, &iu, &iv, &iw);

    q16_t gu;
    q16_t gv;
    q16_t gw;
    inv->fetch_currents(inv, &gu, &gv, &gw);
    q16_t mag = q16_abs(gu);
    q16_t m2 = q16_abs(gv);
    q16_t m3 = q16_abs(gw);
    if (m2 > mag) {
        mag = m2;
    }
    if (m3 > mag) {
        mag = m3;
    }
    if (mag > p->i_peak) {
        p->i_peak = mag;
    }
    /*
     * Guard the mean, not the peak. With the bridge on and nothing applied this
     * bench reads 876 mA of peak on the filtered current and 1397 mA raw, so any
     * instantaneous ceiling low enough to protect a 300 mA probe sits inside the
     * sense noise and refuses every window. The bridge current limit is what
     * catches a genuine fast overcurrent; this only has to stop a sustained one,
     * which the average sees within PROBE_GUARD_TC samples.
     */
    p->i_avg = q16_add(p->i_avg, (mag - p->i_avg) >> PROBE_GUARD_SHIFT);
    p->n_win++;
    const q16_t ceiling =
        (mode == ESP_FOC_MOTOR_ID_PROBE_DC ||
         mode == ESP_FOC_MOTOR_ID_PROBE_TERMINAL)
            ? p->i_abort_dc
            : p->i_abort;
    if (ceiling > 0 && p->i_avg > ceiling && p->n_win > p->guard_after) {
        finish(p, ESP_FOC_MOTOR_ID_ABORT_OVERCURRENT);
        return true;
    }

    if (mode == ESP_FOC_MOTOR_ID_PROBE_TERMINAL) {
        esp_foc_ident_dcprobe_add(&p->dc[0], iu);
        esp_foc_ident_dcprobe_add(&p->dc[1], iv);
        esp_foc_ident_dcprobe_add(&p->dc[2], iw);
        drive_terminal(p);
        if (esp_foc_ident_dcprobe_done(&p->dc[0])) {
            finish(p, ESP_FOC_MOTOR_ID_ABORT_NONE);
        }
        return true;
    }

    q16_t ca;
    q16_t cb;
    q16_t id;
    q16_t iq;
    esp_foc_clarke(iu, iv, iw, &ca, &cb);
    esp_foc_park(p->sin_th, p->cos_th, ca, cb, &id, &iq);

    if (mode == ESP_FOC_MOTOR_ID_PROBE_STEP) {
        /*
         * Read, record, then drive — in that order, because the order is the
         * measurement. The edge is written on the sample indexed step_pre, so the
         * quiet samples before it are the baseline the timing is counted from, and
         * the rest of the period lets the winding decay before the next one.
         */
        const uint32_t k = (p->n_win - 1u) % p->step_period;
        const uint32_t rep = (p->n_win - 1u) / p->step_period;
        if (k < ESP_FOC_IDENT_STEP_MAX) {
            p->step_acc[k] += id;
        }
        drive_dq(p, (k >= p->step_pre && k < ESP_FOC_IDENT_STEP_MAX) ? p->v_pu : 0);
        if (rep + 1u >= p->step_reps && k + 1u >= p->step_period) {
            finish(p, ESP_FOC_MOTOR_ID_ABORT_NONE);
        }
        return true;
    }

    if (mode == ESP_FOC_MOTOR_ID_PROBE_DC) {
        esp_foc_ident_dcprobe_add(&p->dc[0], id);
        drive_dq(p, ramped(p, p->v_pu));
        if (esp_foc_ident_dcprobe_done(&p->dc[0])) {
            finish(p, ESP_FOC_MOTOR_ID_ABORT_NONE);
        }
        return true;
    }

    const q16_t v = esp_foc_ident_zprobe_step(&p->z, id);
    p->v_pu = q16_mul(v, p->inv_vdc);
    p->i_dq = id;
    drive_dq(p, p->v_pu);
    if (esp_foc_ident_zprobe_done(&p->z)) {
        finish(p, ESP_FOC_MOTOR_ID_ABORT_NONE);
    }
    return true;
}

esp_err_t esp_foc_motor_id_probe_init(esp_foc_motor_id_probe_t *p,
                                     esp_foc_inverter_t *inv,
                                     float vdc,
                                     float i_abort_a)
{
    if (p == NULL || inv == NULL || vdc <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(p, 0, sizeof(*p));
    p->inv = inv;
    p->inv_vdc = q16_from_float(1.0f / vdc);
    p->v_max_pu = Q16_INV_SQRT3;
    p->i_abort = (i_abort_a > 0.0f) ? q16_from_float(i_abort_a) : 0;
    p->i_abort_dc = (i_abort_a > 0.0f)
                        ? q16_from_float(i_abort_a * PROBE_DC_ABORT_RATIO)
                        : 0;
    p->settle_samples = PROBE_DC_SETTLE;
    p->avg_samples = PROBE_DC_AVG;
    p->timeout_ms = PROBE_TIMEOUT_MS;
    p->cos_th = Q16_ONE;
    p->mode = ESP_FOC_MOTOR_ID_PROBE_OFF;
    return ESP_OK;
}

/**
 * Arm @p mode and block until the ISR closes the window.
 *
 * The notification handle is taken here rather than at init because the probes
 * are called from whichever task runs the sequence.
 */
static bool wait_window(esp_foc_motor_id_probe_t *p,
                        esp_foc_motor_id_probe_mode_t mode,
                        uint32_t guard_after)
{
    p->n_win = 0;
    p->guard_after = guard_after;
    p->i_peak = 0;
    p->i_avg = 0;
    p->aborted = false;
    p->abort_why = ESP_FOC_MOTOR_ID_ABORT_NONE;
    p->waiter = esp_foc_event_handle_self();
    esp_foc_event_clear();
    p->mode = mode;

    /*
     * Waited in slices so a dead carrier is named.
     *
     * A trip stops the PWM timer, and the ISR is the only thing that can close a
     * window: every probe after a trip therefore reported a two second timeout,
     * which reads as a slow window and sent the search chasing the wrong thing for
     * several runs. Nothing in the ISR can report this, so the wait itself has to
     * ask the bridge.
     */
    uint32_t left = p->timeout_ms;
    while (left > 0u) {
        const uint32_t slice = (left > PROBE_POLL_MS) ? PROBE_POLL_MS : left;
        if (esp_foc_event_wait_ms(slice)) {
            return !p->aborted;
        }
        left -= slice;
        if (p->inv->is_faulted(p->inv)) {
            p->abort_why = ESP_FOC_MOTOR_ID_ABORT_FAULT;
            park_idle(p);
            return false;
        }
    }
    p->abort_why = ESP_FOC_MOTOR_ID_ABORT_TIMEOUT;
    park_idle(p);
    return false;
}

static void set_angle(esp_foc_motor_id_probe_t *p, q16_t theta)
{
    q16_t s;
    q16_t c;
    esp_foc_sincos(theta, &s, &c);
    p->sin_th = s;
    p->cos_th = c;
}

static bool probe_z(void *ctx, q16_t theta, const esp_foc_ident_excite_t *e,
                    esp_foc_ident_z_t *out)
{
    esp_foc_motor_id_probe_t *p = ctx;
    if (!esp_foc_ident_zprobe_init(&p->z, e)) {
        return false;
    }
    set_angle(p, theta);
    /* A sinusoid on a fixed axis makes no net torque, so there is nothing to ride
     * out but the guard average filling: four of its time constants. */
    if (!wait_window(p, ESP_FOC_MOTOR_ID_PROBE_AC, 4u << PROBE_GUARD_SHIFT)) {
        return false;
    }
    return esp_foc_ident_zprobe_solve(&p->z, out);
}

static void arm_dc(esp_foc_motor_id_probe_t *p)
{
    for (int i = 0; i < 3; i++) {
        esp_foc_ident_dcprobe_init(&p->dc[i], p->settle_samples,
                                   p->avg_samples);
        p->i_ma[i] = 0;
    }
}

static bool probe_dc(void *ctx, q16_t theta, q16_t vd, int32_t *i_ma)
{
    esp_foc_motor_id_probe_t *p = ctx;
    arm_dc(p);
    set_angle(p, theta);
    p->v_pu = q16_mul(vd, p->inv_vdc);
    if (!wait_window(p, ESP_FOC_MOTOR_ID_PROBE_DC, p->settle_samples)) {
        return false;
    }
    p->i_ma[0] = esp_foc_ident_dcprobe_ma(&p->dc[0]);
    *i_ma = p->i_ma[0];
    return true;
}

static bool probe_terminal(void *ctx, int terminal, q16_t v, int32_t *i_ma)
{
    esp_foc_motor_id_probe_t *p = ctx;
    if (terminal < 0 || terminal > 2) {
        return false;
    }
    arm_dc(p);
    p->terminal = terminal;
    p->v_pu = q16_mul(v, p->inv_vdc);
    if (!wait_window(p, ESP_FOC_MOTOR_ID_PROBE_TERMINAL, p->settle_samples)) {
        return false;
    }
    for (int i = 0; i < 3; i++) {
        p->i_ma[i] = esp_foc_ident_dcprobe_ma(&p->dc[i]);
    }
    int32_t driven = p->i_ma[0];
    for (int i = 1; i < 3; i++) {
        if (p->i_ma[i] > driven) {
            driven = p->i_ma[i];
        }
    }
    /*
     * The driven leg is the one carrying the positive current, not the one the
     * index names. Terminal injection runs before the phase map is trusted, and
     * on this bench the map had V and W crossed without the discovery being able
     * to tell: the sequence would then have compared three legs it had
     * mislabelled and read a healthy machine as 1:4:1. Symmetry is a property of
     * the windings, so the measurement must not depend on the labels.
     */
    *i_ma = driven;
    return true;
}

static bool probe_step(void *ctx, q16_t theta, q16_t vd, uint32_t n_pre,
                       q16_t *i_out, uint32_t n)
{
    esp_foc_motor_id_probe_t *p = ctx;
    if (i_out == NULL || n == 0u || n > ESP_FOC_IDENT_STEP_MAX ||
        n_pre == 0u || n_pre >= n) {
        return false;
    }
    set_angle(p, theta);
    p->v_pu = q16_mul(vd, p->inv_vdc);
    p->step_pre = n_pre;
    p->step_period = PROBE_STEP_PERIOD;
    p->step_reps = PROBE_STEP_REPS;
    memset(p->step_acc, 0, sizeof(p->step_acc));
    /*
     * The guard stays armed here, unlike the other windows, because this one holds
     * a DC vector for a good fraction of a second: it is the average that the
     * supply and the winding care about, and the duty cycle of the repetition keeps
     * it near half of the settled current.
     */
    if (!wait_window(p, ESP_FOC_MOTOR_ID_PROBE_STEP, 1u << PROBE_GUARD_SHIFT)) {
        return false;
    }
    for (uint32_t k = 0; k < n; k++) {
        i_out[k] = (q16_t)(p->step_acc[k] / (int32_t)PROBE_STEP_REPS);
    }
    return true;
}

void esp_foc_motor_id_probe_bind(esp_foc_motor_id_probe_t *p,
                                 esp_foc_motor_id_seq_ops_t *ops)
{
    if (p == NULL || ops == NULL) {
        return;
    }
    ops->ctx = p;
    ops->probe_z = probe_z;
    ops->probe_dc = probe_dc;
    ops->probe_terminal = probe_terminal;
    ops->probe_step = probe_step;
}
