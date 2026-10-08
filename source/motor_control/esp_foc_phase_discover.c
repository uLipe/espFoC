/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#include <string.h>

#include "sdkconfig.h"
#include "espFoC/motor_control/esp_foc_phase_discover.h"
#include "espFoC/osal/esp_foc_osal.h"
#include "espFoC/utils/esp_foc_angle.h"
#include "espFoC/utils/esp_foc_clarke.h"
#include "espFoC/utils/esp_foc_park.h"
#include "espFoC/utils/esp_foc_svm.h"
#include "espFoC/utils/esp_foc_trig.h"
#include "espFoC/utils/esp_foc_vlim.h"

#define PD_CANDIDATES      12
#define PD_MIN_SAMPLES     3
#define PD_VD_PU_MIN       0.05f
#define PD_VD_PU_MAX       0.25f
/* Lets the inverter drain the previous candidate's last DMA frames before the
 * fault check and the shunt re-zero. */
#define PD_IDLE_SETTLE_MS  20u
/* Mid duty with i_limit armed after calibrate: a candidate that leaves a false
 * offset trips here, not in the middle of its pulse. */
#define PD_ARM_SETTLE_MS   50u
#define PD_STILL_POLL_MS   5u
#define PD_IO_RETRIES      8

typedef struct {
    q16_t d, q, u, v, w;
} pd_obs_t;

typedef struct {
    esp_foc_phase_map_t map;
    q16_t score;
    int8_t idx;
} pd_cand_t;

static const uint8_t k_perms[6][3] = {
    {0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0},
};

static inline q16_t pd_abs(q16_t x)
{
    return (x < 0) ? q16_neg(x) : x;
}

static void pd_tez(void *arg)
{
    esp_foc_phase_discover_t *pd = (esp_foc_phase_discover_t *)arg;
    if (pd == NULL) {
        return;
    }
    esp_foc_inverter_t *inv = pd->inv;
    const bool drive = pd->drive;
    q16_t d = pd->vd;
    q16_t q = pd->vq;
    q16_t th = pd->theta;
    const q16_t dth = pd->dtheta;

    esp_foc_rotor_sensor_step(pd->rotor);
    if (!drive) {
        return;
    }
    if (inv->is_faulted(inv)) {
        inv->set_duties(inv, Q16_HALF, Q16_HALF, Q16_HALF);
        return;
    }
    if (dth != 0) {
        th = q16_wrap_pi(q16_add(th, dth));
    }
    q16_t s;
    q16_t c;
    q16_t a;
    q16_t b;
    q16_t du;
    q16_t dv;
    q16_t dw;
    esp_foc_vlim_dq(&d, &q, Q16_INV_SQRT3);
    esp_foc_sincos(th, &s, &c);
    esp_foc_inv_park(s, c, d, q, &a, &b);
    esp_foc_svm(a, b, &du, &dv, &dw);
    inv->set_duties(inv, du, dv, dw);
    if (dth != 0) {
        pd->theta = th;
    }
}

static void pd_fault(void *arg, esp_foc_fault_reason_t reason)
{
    esp_foc_phase_discover_t *pd = (esp_foc_phase_discover_t *)arg;
    if (pd == NULL) {
        return;
    }
    pd->fault_count++;
    pd->fault_reason = reason;
}

static void pd_set_vector(esp_foc_phase_discover_t *pd, q16_t vd, q16_t vq,
                          q16_t theta, q16_t dtheta)
{
    esp_foc_critical_enter();
    pd->vd = vd;
    pd->vq = vq;
    pd->theta = theta;
    pd->dtheta = dtheta;
    esp_foc_critical_leave();
}

static void pd_install(esp_foc_phase_discover_t *pd)
{
    esp_foc_inverter_t *inv = pd->inv;
    pd->drive = false;
    pd->fault_count = 0;
    pd->fault_reason = ESP_FOC_FAULT_NONE;
    pd_set_vector(pd, 0, 0, 0, 0);
    /* The ISRs read cb and arg as a pair; never let them see one of ours and
     * one of the previous owner's. */
    esp_foc_critical_enter();
    inv->set_pwm_callback(inv, pd_tez, pd);
    inv->set_dma_callback(inv, NULL, NULL);
    inv->set_fault_callback(inv, pd_fault, pd);
    esp_foc_critical_leave();
    pd->installed = true;
}

static void pd_emit(esp_foc_phase_discover_t *pd, const esp_foc_phase_discover_event_t *e)
{
    if (pd->cfg.on_event != NULL) {
        pd->cfg.on_event(pd->cfg.ctx, e);
    }
}

static esp_err_t pd_bridge_enable(esp_foc_phase_discover_t *pd)
{
    esp_err_t err = pd->inv->enable(pd->inv);
    if (err == ESP_OK) {
        pd->bridge_on = true;
    }
    return err;
}

/* Only undoes our own enable: disable() is not free on an idle inverter, it
 * reruns the whole stop sequence of the converter and the PWM. */
static void pd_park_idle(esp_foc_phase_discover_t *pd)
{
    esp_foc_inverter_t *inv = pd->inv;
    pd->drive = false;
    pd_set_vector(pd, 0, 0, 0, 0);
    if (!pd->bridge_on) {
        return;
    }
    inv->set_duties(inv, Q16_HALF, Q16_HALF, Q16_HALF);
    inv->disable(inv);
    pd->bridge_on = false;
}

static esp_err_t pd_apply_idle(esp_foc_phase_discover_t *pd, const esp_foc_phase_map_t *map)
{
    esp_foc_inverter_t *inv = pd->inv;
    pd_park_idle(pd);
    esp_foc_sleep_ms(PD_IDLE_SETTLE_MS);
    if (inv->is_faulted(inv)) {
        (void)inv->clear_fault(inv);
    }
    esp_err_t err = inv->set_phase_map(inv, map);
    if (err != ESP_OK) {
        return err;
    }
    err = pd_bridge_enable(pd);
    if (err != ESP_OK) {
        return err;
    }
    inv->set_duties(inv, Q16_HALF, Q16_HALF, Q16_HALF);
    /* Before the settle: calibrate holds i_limit off while the first frames
     * after re-arm can still carry the previous candidate's pulse. */
    inv->calibrate_currents(inv, pd->cfg.cal_rounds);
    inv->set_duties(inv, Q16_HALF, Q16_HALF, Q16_HALF);
    esp_foc_sleep_ms(PD_ARM_SETTLE_MS);
    if (inv->is_faulted(inv)) {
        pd_park_idle(pd);
        return ESP_FAIL;
    }
    pd->fault_count = 0;
    pd->fault_reason = ESP_FOC_FAULT_NONE;
    return ESP_OK;
}

/*
 * Between the electrical time constant (~100 us) and the mechanical one (tens
 * of ms): the vector is gone before the shaft answers it.
 */
static bool pd_pulse(esp_foc_phase_discover_t *pd, q16_t vd, q16_t vq, pd_obs_t *out)
{
    esp_foc_inverter_t *inv = pd->inv;
    int64_t sd = 0;
    int64_t sq = 0;
    int64_t su = 0;
    int64_t sv = 0;
    int64_t sw = 0;
    int32_t n = 0;

    pd_set_vector(pd, vd, vq, 0, 0);
    pd->drive = true;
    /* Sleep before each read: a frame latched before the vector reached the
     * bridge reads zero and biases id, and the admittance, low by 1/pulse_ms. */
    for (uint32_t i = 0; i < pd->cfg.pulse_ms; i++) {
        esp_foc_sleep_ms(1);
        if (inv->is_faulted(inv)) {
            pd_set_vector(pd, 0, 0, 0, 0);
            return false;
        }
        if (inv->sample_ready(inv)) {
            q16_t iu = 0;
            q16_t iv = 0;
            q16_t iw = 0;
            q16_t a;
            q16_t b;
            q16_t d;
            q16_t q;
            inv->fetch_currents(inv, &iu, &iv, &iw);
            esp_foc_clarke(iu, iv, iw, &a, &b);
            esp_foc_park(0, Q16_ONE, a, b, &d, &q);
            sd += d;
            sq += q;
            su += iu;
            sv += iv;
            sw += iw;
            n++;
        }
    }
    pd_set_vector(pd, 0, 0, 0, 0);
    if (n < PD_MIN_SAMPLES) {
        return false;
    }
    out->d = (q16_t)(sd / n);
    out->q = (q16_t)(sq / n);
    out->u = (q16_t)(su / n);
    out->v = (q16_t)(sv / n);
    out->w = (q16_t)(sw / n);
    return true;
}

/*
 * At standstill the winding answers a vector with v/Z whichever way the rotor
 * faces, so reversing the vector reverses every current. What does not
 * reverse (shunt offset, back-EMF of a drifting shaft) is not the winding,
 * and the half difference removes it.
 */
static bool pd_pulse_pair(esp_foc_phase_discover_t *pd, q16_t vd, q16_t vq, pd_obs_t *out)
{
    pd_obs_t p;
    pd_obs_t m;
    if (!pd_pulse(pd, vd, vq, &p)) {
        return false;
    }
    esp_foc_sleep_ms(pd->cfg.pulse_ms);
    if (!pd_pulse(pd, q16_neg(vd), q16_neg(vq), &m)) {
        return false;
    }
    out->d = (q16_t)(((int64_t)p.d - m.d) / 2);
    out->q = (q16_t)(((int64_t)p.q - m.q) / 2);
    out->u = (q16_t)(((int64_t)p.u - m.u) / 2);
    out->v = (q16_t)(((int64_t)p.v - m.v) / 2);
    out->w = (q16_t)(((int64_t)p.w - m.w) / 2);
    return true;
}

static bool pd_tripped(esp_foc_phase_discover_t *pd)
{
    return pd->inv->is_faulted(pd->inv) || (pd->fault_count > 0u);
}

/* One candidate: arm with the map, pulse pair, back to disabled. */
static esp_err_t pd_probe(esp_foc_phase_discover_t *pd, const esp_foc_phase_map_t *map,
                          q16_t vd, q16_t vq, pd_obs_t *obs, bool *ok, bool *tripped)
{
    esp_err_t err = pd_apply_idle(pd, map);
    if (err != ESP_OK) {
        return err;
    }
    memset(obs, 0, sizeof(*obs));
    *ok = pd_pulse_pair(pd, vd, vq, obs);
    *tripped = pd_tripped(pd);
    pd_park_idle(pd);
    return ESP_OK;
}

static void pd_rank_insert(pd_cand_t *r, int *n, const esp_foc_phase_map_t *map,
                           q16_t score, int8_t idx)
{
    int at = *n;
    while (at > 0 && r[at - 1].score < score) {
        r[at] = r[at - 1];
        at--;
    }
    r[at].map = *map;
    r[at].score = score;
    r[at].idx = idx;
    (*n)++;
}

static void pd_event_obs(esp_foc_phase_discover_event_t *e, esp_foc_phase_discover_ev_t ev,
                         int8_t idx, uint8_t attempt, const esp_foc_phase_map_t *map,
                         const pd_obs_t *o)
{
    memset(e, 0, sizeof(*e));
    e->ev = ev;
    e->idx = idx;
    e->attempt = attempt;
    if (map != NULL) {
        e->map = *map;
    }
    if (o != NULL) {
        e->id = o->d;
        e->iq = o->q;
        e->iu = o->u;
        e->iv = o->v;
        e->iw = o->w;
    }
}

static esp_err_t pd_stage_a(esp_foc_phase_discover_t *pd, uint8_t attempt,
                            pd_cand_t *rank, int *n_rank)
{
    *n_rank = 0;
    for (int cand = 0; cand < PD_CANDIDATES; cand++) {
        const int8_t sg = (cand & 1) ? -1 : 1;
        esp_foc_phase_map_t map;
        for (int L = 0; L < 3; L++) {
            map.pwm_to_hw[L] = k_perms[cand / 2][L];
            map.i_sign[L] = sg;
        }
        pd_obs_t o;
        bool ok;
        bool tripped;
        esp_err_t err = pd_probe(pd, &map, pd->vd_pu, 0, &o, &ok, &tripped);
        if (err != ESP_OK) {
            return err;
        }
        /*
         * Admission is only "real current on +d". Return-leg balance and q
         * leakage are compared across candidates: as absolute gates they sat
         * on the median of the measured population (split/id 0.643 against a
         * 0.600 gate) and refused whole sweeps.
         */
        const int64_t score = (int64_t)o.d - 2 * (int64_t)pd_abs(o.q) -
                              2 * (int64_t)pd_abs(q16_sub(o.v, o.w));
        const bool good = ok && !tripped && (o.d > pd->id_min);
        esp_foc_phase_discover_event_t e;
        pd_event_obs(&e, ESP_FOC_PHASE_DISCOVER_EV_CANDIDATE, (int8_t)cand, attempt, &map, &o);
        e.score = (q16_t)score;
        e.good = good;
        e.tripped = tripped;
        pd_emit(pd, &e);
        if (good) {
            pd_rank_insert(rank, n_rank, &map, (q16_t)score, (int8_t)cand);
        }
    }
    return (*n_rank > 0) ? ESP_OK : ESP_FAIL;
}

/*
 * Whether +Vq comes back as +Iq. A permutation moves drive and sense together,
 * so neither the best map nor its V<->W mirror can change that sign; a
 * negative answer means the mirror lives in which leg each shunt reads, where
 * no map can reach it, and a sign that differs between the two means the
 * model behind the stage does not hold.
 */
static esp_err_t pd_stage_c(esp_foc_phase_discover_t *pd, uint8_t attempt,
                            const esp_foc_phase_map_t *best)
{
    esp_foc_phase_map_t cand[2] = {*best, *best};
    cand[1].pwm_to_hw[1] = best->pwm_to_hw[2];
    cand[1].pwm_to_hw[2] = best->pwm_to_hw[1];
    q16_t iq0 = 0;
    bool good0 = false;

    for (int k = 0; k < 2; k++) {
        pd_obs_t o;
        bool ok;
        bool tripped;
        esp_err_t err = pd_probe(pd, &cand[k], 0, pd->vd_pu, &o, &ok, &tripped);
        if (err != ESP_OK) {
            return err;
        }
        const bool good = ok && !tripped && (pd_abs(o.q) > pd->id_min);
        esp_foc_phase_discover_event_t e;
        pd_event_obs(&e, ESP_FOC_PHASE_DISCOVER_EV_HANDED, (int8_t)k, attempt, &cand[k], &o);
        e.good = good;
        e.tripped = tripped;
        pd_emit(pd, &e);
        if (k == 0) {
            iq0 = o.q;
            good0 = good;
        } else if (good && ((o.q > 0) != (iq0 > 0))) {
            return ESP_ERR_INVALID_RESPONSE;
        }
    }
    return (good0 && iq0 > 0) ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

/*
 * Walk the ranking and take the first map that puts +Id on +Vd with |Iq| < Id
 * and whose phase currents close. On a two-shunt inverter the third leg is
 * -(a + b) before the map, so with uniform signs the Kirchhoff sum is zero by
 * construction and only the amplitude term can refuse.
 */
static esp_err_t pd_verify(esp_foc_phase_discover_t *pd, uint8_t attempt,
                           const pd_cand_t *rank, int n_rank,
                           esp_foc_phase_discover_result_t *out)
{
    for (int k = 0; k < n_rank; k++) {
        pd_obs_t o;
        bool ok;
        bool tripped;
        esp_err_t err = pd_probe(pd, &rank[k].map, pd->vd_pu, 0, &o, &ok, &tripped);
        if (err != ESP_OK) {
            return err;
        }
        const int64_t k_sum = pd_abs((q16_t)((int64_t)o.u + o.v + o.w));
        const int64_t k_mag = (int64_t)pd_abs(o.u) + pd_abs(o.v) + pd_abs(o.w);
        const bool ok_dq = ok && !tripped && (o.d > pd->id_min) && (pd_abs(o.q) < pd_abs(o.d));
        const bool ok_k = (k_mag >= 3 * (int64_t)pd->id_min) && (5 * k_sum <= k_mag);
        esp_foc_phase_discover_event_t e;
        pd_event_obs(&e, ESP_FOC_PHASE_DISCOVER_EV_VERIFY, (int8_t)k, attempt, &rank[k].map, &o);
        e.score = rank[k].score;
        e.value = (q16_t)k_sum;
        e.good = ok_dq && ok_k;
        e.tripped = tripped;
        pd_emit(pd, &e);
        if (ok_dq && ok_k) {
            out->map = rank[k].map;
            out->rank_idx = rank[k].idx;
            out->id_verify = o.d;
            out->iq_verify = o.q;
            out->admittance = q16_div(o.d, pd->vd_pu);
            return ESP_OK;
        }
    }
    return ESP_ERR_INVALID_RESPONSE;
}

static esp_err_t pd_once(esp_foc_phase_discover_t *pd, uint8_t attempt,
                         esp_foc_phase_discover_result_t *out)
{
    pd_cand_t rank[PD_CANDIDATES];
    int n_rank = 0;

    esp_err_t err = pd_stage_a(pd, attempt, rank, &n_rank);
    if (err != ESP_OK) {
        return err;
    }
    esp_foc_phase_discover_event_t e;
    pd_event_obs(&e, ESP_FOC_PHASE_DISCOVER_EV_RANKED, (int8_t)n_rank, attempt, &rank[0].map, NULL);
    e.score = rank[0].score;
    pd_emit(pd, &e);

    err = pd_stage_c(pd, attempt, &rank[0].map);
    if (err != ESP_OK) {
        return err;
    }
    err = pd_verify(pd, attempt, rank, n_rank, out);
    if (err != ESP_OK) {
        return err;
    }
    err = pd_apply_idle(pd, &out->map);
    pd_park_idle(pd);
    return err;
}

/*
 * The pulse pair cancels its own torque, so consecutive attempts see the same
 * rotor. Park it 120 deg elec further each time; the settle is mechanical,
 * because the next candidate's shunt re-zero must not average the rotor
 * ringing in its detent.
 */
static void pd_nudge(esp_foc_phase_discover_t *pd, uint8_t attempt)
{
    esp_foc_inverter_t *inv = pd->inv;
    const int step = attempt % 3;
    if (inv->is_faulted(inv)) {
        (void)inv->clear_fault(inv);
    }
    inv->set_duties(inv, Q16_HALF, Q16_HALF, Q16_HALF);
    if (pd_bridge_enable(pd) == ESP_OK) {
        const q16_t th = q16_wrap_pi((q16_t)(((int64_t)Q16_TWO_PI * step) / 3));
        pd_set_vector(pd, pd->vd_pu, 0, th, 0);
        pd->drive = true;
        esp_foc_sleep_ms(pd->cfg.nudge_ms);
    }
    pd_park_idle(pd);
    esp_foc_phase_discover_event_t e;
    pd_event_obs(&e, ESP_FOC_PHASE_DISCOVER_EV_NUDGE, (int8_t)step, attempt, NULL, NULL);
    e.value = pd->vd_pu;
    e.ms = pd->cfg.nudge_ms;
    pd_emit(pd, &e);
    esp_foc_sleep_ms(pd->cfg.settle_ms);
}

static esp_err_t pd_read_theta(esp_foc_phase_discover_t *pd, q16_t *theta)
{
    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < PD_IO_RETRIES; i++) {
        err = esp_foc_rotor_sensor_fetch(pd->rotor);
        if (err == ESP_OK) {
            *theta = esp_foc_rotor_sensor_get_position(pd->rotor);
            return ESP_OK;
        }
        esp_foc_sleep_ms(1);
    }
    return err;
}

static esp_err_t pd_wait_still(esp_foc_phase_discover_t *pd, uint32_t *ms_out)
{
    q16_t ref = 0;
    uint32_t calm = 0;
    uint32_t elapsed = 0;
    esp_err_t err = pd_read_theta(pd, &ref);
    *ms_out = 0;
    if (err != ESP_OK) {
        return err;
    }
    while (calm < pd->cfg.sensor.calm_ms) {
        if (elapsed >= pd->cfg.sensor.timeout_ms) {
            return ESP_ERR_TIMEOUT;
        }
        esp_foc_sleep_ms(PD_STILL_POLL_MS);
        elapsed += PD_STILL_POLL_MS;
        *ms_out = elapsed;
        if (pd_tripped(pd)) {
            return ESP_FAIL;
        }
        q16_t th = 0;
        err = pd_read_theta(pd, &th);
        if (err != ESP_OK) {
            return err;
        }
        if (pd_abs(q16_angle_delta(ref, th)) > pd->still_tol) {
            ref = th;
            calm = 0;
        } else {
            calm += PD_STILL_POLL_MS;
        }
    }
    return ESP_OK;
}

static esp_err_t pd_zero(esp_foc_phase_discover_t *pd)
{
    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < PD_IO_RETRIES; i++) {
        err = esp_foc_rotor_sensor_calibrate_offset(pd->rotor, pd->cfg.sensor.zero_samples);
        if (err != ESP_ERR_INVALID_STATE) {
            break;
        }
        esp_foc_sleep_ms(1);
    }
    return err;
}

static esp_err_t pd_sensor_stage(esp_foc_phase_discover_t *pd,
                                 esp_foc_phase_discover_result_t *out)
{
    esp_foc_inverter_t *inv = pd->inv;
    esp_foc_phase_discover_event_t e;
    const q16_t v_well = q16_clamp(q16_div(q16_mul(pd->i_well, pd->vd_pu), out->id_verify),
                                   0, q16_from_float(PD_VD_PU_MAX));

    if (inv->is_faulted(inv)) {
        (void)inv->clear_fault(inv);
    }
    inv->set_duties(inv, Q16_HALF, Q16_HALF, Q16_HALF);
    esp_err_t err = pd_bridge_enable(pd);
    if (err != ESP_OK) {
        return err;
    }
    pd->fault_count = 0;
    pd_set_vector(pd, v_well, 0, 0, pd->sweep_step);
    pd->drive = true;
    esp_foc_sleep_ms(pd->cfg.sensor.sweep_ms);
    pd_set_vector(pd, v_well, 0, 0, 0);

    pd_event_obs(&e, ESP_FOC_PHASE_DISCOVER_EV_WELL, 0, out->attempts, &out->map, NULL);
    e.value = v_well;
    e.tripped = pd_tripped(pd);
    pd_emit(pd, &e);
    if (e.tripped) {
        return ESP_FAIL;
    }

    uint32_t still_ms = 0;
    err = pd_wait_still(pd, &still_ms);
    pd_event_obs(&e, ESP_FOC_PHASE_DISCOVER_EV_STILL, 0, out->attempts, &out->map, NULL);
    e.ms = still_ms;
    e.err = err;
    e.good = (err == ESP_OK);
    e.tripped = pd_tripped(pd);
    pd_emit(pd, &e);
    if (err != ESP_OK) {
        return err;
    }

    err = pd_zero(pd);
    pd_event_obs(&e, ESP_FOC_PHASE_DISCOVER_EV_ZERO, 0, out->attempts, &out->map, NULL);
    e.err = err;
    e.good = (err == ESP_OK);
    pd_emit(pd, &e);
    if (err != ESP_OK) {
        return err;
    }
    out->sensor_zeroed = true;

    q16_t th0 = 0;
    q16_t th1 = 0;
    err = pd_read_theta(pd, &th0);
    if (err != ESP_OK) {
        return err;
    }
    pd_set_vector(pd, 0, pd->dir_vq_pu, 0, 0);
    esp_foc_sleep_ms(pd->cfg.sensor.dir_ms);
    err = pd_read_theta(pd, &th1);
    pd_park_idle(pd);
    if (err != ESP_OK) {
        return err;
    }
    const q16_t dth = q16_angle_delta(th0, th1);
    out->sensor_reversed = (dth < 0);

    pd_event_obs(&e, ESP_FOC_PHASE_DISCOVER_EV_DIR, 0, out->attempts, &out->map, NULL);
    e.value = dth;
    e.reversed = out->sensor_reversed;
    e.tripped = pd_tripped(pd);
    e.good = !e.tripped && (pd_abs(dth) >= pd->dir_min);
    pd_emit(pd, &e);
    if (e.tripped) {
        return ESP_FAIL;
    }
    return e.good ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

static esp_err_t pd_sequence(esp_foc_phase_discover_t *pd, esp_foc_phase_discover_result_t *out)
{
    esp_foc_phase_discover_event_t e;
    esp_err_t err = ESP_FAIL;

    for (uint8_t attempt = 1; attempt <= pd->cfg.tries; attempt++) {
        out->attempts = attempt;
        pd_event_obs(&e, ESP_FOC_PHASE_DISCOVER_EV_ATTEMPT, (int8_t)attempt, attempt, NULL, NULL);
        e.value = pd->vd_pu;
        pd_emit(pd, &e);
        err = pd_once(pd, attempt, out);
        if (err == ESP_OK) {
            break;
        }
        pd_event_obs(&e, ESP_FOC_PHASE_DISCOVER_EV_REFUSED, (int8_t)attempt, attempt, NULL, NULL);
        e.err = err;
        pd_emit(pd, &e);
        if (attempt < pd->cfg.tries) {
            pd_nudge(pd, attempt);
        }
    }
    if (err != ESP_OK) {
        return err;
    }
    pd_event_obs(&e, ESP_FOC_PHASE_DISCOVER_EV_MAP, out->rank_idx, out->attempts, &out->map, NULL);
    e.id = out->id_verify;
    e.iq = out->iq_verify;
    e.value = out->admittance;
    e.good = true;
    pd_emit(pd, &e);

    if (pd->rotor == NULL) {
        return ESP_OK;
    }
    return pd_sensor_stage(pd, out);
}

void esp_foc_phase_discover_default_config(esp_foc_phase_discover_config_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));
    cfg->vd_v = (float)CONFIG_ESP_FOC_PHASE_DISCOVER_VD_MV * 1.0e-3f;
    cfg->id_min_a = (float)CONFIG_ESP_FOC_PHASE_DISCOVER_ID_MIN_MA * 1.0e-3f;
    cfg->pulse_ms = CONFIG_ESP_FOC_PHASE_DISCOVER_PULSE_MS;
    cfg->cal_rounds = CONFIG_ESP_FOC_PHASE_DISCOVER_CAL_ROUNDS;
    cfg->tries = CONFIG_ESP_FOC_PHASE_DISCOVER_TRIES;
    cfg->nudge_ms = CONFIG_ESP_FOC_PHASE_DISCOVER_NUDGE_MS;
    cfg->settle_ms = CONFIG_ESP_FOC_PHASE_DISCOVER_SETTLE_MS;
    cfg->sensor.i_well_a = (float)CONFIG_ESP_FOC_PHASE_DISCOVER_WELL_MA * 1.0e-3f;
    cfg->sensor.sweep_ms = CONFIG_ESP_FOC_PHASE_DISCOVER_WELL_SWEEP_MS;
    cfg->sensor.still_tol_rad = (float)CONFIG_ESP_FOC_PHASE_DISCOVER_STILL_TOL_MRAD * 1.0e-3f;
    cfg->sensor.calm_ms = CONFIG_ESP_FOC_PHASE_DISCOVER_STILL_CALM_MS;
    cfg->sensor.timeout_ms = CONFIG_ESP_FOC_PHASE_DISCOVER_STILL_TIMEOUT_MS;
    cfg->sensor.zero_samples = CONFIG_ESP_FOC_PHASE_DISCOVER_ZERO_SAMPLES;
    cfg->sensor.dir_v = (float)CONFIG_ESP_FOC_PHASE_DISCOVER_DIR_MV * 1.0e-3f;
    cfg->sensor.dir_ms = CONFIG_ESP_FOC_PHASE_DISCOVER_DIR_MS;
    cfg->sensor.dir_min_rad = (float)CONFIG_ESP_FOC_PHASE_DISCOVER_DIR_MIN_MRAD * 1.0e-3f;
}

static float pd_clamp_pu(float v, float vdc)
{
    float pu = v / vdc;
    if (pu > PD_VD_PU_MAX) {
        pu = PD_VD_PU_MAX;
    }
    if (pu < PD_VD_PU_MIN) {
        pu = PD_VD_PU_MIN;
    }
    return pu;
}

static esp_err_t pd_check_sensor_cfg(const esp_foc_phase_discover_config_t *c, uint32_t pwm_hz)
{
    if (c->sensor.i_well_a <= 0.0f || c->sensor.sweep_ms == 0u ||
        c->sensor.still_tol_rad <= 0.0f || c->sensor.calm_ms == 0u ||
        c->sensor.timeout_ms < c->sensor.calm_ms || c->sensor.zero_samples < 1 ||
        c->sensor.dir_v <= 0.0f || c->sensor.dir_ms == 0u ||
        c->sensor.dir_min_rad <= 0.0f || pwm_hz == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

esp_err_t esp_foc_phase_discover_init(esp_foc_phase_discover_t *pd,
                                      esp_foc_inverter_t *inv,
                                      esp_foc_rotor_sensor_t *rotor,
                                      const esp_foc_phase_discover_config_t *cfg)
{
    if (pd == NULL || inv == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (pd->installed) {
        return ESP_ERR_INVALID_STATE;
    }
    /* pulse_ms is counted in 1 ms sleeps; a coarser tick stretches a 6 ms
     * probe into a torque pulse the shaft answers. */
    if (esp_foc_ms_to_ticks(1) == 0u) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    esp_foc_phase_discover_config_t c;
    if (cfg != NULL) {
        c = *cfg;
    } else {
        esp_foc_phase_discover_default_config(&c);
    }
    if (c.vd_v <= 0.0f || c.id_min_a <= 0.0f || c.pulse_ms < (uint32_t)PD_MIN_SAMPLES ||
        c.cal_rounds < 0 || c.tries == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    const float vdc = q16_to_float(inv->get_dc_link_voltage(inv));
    if (vdc <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint32_t pwm_hz = inv->get_pwm_rate_hz(inv);
    if (rotor != NULL) {
        if (!esp_foc_rotor_sensor_has_cap(rotor, ESP_FOC_ROTOR_CAP_MECH_ABS)) {
            return ESP_ERR_NOT_SUPPORTED;
        }
        esp_err_t err = pd_check_sensor_cfg(&c, pwm_hz);
        if (err != ESP_OK) {
            return err;
        }
    }

    memset(pd, 0, sizeof(*pd));
    pd->inv = inv;
    pd->rotor = rotor;
    pd->cfg = c;
    pd->vd_pu = q16_from_float(pd_clamp_pu(c.vd_v, vdc));
    pd->id_min = q16_from_float(c.id_min_a);
    if (rotor != NULL) {
        const float ticks = (float)pwm_hz * (float)c.sensor.sweep_ms * 1.0e-3f;
        pd->dir_vq_pu = q16_from_float(pd_clamp_pu(c.sensor.dir_v, vdc));
        pd->dir_min = q16_from_float(c.sensor.dir_min_rad);
        pd->still_tol = q16_from_float(c.sensor.still_tol_rad);
        pd->i_well = q16_from_float(c.sensor.i_well_a);
        pd->sweep_step = q16_from_float(6.28318531f / ticks);
    }
    pd->fault_reason = ESP_FOC_FAULT_NONE;
    pd->inited = true;
    return ESP_OK;
}

esp_err_t esp_foc_phase_discover_run(esp_foc_phase_discover_t *pd,
                                     esp_foc_phase_discover_result_t *out)
{
    if (pd == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (esp_foc_in_isr_context() || !pd->inited || pd->installed) {
        return ESP_ERR_INVALID_STATE;
    }
    memset(out, 0, sizeof(*out));
    out->rank_idx = -1;
    pd_install(pd);
    esp_err_t err = pd_sequence(pd, out);
    esp_foc_phase_discover_cleanup(pd);
    return err;
}

void esp_foc_phase_discover_cleanup(esp_foc_phase_discover_t *pd)
{
    if (pd == NULL || !pd->inited) {
        return;
    }
    if (pd->installed) {
        esp_foc_inverter_t *inv = pd->inv;
        esp_foc_critical_enter();
        inv->set_pwm_callback(inv, NULL, NULL);
        inv->set_dma_callback(inv, NULL, NULL);
        inv->set_fault_callback(inv, NULL, NULL);
        esp_foc_critical_leave();
        pd->installed = false;
    }
    pd_park_idle(pd);
}
