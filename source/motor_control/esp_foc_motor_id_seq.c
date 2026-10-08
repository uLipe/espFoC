/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#include <math.h>
#include <string.h>

#include "esp_foc_motor_id_seq.h"
#include "espFoC/utils/esp_foc_pid.h"

/* 2*pi in q16 radians. */
#define MOTOR_ID_TWO_PI_Q16 411775LL
/* Below this the demodulated response is not worth designing a loop against. */
#define MOTOR_ID_MIN_PROBE_MA 40
/* Fewer samples per period than this and the probe cannot demodulate. */
#define MOTOR_ID_MIN_PROBE_DIV 8
/* Excitation search attempts per probe frequency. */
#define MOTOR_ID_AMP_TRIES 5
/*
 * Where the trough of a biased excitation is placed, in dead zones. One would
 * leave it exactly on the edge; two keeps the whole waveform in the region where
 * the bridge has gain.
 */
#define MOTOR_ID_DEAD_MARGIN 2.0f
/* Share of the current ceiling the DC bias may spend. */
#define MOTOR_ID_BIAS_I_FRAC 0.6f
/* Share of it left for the AC amplitude when asking whether a frequency is
 * usable at all, which is asked at full drive. */
#define MOTOR_ID_REACH_I_FRAC 0.7f
/* Largest command-to-measurement delay the sweep considers, in samples. */
#define MOTOR_ID_LAG_MAX (ESP_FOC_IDENT_LAG_MAX - 1)
/* The sweep needs the two probe frequencies at least this far apart. */
#define MOTOR_ID_LAGCAL_MIN_RATIO 2
/* Steps driven before giving up on two of them agreeing. */
#define MOTOR_ID_STEP_TRIES 4
/* Quiet samples ahead of the edge, enough to average the sense floor out of the
 * baseline without stretching the window past a few time constants. */
#define MOTOR_ID_STEP_PRE 6
/* Times the probe target the step drives, to put the first sample of the rise
 * above the sense floor. */
#define MOTOR_ID_STEP_I_RATIO 2.0f
/*
 * Floor under the kernel's noise-derived bar, as a fraction of where the step
 * settles. It must sit below the first sample of the rise on the least inductive
 * winding worth driving, so it cannot be a large fraction: this is 5 per cent, and
 * one sample of a winding whose L over R is ten samples already gives 10.
 */
#define MOTOR_ID_STEP_THRESH_FRAC 0.05f
/*
 * Ceiling for the sweep frequency, as a divisor of the carrier. One sample of
 * delay is a rotation of 360/div degrees there, so this fixes the resolution the
 * sweep can possibly have: at fs/8 a step is 45 degrees and the candidates all
 * alias into each other, which is how a real 2.7 sample delay produced the
 * unordered sweep 500/472/885/290/678 per mille and no winner. fs/24 keeps a step
 * at 15 degrees while arg(Z) is still large enough to be sensitive to it.
 */
#define MOTOR_ID_LAGCAL_MAX_DIV 24
/* Times the fine probe may re-aim itself at R/L before accepting the point. */
#define MOTOR_ID_FINE_PASSES 3
/* Highest probe frequency worth using, as a multiple of the current loop's bw. */
#define MOTOR_ID_PROBE_BW_RATIO 2
#define MOTOR_ID_LAGCAL_DEFAULT_PERMIL 150

static int32_t q16_to_milli(q16_t v)
{
    return (int32_t)(((int64_t)v * 1000) >> 16);
}

static void go(esp_foc_motor_id_seq_t *s, esp_foc_motor_id_phase_t ph)
{
    s->phase = ph;
    if (s->ops.on_phase != NULL) {
        s->ops.on_phase(s->ops.ctx, ph);
    }
}

static void outputs_idle(esp_foc_motor_id_seq_t *s)
{
    if (s->ops.set_fe_hz != NULL) {
        s->ops.set_fe_hz(s->ops.ctx, 0);
    }
    if (s->ops.set_idq != NULL) {
        s->ops.set_idq(s->ops.ctx, 0, 0);
    }
    s->ops.set_vdq(s->ops.ctx, 0, 0);
}

static esp_err_t bail(esp_foc_motor_id_seq_t *s, esp_err_t err)
{
    s->result.failed_at = s->phase;
    outputs_idle(s);
    go(s, ESP_FOC_MOTOR_ID_FAIL);
    return err;
}

static bool faulted(esp_foc_motor_id_seq_t *s)
{
    return (s->ops.faulted != NULL) && s->ops.faulted(s->ops.ctx);
}

static void nap(esp_foc_motor_id_seq_t *s, uint32_t ms)
{
    if (ms > 0u) {
        s->ops.sleep_ms(s->ops.ctx, ms);
    }
}

void esp_foc_motor_id_seq_default_config(esp_foc_motor_id_seq_config_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));
    cfg->v_probe_frac = 0.035f;
    cfg->v_probe_frac_max = 0.35f;
    cfg->i_probe_target_a = 0.25f;
    cfg->i_probe_max_a = 1.0f;
    cfg->v_dc_probe_frac = 0.06f;
    /* Typical for a low-voltage bridge with about a microsecond of deadtime plus
     * the diode drops. Every AC probe is biased past it. */
    cfg->v_deadzone_v = 0.6f;
    cfg->coarse_hz = 100u;
    cfg->probe_periods = 32u;
    cfg->settle_periods = 2u;
    /* Seed only; LAGCAL measures it. One PWM period of sense latency, with the
     * half-period ZOH removed in the solve. */
    cfg->lag_samples = 1u;
    cfg->lag_cal_limit_permil = MOTOR_ID_LAGCAL_DEFAULT_PERMIL;
    cfg->sym_limit_permil = 150;
    cfg->tune_backoff = 0.5f;
    cfg->i_flux_a = 0.30f;
    cfg->flux_hz = 100;
    cfg->ramp_ms = 600u;
    cfg->flux_hold_ms = 400u;
    cfg->dt_ms = 20u;
}

esp_err_t esp_foc_motor_id_seq_gains_for(const esp_foc_motor_id_seq_config_t *cfg,
                                         float r_ohm, float l_h,
                                         float *kp, float *ki)
{
    if (cfg == NULL || kp == NULL || ki == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (r_ohm <= 0.0f || l_h <= 0.0f || cfg->pwm_hz == 0u || cfg->vdc <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Auto bandwidth stays well under the InstaSPIN default of about fs/25,
     * which this bench could not hold. */
    float bw = cfg->i_bw_hz;
    if (bw <= 0.0f) {
        bw = (float)cfg->pwm_hz / 64.0f;
    }

    float kp_raw = 0.0f;
    float ki_raw = 0.0f;
    /* Plant gain is Vdc/R because the loop output is per-unit of Vdc. */
    esp_err_t err = esp_foc_pid_design_imc_zoh(cfg->vdc / r_ohm,
                                               l_h / r_ohm,
                                               (float)cfg->pwm_hz,
                                               bw,
                                               &kp_raw,
                                               &ki_raw);
    if (err != ESP_OK) {
        return err;
    }

    float backoff = (cfg->tune_backoff > 0.0f) ? cfg->tune_backoff : 1.0f;
    *kp = kp_raw * backoff;
    *ki = ki_raw * backoff;
    return ESP_OK;
}

static esp_err_t check_args(const esp_foc_motor_id_seq_t *s)
{
    const esp_foc_motor_id_seq_ops_t *o = &s->ops;
    if (o->set_vdq == NULL || o->set_theta == NULL || o->sleep_ms == NULL ||
        o->probe_z == NULL || o->apply_gains == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s->cfg.skip_terminal && o->probe_terminal == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s->cfg.skip_rs && o->probe_dc == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_foc_motor_id_seq_config_t *c = &s->cfg;
    if (c->vdc <= 0.0f || c->pwm_hz == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    if (c->pole_pairs <= 0 && o->fetch_rotor == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (c->probe_periods < 4u || c->v_probe_frac <= 0.0f ||
        c->v_dc_probe_frac <= 0.0f || c->coarse_hz == 0u ||
        c->i_probe_target_a <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    if (c->do_flux && (o->set_idq == NULL || o->set_fe_hz == NULL ||
                       o->fetch_dq == NULL || c->flux_hz <= 0)) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static void fill_excite(const esp_foc_motor_id_seq_t *s, uint32_t f_hz,
                        esp_foc_ident_excite_t *e)
{
    const esp_foc_motor_id_seq_config_t *c = &s->cfg;
    memset(e, 0, sizeof(*e));
    e->v_amp = q16_from_float(c->v_probe_frac * c->vdc);
    e->f_hz = f_hz;
    e->fs_hz = c->pwm_hz;
    e->periods = c->probe_periods;
    e->settle_periods = c->settle_periods;
    e->lag_samples = s->lag_used;
    e->phase_trim_cdeg = c->phase_trim_cdeg + s->trim_used;
}

static bool response_sane(const esp_foc_motor_id_seq_t *s, int32_t i_ma)
{
    int32_t limit = (int32_t)(s->cfg.i_probe_max_a * 1000.0f);
    int32_t mag = (i_ma < 0) ? -i_ma : i_ma;
    return (mag >= MOTOR_ID_MIN_PROBE_MA) && (limit <= 0 || mag <= limit);
}

/**
 * DC term that keeps an AC probe of amplitude @p v_amp from reversing the
 * command.
 *
 * The bridge has no gain until the command clears its deadtime dead zone, so an
 * excitation that crosses zero is demodulated from a clipped waveform: this
 * bench read R at twice its value and L negative until the bias went in.
 *
 * The condition is on voltage and it is exact — the trough, `bias - v_amp`, has
 * to sit clear of the dead zone — so the head is the amplitude itself. Sizing it
 * from `target * R` instead assumed the amplitude search had already landed on
 * the target current, and at 833 Hz it had not: it asked for 2.25 V against a
 * 1.49 V bias, the command reversed, and the clipped window published R lower at
 * 833 Hz than at 100 with |Z| below the DC resistance. Both are impossible for a
 * winding, and either is enough to leave L undefined at the fine point.
 *
 * What the head costs is `head / R` of DC current, which on an inductive machine
 * is |Z|/R times the AC current being probed with — a factor of thirteen on a
 * 40 mH winding at 100 Hz, which would trip the bridge rather than measure it.
 * So it is capped by the current budget. Where that cap binds, the trough dips
 * into the dead zone, but the sliver it clips is then a small fraction of a large
 * amplitude, which is the opposite of the low-impedance case this bias exists
 * for.
 */
static float bias_for(const esp_foc_motor_id_seq_t *s, float v_amp)
{
    const esp_foc_motor_id_seq_config_t *c = &s->cfg;
    const float dead = (s->result.v_deadzone_v > 0.0f) ? s->result.v_deadzone_v
                                                       : c->v_deadzone_v;
    float head = v_amp;
    const float r = s->result.rs_ohm;
    if (r > 0.0f) {
        const float head_max = c->i_probe_max_a * r * MOTOR_ID_BIAS_I_FRAC;
        if (head_max > 0.0f && head > head_max) {
            head = head_max;
        }
        /* Enough current for the reading itself, whatever the search asks for. */
        const float floor_v = c->i_probe_target_a * r;
        if (head < floor_v) {
            head = floor_v;
        }
    }
    return dead * MOTOR_ID_DEAD_MARGIN + head;
}

/** The part of |Z| that is not R, in milliohms; 0 when R already accounts for it. */
static int32_t reactance_mohm(int32_t z_mohm, int32_t r_mohm)
{
    if (z_mohm <= r_mohm || r_mohm < 0) {
        return 0;
    }
    const float z = (float)z_mohm;
    const float r = (float)r_mohm;
    return (int32_t)sqrtf(z * z - r * r);
}

/** Whether the winding answers at @p f_hz at all, asked at full drive. */
static bool probe_reaches(esp_foc_motor_id_seq_t *s, uint32_t f_hz)
{
    const esp_foc_motor_id_seq_config_t *c = &s->cfg;
    const float frac = (c->v_probe_frac_max > c->v_probe_frac)
                           ? c->v_probe_frac_max
                           : c->v_probe_frac;
    esp_foc_ident_excite_t e;
    fill_excite(s, f_hz, &e);
    float v_amp = frac * c->vdc;
    /*
     * Full drive is not a safe question to ask. |Z| is unknown here — that is why
     * the question exists — but it can never be below R, so v/R bounds the current
     * this will draw, and R is known from the coarse point. Without that bound this
     * asked with 4.2 V into a 2.8 ohm winding and tripped the bridge at 833 Hz,
     * then read its own trip as "the winding does not answer up here" and walked
     * the frequency down with the bridge already latched.
     *
     * Only part of the budget, because the bias this rides on spends the rest.
     * Claiming all of it tripped the guard at every frequency the walk tried, and
     * a refusal the probe caused itself is what left the delay unmeasured and
     * published 38 uH for a 250 uH winding.
     */
    if (s->result.coarse.r_mohm > 0 && c->i_probe_max_a > 0.0f) {
        const float v_cap = c->i_probe_max_a * MOTOR_ID_REACH_I_FRAC *
                            ((float)s->result.coarse.r_mohm / 1000.0f);
        if (v_cap < v_amp) {
            v_amp = v_cap;
        }
    }
    e.v_amp = q16_from_float(v_amp);
    e.v_bias = q16_from_float(bias_for(s, v_amp));

    esp_foc_ident_z_t z;
    if (!s->ops.probe_z(s->ops.ctx, 0, &e, &z)) {
        return false;
    }
    return z.valid && z.i_amp_ma >= MOTOR_ID_MIN_PROBE_MA;
}

/**
 * Probe at @p f_hz, searching the excitation for a usable response.
 *
 * The impedance spans two orders of magnitude across plausible machines and
 * also moves with the probe frequency, so a fixed fraction of Vdc either
 * starves the measurement or trips the current limit. Scaling toward a target
 * response is what lets one sequence cover both a 500 uH and a 40 mH winding.
 */
static bool probe_adapt(esp_foc_motor_id_seq_t *s, uint32_t f_hz,
                        esp_foc_ident_z_t *out)
{
    const esp_foc_motor_id_seq_config_t *c = &s->cfg;
    int32_t target = (int32_t)(c->i_probe_target_a * 1000.0f);
    int32_t i_max = (int32_t)(c->i_probe_max_a * 1000.0f);
    float frac = c->v_probe_frac;
    float frac_max = (c->v_probe_frac_max > c->v_probe_frac)
                         ? c->v_probe_frac_max
                         : c->v_probe_frac;
    for (int attempt = 0; attempt < MOTOR_ID_AMP_TRIES; attempt++) {
        esp_foc_ident_excite_t e;
        fill_excite(s, f_hz, &e);
        const float v_amp = frac * c->vdc;
        e.v_amp = q16_from_float(v_amp);
        e.v_bias = q16_from_float(bias_for(s, v_amp));

        const bool got = s->ops.probe_z(s->ops.ctx, 0, &e, out);

        float scale;
        if (!got) {
            /* Under the kernel's noise floor; the only lever is more drive. */
            scale = 4.0f;
        } else if (out->i_amp_ma > i_max && i_max > 0) {
            scale = (float)i_max / (float)out->i_amp_ma * 0.7f;
        } else if (out->i_amp_ma < (target * 3) / 5 ||
                   out->i_amp_ma > (target * 8) / 5) {
            /*
             * Converge near the target rather than accepting any response in
             * range: the DC bias that keeps the current one-sided scales with the
             * AC amplitude, so an amplitude twice what is needed doubles the bias
             * and spends the current budget on nothing.
             */
            scale = (float)target / (float)out->i_amp_ma;
        } else {
            s->result.probe_v_mv = (int32_t)(frac * c->vdc * 1000.0f);
            return true;
        }

        if (scale > 8.0f) {
            scale = 8.0f;
        } else if (scale < 0.125f) {
            scale = 0.125f;
        }
        float next = frac * scale;
        if (next > frac_max) {
            next = frac_max;
        }
        if (next <= frac * 1.02f && next >= frac * 0.98f) {
            /* Pinned at the ceiling and still short: nothing more to try. */
            break;
        }
        frac = next;
    }
    return false;
}

static esp_err_t run_terminal(esp_foc_motor_id_seq_t *s)
{
    esp_foc_motor_id_result_t *r = &s->result;
    q16_t v = q16_from_float(s->cfg.v_dc_probe_frac * s->cfg.vdc);

    for (int t = 0; t < 3; t++) {
        if (!s->ops.probe_terminal(s->ops.ctx, t, v, &r->terminal_ma[t])) {
            return bail(s, ESP_FAIL);
        }
        if (faulted(s)) {
            return bail(s, ESP_ERR_INVALID_STATE);
        }
    }

    /* A ratio across three near-zero readings is noise, not a verdict. */
    if (!response_sane(s, r->terminal_ma[0]) &&
        !response_sane(s, r->terminal_ma[1]) &&
        !response_sane(s, r->terminal_ma[2])) {
        return bail(s, ESP_ERR_INVALID_RESPONSE);
    }

    esp_foc_ident_sym_t sym;
    esp_foc_ident_symmetry(r->terminal_ma, s->cfg.sym_limit_permil, &sym);
    r->symmetry_spread = (float)sym.spread_permil / 1000.0f;
    if (!sym.ok) {
        /*
         * An open delta winding reads 1:2:1 across the terminals. Refusing here
         * is the whole point of the state: the machine that motivated this
         * sequence handed a plausible-looking phase map to the lock-in and then
         * diverged on the q axis, where no gain could have saved it.
         */
        return bail(s, ESP_ERR_INVALID_RESPONSE);
    }
    r->valid_mask |= ESP_FOC_MOTOR_ID_VALID_SYMMETRY;
    return ESP_OK;
}

/**
 * Time the command-to-measurement delay off a voltage step.
 *
 * The winding cannot answer before it is driven, so the first sample that moves
 * is the delay. Nothing else about the machine enters: not R, not L, and in
 * particular not whether R is the same at two frequencies.
 *
 * That last point is why this replaced a sweep. Matching R across frequencies
 * infers the delay from a premise the iron denies — this bench's eight candidates
 * came back 740, 502, 313, 158, 88 per-mil with no minimum in range, because every
 * extra sample of compensation kept closing a gap that was partly loss and not
 * delay at all. Left alone it would have rotated the vector until L went negative,
 * which it did, and cos being even means the mirror is invisible to R.
 *
 * L is what this protects. R survives a wrong delay; L does not.
 */
/**
 * The step itself: drive one, time it, and repeat until two agree.
 *
 * Returns the delay in samples, or -1 when the winding never crossed the
 * threshold or no two tries agreed.
 */
static int32_t step_lag_measure(esp_foc_motor_id_seq_t *s, int32_t r_ref_mohm)
{
    const esp_foc_motor_id_seq_config_t *c = &s->cfg;
    q16_t win[ESP_FOC_IDENT_STEP_MAX];
    int32_t seen[MOTOR_ID_STEP_TRIES];
    int n_seen = 0;

    /*
     * Drive it harder than the impedance probes do. What has to clear the noise is
     * the first sample of the rise, which is V*Ts/L and therefore scales with the
     * voltage, while the current the winding settles at only lasts for the window's
     * duty cycle. At twice the probe target that settled value is still half of the
     * guard on this bench.
     */
    const float i_step = c->i_probe_target_a * MOTOR_ID_STEP_I_RATIO;
    const float v = c->v_deadzone_v + i_step * ((float)r_ref_mohm / 1000.0f);
    const q16_t vd = q16_from_float(v);
    /* Only a floor: the kernel raises it to whatever the baseline is doing. */
    const q16_t thresh = q16_from_float(i_step * MOTOR_ID_STEP_THRESH_FRAC);

    for (int t = 0; t < MOTOR_ID_STEP_TRIES; t++) {
        if (!s->ops.probe_step(s->ops.ctx, 0, vd, MOTOR_ID_STEP_PRE, win,
                               ESP_FOC_IDENT_STEP_MAX)) {
            continue;
        }
        const int32_t lag = esp_foc_ident_step_lag(win, ESP_FOC_IDENT_STEP_MAX,
                                                   MOTOR_ID_STEP_PRE, thresh);
        /*
         * Zero is not a fast machine, it is a baseline that already moved: the
         * sample carrying the edge was read before the edge was written.
         */
        if (lag <= 0) {
            continue;
        }
        for (int k = 0; k < n_seen; k++) {
            if (seen[k] == lag) {
                return lag;
            }
        }
        seen[n_seen++] = lag;
    }
    return -1;
}

static esp_err_t run_lagcal_step(esp_foc_motor_id_seq_t *s, int32_t r_ref_mohm,
                                 uint32_t f_hi)
{
    esp_foc_motor_id_result_t *r = &s->result;

    const int32_t lag = step_lag_measure(s, r_ref_mohm);
    if (lag < 0 || lag > (int32_t)MOTOR_ID_LAG_MAX) {
        s->lag_used = s->cfg.lag_samples;
        return bail(s, ESP_ERR_INVALID_RESPONSE);
    }
    s->lag_used = (uint32_t)lag;
    r->lag_samples = (uint32_t)lag;
    /*
     * No sub-sample trim to solve. The conversion is triggered from the carrier's
     * own zero event, so the sampling instant is phase-locked to the period and the
     * delay is a whole number of them; the half-period the zero-order hold adds is
     * already taken out in the solve. The fraction the sweep used to chase was the
     * sweep's own resolution, not the bench's.
     */
    s->trim_used = 0;
    r->phase_trim_cdeg = 0;

    /*
     * Re-anchor and record how far R still moves between the two frequencies. It
     * is no longer a decision, but it is the number that says how much of the
     * winding is iron rather than copper, and a run with nothing to compare
     * against cannot say whether L is worth believing.
     */
    esp_foc_ident_z_t anchor;
    if (probe_adapt(s, s->cfg.coarse_hz, &anchor) && anchor.r_mohm > 0) {
        r_ref_mohm = anchor.r_mohm;
        r->coarse = anchor;
    }
    esp_foc_ident_z_t z;
    if (probe_adapt(s, f_hi, &z) && z.r_mohm > 0 && r_ref_mohm > 0) {
        int32_t d = z.r_mohm - r_ref_mohm;
        if (d < 0) {
            d = -d;
        }
        r->lag_r_permil[lag] = (int32_t)(((int64_t)d * 1000) / r_ref_mohm);
    } else {
        r->lag_r_permil[lag] = 0;
    }
    return ESP_OK;
}

static esp_err_t run_lagcal(esp_foc_motor_id_seq_t *s, int32_t r_ref_mohm,
                            uint32_t f_hi)
{
    esp_foc_motor_id_result_t *r = &s->result;
    int32_t best_permil = INT32_MAX;
    int32_t best_cdeg = 0;
    int best = -1;

    if (s->ops.probe_step != NULL) {
        return run_lagcal_step(s, r_ref_mohm, f_hi);
    }

    for (uint32_t lag = 0; lag <= MOTOR_ID_LAG_MAX; lag++) {
        s->lag_used = lag;
        r->lag_r_permil[lag] = -1;

        esp_foc_ident_z_t z;
        if (!probe_adapt(s, f_hi, &z) || z.r_mohm <= 0) {
            continue;
        }
        if (faulted(s)) {
            return bail(s, ESP_ERR_INVALID_STATE);
        }
        /*
         * A winding cannot lead its own voltage. Matching R alone cannot tell a
         * rotation of -2*phi from none, because cos is even, and on silicon the
         * sweep took that mirror: it settled on a delay that reported arg(Z) as
         * -29 degrees, which is a negative inductance.
         */
        if (z.l_uh <= 0) {
            continue;
        }

        int32_t d = z.r_mohm - r_ref_mohm;
        if (d < 0) {
            d = -d;
        }
        int32_t permil = (int32_t)(((int64_t)d * 1000) / r_ref_mohm);
        r->lag_r_permil[lag] = permil;
        if (permil < best_permil) {
            best_permil = permil;
            best_cdeg = z.phase_cdeg;
            best = (int)lag;
        }
    }

    int32_t limit = (s->cfg.lag_cal_limit_permil > 0)
                        ? s->cfg.lag_cal_limit_permil
                        : MOTOR_ID_LAGCAL_DEFAULT_PERMIL;
    int32_t half_permil = 0;
    /*
     * The sweep steps in whole samples, so up to half a sample of the true delay
     * is never on any candidate, and dR/R = dphi * tan(arg Z) says exactly what
     * that costs at this frequency. Holding the winner to a tighter figure than
     * its own resolution rejects perfectly good sweeps: at fs/24 with arg(Z) near
     * 54 degrees, half a sample is already 18 per cent. What the sweep leaves
     * behind is then solved as a trim, not searched.
     */
    if (best >= 0) {
        const float phi = (float)best_cdeg * 0.01f * (float)M_PI / 180.0f;
        const float t = tanf(phi);
        if (t > 0.0f && t < 20.0f) {
            const float half = (float)M_PI * (float)f_hi / (float)s->cfg.pwm_hz;
            half_permil = (int32_t)(half * t * 1500.0f);
            if (half_permil > limit) {
                limit = half_permil;
            }
        }
    }
    /*
     * The winner has to be a minimum, not just the smallest thing that answered.
     *
     * A sweep that only descends has not found a delay, it has run out of range,
     * and its last surviving candidate is indistinguishable from a trend. Silicon
     * produced exactly that once the sense noise floor rose above the probe
     * response: 523, 373, 241, 147, 100 per-mil and nothing beyond, so the edge won
     * with a figure under the limit and the fine probe was aimed from it.
     *
     * The right neighbour is allowed to be missing, because on a machine whose
     * arg(Z) is small at this frequency one extra sample of compensation always
     * rotates past zero and that candidate is refused as a negative inductance —
     * the true delay legitimately sits at the edge of what can resolve. What is
     * then required instead is that the winner be as good as the sweep's own
     * resolution allows, which is the half-sample figure and not the caller's
     * limit: a descending trend stopped by refusals cannot meet that.
     */
    if (best > 0) {
        const int32_t lo = r->lag_r_permil[best - 1];
        const int32_t hi = (best < (int)MOTOR_ID_LAG_MAX)
                               ? r->lag_r_permil[best + 1]
                               : -1;
        if (lo < 0 || lo <= best_permil) {
            best = -1;
        } else if (hi < 0 && best_permil > half_permil) {
            best = -1;
        } else if (hi >= 0 && hi <= best_permil) {
            best = -1;
        }
    } else {
        best = -1;
    }

    if (best < 0 || best_permil > limit) {
        /*
         * No delay reconciles the two frequencies, so the error is not a delay:
         * a filter in the sense path, a saturating probe, or a winding whose R
         * really is moving. Designing L from any of these is worse than
         * stopping.
         */
        s->lag_used = s->cfg.lag_samples;
        return bail(s, ESP_ERR_INVALID_RESPONSE);
    }

    s->lag_used = (uint32_t)best;
    r->lag_samples = (uint32_t)best;

    /*
     * Refresh the anchor with the delay that was just found. The coarse R used
     * above came from a probe running the caller's seed, and while that seed only
     * costs R a fraction of a per cent, the sub-sample solve below divides by
     * tan(45 degrees) and would turn that fraction into degrees of false trim.
     */
    esp_foc_ident_z_t anchor;
    if (probe_adapt(s, s->cfg.coarse_hz, &anchor) && anchor.r_mohm > 0) {
        r_ref_mohm = anchor.r_mohm;
        r->coarse = anchor;
    }

    /*
     * Whatever is left is smaller than one sample, and it does not have to be
     * searched: dR/R = dphi * tan(arg Z), so the residual angle follows from the
     * mismatch the winner still shows. Solving it beats another sweep, and without
     * it the coarse point keeps its error where L is most fragile.
     */
    esp_foc_ident_z_t z;
    if (probe_adapt(s, f_hi, &z) && z.r_mohm > 0 && z.phase_cdeg > 0) {
        const float phi = (float)z.phase_cdeg * 0.01f * (float)M_PI / 180.0f;
        const float tan_phi = tanf(phi);
        if (tan_phi > 0.1f) {
            const float dr = ((float)z.r_mohm - (float)r_ref_mohm) /
                             (float)r_ref_mohm;
            const float dphi = dr / tan_phi;
            /* A residual larger than a sample means the integer pick was wrong,
             * not that there is a fraction to trim. */
            const float span = 2.0f * (float)M_PI * (float)f_hi /
                               (float)s->cfg.pwm_hz;
            const int32_t trim =
                (int32_t)(dphi * 180.0f / (float)M_PI * 100.0f);
            const int32_t mag = (trim < 0) ? -trim : trim;
            /* Below half a degree it is the bench's own repeatability, not a
             * residual, and chasing it makes the answer worse. */
            if (dphi > -span && dphi < span && mag > 50) {
                r->phase_trim_cdeg = trim;
                s->trim_used = trim;
            }
        }
    }
    return ESP_OK;
}

static esp_err_t run_roverl(esp_foc_motor_id_seq_t *s)
{
    const esp_foc_motor_id_seq_config_t *c = &s->cfg;
    esp_foc_motor_id_result_t *r = &s->result;

    esp_foc_ident_z_t coarse;
    if (!probe_adapt(s, c->coarse_hz, &coarse)) {
        return bail(s, ESP_ERR_INVALID_RESPONSE);
    }
    r->coarse = coarse;
    if (!response_sane(s, coarse.i_amp_ma)) {
        return bail(s, ESP_ERR_INVALID_RESPONSE);
    }
    if (faulted(s)) {
        return bail(s, ESP_ERR_INVALID_STATE);
    }

    /*
     * The coarse point is where R is worth taking: arg(Z) is a few degrees there,
     * so R is almost all of |Z| and a delay error costs it a fraction of a per
     * cent. Everything downstream leans on this one number — the sweep compares
     * against it and L is derived from |Z| minus it.
     */
    int32_t r_anchor_mohm = coarse.r_mohm;
    if (r_anchor_mohm <= 0) {
        r_anchor_mohm = coarse.z_mohm;
    }

    int32_t f_max = (int32_t)(c->pwm_hz / MOTOR_ID_MIN_PROBE_DIV);
    /*
     * Do not measure the machine far above the loop that will use the answer.
     * Iron is not transparent at kilohertz: eddy currents shield the flux, so the
     * apparent L falls while R climbs with loss. Left alone, the search for
     * omega*L == R chases that trend and finds a perfectly self-consistent point
     * far up — on the reference winding it settled at 2397 Hz and reported 2.64 ohm
     * with 172 uH, against 2.08 ohm at DC and a 500 uH nameplate. Both numbers are
     * true at 2397 Hz and neither is the plant a 300 Hz current loop sees.
     */
    if (c->i_bw_hz > 0.0f) {
        const int32_t f_bw = (int32_t)(c->i_bw_hz * MOTOR_ID_PROBE_BW_RATIO);
        if (f_bw > (int32_t)c->coarse_hz && f_bw < f_max) {
            f_max = f_bw;
        }
    }
    /*
     * Aim from above, on the measured angle, and never on the coarse L.
     *
     * At the coarse point the quadrature current is what carries L, and on this
     * bench that is about 18 mA of a 176 mA response while the sense noise
     * averages down to roughly 11 mA over the window. So the coarse arg(Z) came
     * back as 1.3, 2.9 and -6.2 degrees on consecutive runs of the same winding
     * where 5.9 was true, and a negative one even implies a negative L. Aiming the
     * fine probe from that lands anywhere.
     *
     * Starting high inverts the conditioning: arg(Z) is large, the quadrature part
     * dominates, and arg alone gives the next frequency, since
     * omega*L = R*tan(arg) means f45 = f / tan(arg).
     */
    int32_t f_fine = (int32_t)(c->pwm_hz / (MOTOR_ID_MIN_PROBE_DIV * 2));
    /*
     * Under the same ceiling the search itself respects. Starting above it asks
     * the one question the run cannot afford to get wrong at the frequency where
     * it is least answerable: a delay error of two samples is 45 degrees at
     * fs/16, which rotates the demodulated vector past the real axis, and a
     * negative in-phase component is refused outright — so the aim would burn its
     * whole amplitude search before ever reaching a frequency it could read.
     */
    if (f_fine > f_max) {
        f_fine = f_max;
    }

    go(s, ESP_FOC_MOTOR_ID_LAGCAL);
    /*
     * The sweep runs below the fine point, not at it. The L that aimed f_fine came
     * from the coarse probe, which was itself measured with the unknown delay
     * still in place, so f_fine can be far too high before the sweep has had a
     * chance to correct anything. Capping the sweep keeps one sample worth a small
     * angle, and after the delay is known the fine probe re-aims from a coarse
     * measurement that is finally trustworthy.
     */
    /*
     * Walk the sweep point down until the winding answers it. A fixed fraction of
     * the carrier needs no L to choose, which is the whole point, but a very
     * inductive machine is nearly an open circuit up there and would starve the
     * probe. Usability is asked at full drive, not through the adaptive search:
     * that search insists on landing near the target current, which is a bias
     * budget question and not the question here.
     */
    int32_t f_lag = (int32_t)(c->pwm_hz / MOTOR_ID_LAGCAL_MAX_DIV);
    bool lag_point = false;
    const int32_t f_lag_min = (int32_t)(c->coarse_hz * MOTOR_ID_LAGCAL_MIN_RATIO);
    if (!c->skip_lag_cal) {
        while (f_lag >= f_lag_min) {
            if (probe_reaches(s, (uint32_t)f_lag)) {
                lag_point = true;
                break;
            }
            if (faulted(s)) {
                return bail(s, ESP_ERR_INVALID_STATE);
            }
            f_lag /= 2;
        }
    } else {
        f_fine = (int32_t)c->coarse_hz;
    }
    /*
     * These two gates belong to the sweep alone.
     *
     * It compares R at two frequencies against the coarse one, so it needs them
     * separated and it needs the coarse point to be R-dominated: on a 40 mH winding
     * at 100 Hz arg(Z) is already 86 degrees, one sample of delay moves the anchor
     * by 40 per cent, and nothing can be reconciled against that.
     *
     * A step needs neither, and gating it on the coarse angle was circular — the
     * angle is wrong until the delay is known. On silicon the coarse point came back
     * at -0.30 degrees for exactly that reason, the gate then skipped the whole
     * calibration, and L was published as 61 uH against a 250 uH winding from a
     * delay that had never been measured.
     */
    const bool stepping = (s->ops.probe_step != NULL);
    const bool anchor_ok = coarse.phase_cdeg > 0 && coarse.phase_cdeg < 3000;
    if (!lag_point || (!anchor_ok && !stepping)) {
        f_fine = (int32_t)c->coarse_hz;
    }
    bool lag_ok = false;
    if (!c->skip_lag_cal && lag_point && (anchor_ok || stepping) &&
        coarse.r_mohm > 0) {
        esp_err_t err = run_lagcal(s, coarse.r_mohm, (uint32_t)f_lag);
        if (err != ESP_OK) {
            return err;
        }
        /* The sweep point is the first frequency in the run whose angle was taken
         * with the right delay, so it is where the aim starts — still under the
         * ceiling, since it is chosen from the carrier and not from the loop. */
        f_fine = (f_lag > f_max) ? f_max : f_lag;
        if (r->coarse.r_mohm > 0) {
            r_anchor_mohm = r->coarse.r_mohm;
        }
        lag_ok = true;
    }

    go(s, ESP_FOC_MOTOR_ID_ROVERL_FINE);
    esp_foc_ident_z_t fine;
    for (int pass = 0;; pass++) {
        if (!probe_adapt(s, (uint32_t)f_fine, &fine)) {
            return bail(s, ESP_ERR_INVALID_RESPONSE);
        }
        if (!response_sane(s, fine.i_amp_ma)) {
            return bail(s, ESP_ERR_INVALID_RESPONSE);
        }
        if (faulted(s)) {
            return bail(s, ESP_ERR_INVALID_STATE);
        }
        if (pass + 1 >= MOTOR_ID_FINE_PASSES) {
            break;
        }

        /*
         * Steer on |Z| against the known R, not on the angle. At the point worth
         * measuring omega*L equals R, so |Z| is R*sqrt(2); the ratio of where we
         * are to where we want to be is R/(omega*L), and omega*L comes out of the
         * magnitudes alone. The angle is not trustworthy here — see
         * esp_foc_ident_l_from_mag_uh.
         */
        const int32_t xl = reactance_mohm(fine.z_mohm, r_anchor_mohm);
        /* Within this much of the crossing the point is good enough to design
         * from, and another window only spends time. */
        if (xl > (r_anchor_mohm * 3) / 4 && xl < (r_anchor_mohm * 4) / 3) {
            break;
        }
        if (xl < r_anchor_mohm / 40) {
            if (c->skip_lag_cal) {
                break;
            }
            /* The reactance is still buried under R, so the ratio it implies is
             * not a step but a leap; the search cannot be steered from here. */
            f_fine = f_max;
            continue;
        }
        int32_t f_next = (int32_t)(((int64_t)f_fine * r_anchor_mohm) / xl);
        if (f_next < (int32_t)c->coarse_hz) {
            f_next = (int32_t)c->coarse_hz;
        }
        if (f_next > f_max) {
            f_next = f_max;
        }
        if (f_next == f_fine) {
            break;
        }
        f_fine = f_next;
    }

    /*
     * R comes from the coarse point. L has two independent estimates, and which
     * one is usable depends on the angle.
     *
     * From the magnitude, dL/L is the error in R amplified by (R/XL)^2. From the
     * angle it is dphi/tan(arg Z). They cross at 45 degrees; below it the
     * magnitude degrades quadratically while the angle degrades linearly, and the
     * carrier will not let arg Z reach 45 on a machine whose R/L sits above the
     * band the current loop is allowed to use.
     *
     * The bench measured exactly that. Sweeping this winding from 200 Hz to 1 kHz,
     * where arg Z ran 5 to 29 degrees, the angle gave 160 to 380 uH around a
     * plate value of 250, while the magnitude gave 0 to 958 uH from the same
     * windows. So the angle is what is taken whenever the delay was measured —
     * which is the entire reason that state exists — and the magnitude is the
     * fallback for the runs that skipped it.
     *
     * The order matters, and getting it wrong cost three silicon runs. Computing
     * the magnitude first and refusing on it discards runs whose angle was
     * perfectly good: |Z| at the fine point is measured at a different frequency
     * than the anchor R, and when the two land within noise of each other the
     * magnitude estimate is undefined while the angle still reads 45 degrees.
     * The fallback failing is not grounds to reject the primary.
     */
    int32_t l_uh = (lag_ok && fine.l_uh > 0) ? fine.l_uh : 0;
    if (l_uh <= 0) {
        l_uh = esp_foc_ident_l_from_mag_uh(fine.z_mohm, r_anchor_mohm,
                                           fine.f_hz);
    }
    if (l_uh <= 0 && r->coarse.l_uh > 0) {
        l_uh = r->coarse.l_uh;
    }
    if (l_uh <= 0) {
        return bail(s, ESP_ERR_INVALID_RESPONSE);
    }

    r->r_loop_ohm = (float)r_anchor_mohm / 1000.0f;
    r->ls_h = (float)l_uh / 1000000.0f;
    r->probe_hz = (float)fine.f_hz;
    r->probe_phase_cdeg = fine.phase_cdeg;
    r->probe_i_ma = fine.i_amp_ma;
    r->roverl_rad_s = (float)r_anchor_mohm * 1000.0f / (float)l_uh;
    r->valid_mask |= ESP_FOC_MOTOR_ID_VALID_R_LOOP | ESP_FOC_MOTOR_ID_VALID_LS;
    return ESP_OK;
}

static esp_err_t run_rs(esp_foc_motor_id_seq_t *s)
{
    const esp_foc_motor_id_seq_config_t *c = &s->cfg;
    esp_foc_motor_id_result_t *r = &s->result;

    /*
     * Size the operating points from what the terminal state already drew rather
     * than a fixed fraction of Vdc, with headroom for the bridge offset the slope
     * is there to remove. Both points must fit under the modulation ceiling, so
     * the first one gets at most half of it.
     */
    float v_ceiling = c->v_probe_frac_max * c->vdc * 0.5f;
    float v1 = c->v_dc_probe_frac * c->vdc;
    if (r->terminal_ma[0] > MOTOR_ID_MIN_PROBE_MA) {
        /* The terminal drive saw 1.5 R because the other two legs are in
         * parallel; the d axis sees R alone, so aim the same current. */
        const float z_term = c->v_dc_probe_frac * c->vdc /
                             ((float)r->terminal_ma[0] / 1000.0f);
        v1 = c->i_probe_target_a * (z_term / 1.5f) * 1.5f;
    }
    int32_t i1 = 0;
    int32_t i2 = 0;

    const int32_t i_max = (int32_t)(c->i_probe_max_a * 1000.0f);
    for (int attempt = 0; attempt < MOTOR_ID_AMP_TRIES; attempt++) {
        if (v1 > v_ceiling) {
            v1 = v_ceiling;
        }
        if (!s->ops.probe_dc(s->ops.ctx, 0, q16_from_float(v1), &i1)) {
            return bail(s, ESP_FAIL);
        }
        if (!s->ops.probe_dc(s->ops.ctx, 0, q16_from_float(v1 * 2.0f), &i2)) {
            return bail(s, ESP_FAIL);
        }
        if (faulted(s)) {
            return bail(s, ESP_ERR_INVALID_STATE);
        }
        /* The second point is the one at risk, since it is twice the first. */
        if (i_max > 0 && i2 > i_max) {
            v1 *= ((float)i_max / (float)i2) * 0.7f;
            continue;
        }
        if (i2 > MOTOR_ID_MIN_PROBE_MA || v1 >= v_ceiling) {
            break;
        }
        v1 *= 3.0f;
    }
    if (!response_sane(s, i2)) {
        /*
         * No pair of DC points fits between the bridge offset and the current
         * ceiling, which happens on a winding low enough that the offset alone
         * would exceed the limit. That says nothing is wrong with the machine, so
         * report no Rs and let the AC probes carry the run on the configured dead
         * zone instead of refusing everything.
         */
        return ESP_OK;
    }

    /*
     * The slope divides by (i2 - i1), so two points that landed on top of each
     * other produce an arbitrarily large resistance from nothing but sense noise.
     * On this bench that published 40.9 ohm for a 1.9 ohm winding, and the dead
     * zone derived from the same line then poisoned the AC bias. Refuse instead.
     */
    const int32_t di = (i2 > i1) ? (i2 - i1) : (i1 - i2);
    if (di < MOTOR_ID_MIN_PROBE_MA) {
        return ESP_OK;
    }

    int32_t rs = esp_foc_ident_r_slope_mohm((int32_t)(v1 * 1000.0f), i1,
                                            (int32_t)(v1 * 2000.0f), i2);
    if (rs <= 0) {
        return ESP_OK;
    }
    r->rs_ohm = (float)rs / 1000.0f;
    r->valid_mask |= ESP_FOC_MOTOR_ID_VALID_RS;

    /*
     * Where the two-point line crosses zero current is the voltage the bridge
     * eats before the winding sees anything, which is the dead zone the AC probes
     * have to be biased past. Measuring it beats configuring it: it moves with
     * deadtime, device drops and temperature.
     */
    const float v_dead = v1 - ((float)i1 / 1000.0f) * r->rs_ohm;
    r->v_deadzone_v = (v_dead > 0.0f) ? v_dead : 0.0f;
    return ESP_OK;
}

static esp_err_t ramp_fe(esp_foc_motor_id_seq_t *s, int from_hz, int to_hz,
                         uint32_t ms)
{
    uint32_t dt = (s->cfg.dt_ms > 0u) ? s->cfg.dt_ms : 10u;
    uint32_t steps = (ms + dt - 1u) / dt;
    if (steps == 0u) {
        steps = 1u;
    }
    for (uint32_t k = 1u; k <= steps; k++) {
        int fe = from_hz + (int)(((int64_t)(to_hz - from_hz) * (int)k) / (int)steps);
        s->ops.set_fe_hz(s->ops.ctx, (q16_t)((int64_t)fe * 65536));
        nap(s, dt);
        if (faulted(s)) {
            return bail(s, ESP_ERR_INVALID_STATE);
        }
    }
    return ESP_OK;
}

static esp_err_t run_flux(esp_foc_motor_id_seq_t *s)
{
    const esp_foc_motor_id_seq_config_t *c = &s->cfg;
    esp_foc_motor_id_result_t *r = &s->result;

    s->ops.set_idq(s->ops.ctx, q16_from_float(c->i_flux_a), 0);
    esp_err_t err = ramp_fe(s, 0, c->flux_hz, c->ramp_ms);
    if (err != ESP_OK) {
        return err;
    }

    go(s, ESP_FOC_MOTOR_ID_RATED_FLUX);
    nap(s, c->flux_hold_ms);
    if (faulted(s)) {
        return bail(s, ESP_ERR_INVALID_STATE);
    }

    q16_t vd = 0;
    q16_t vq = 0;
    q16_t id = 0;
    q16_t iq = 0;
    s->ops.fetch_dq(s->ops.ctx, &vd, &vq, &id, &iq);

    int32_t w = (int32_t)((MOTOR_ID_TWO_PI_Q16 * c->flux_hz) >> 16);
    if (s->ops.fetch_rotor != NULL) {
        q16_t th_m = 0;
        q16_t w_m = 0;
        if (!s->ops.fetch_rotor(s->ops.ctx, &th_m, &w_m)) {
            return bail(s, ESP_ERR_INVALID_STATE);
        }
        float wm = q16_to_float(w_m);
        if (wm < 0.0f) {
            wm = -wm;
        }
        const float fm = wm / 6.283185f;
        if (fm < 1.0f) {
            return bail(s, ESP_ERR_INVALID_RESPONSE);
        }
        const float ppr = (float)c->flux_hz / fm;
        int pp = (int)(ppr + 0.5f);
        if (pp < 1 || pp > 40) {
            return bail(s, ESP_ERR_INVALID_RESPONSE);
        }
        float rem = ppr - (float)pp;
        if (rem < 0.0f) {
            rem = -rem;
        }
        if (rem > 0.35f) {
            return bail(s, ESP_ERR_INVALID_RESPONSE);
        }
        r->pole_pairs = pp;
        r->valid_mask |= ESP_FOC_MOTOR_ID_VALID_PP;
        r->flux_fm_hz = fm;
        r->flux_we_rad_s = wm * (float)pp;
        w = (int32_t)(r->flux_we_rad_s + 0.5f);
        if (w <= 0) {
            return bail(s, ESP_ERR_INVALID_RESPONSE);
        }
    }
    int32_t psi = esp_foc_ident_psi_uwb(q16_to_milli(vq),
                                        q16_to_milli(iq),
                                        q16_to_milli(id),
                                        (int32_t)(r->r_loop_ohm * 1000.0f),
                                        (int32_t)(r->ls_h * 1000000.0f),
                                        w);

    go(s, ESP_FOC_MOTOR_ID_RAMPDOWN);
    err = ramp_fe(s, c->flux_hz, 0, c->ramp_ms / 2u);
    s->ops.set_idq(s->ops.ctx, 0, 0);
    if (err != ESP_OK) {
        return err;
    }

    if (psi > 0) {
        r->psi_f_wb = (float)psi / 1000000.0f;
        r->valid_mask |= ESP_FOC_MOTOR_ID_VALID_PSI_F;
    }
    return ESP_OK;
}

esp_err_t esp_foc_motor_id_seq_run(esp_foc_motor_id_seq_t *s)
{
    if (s == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(&s->result, 0, sizeof(s->result));
    s->phase = ESP_FOC_MOTOR_ID_IDLE;
    s->result.failed_at = ESP_FOC_MOTOR_ID_IDLE;

    esp_err_t err = check_args(s);
    if (err != ESP_OK) {
        return err;
    }
    s->result.pole_pairs = s->cfg.pole_pairs;
    s->lag_used = s->cfg.lag_samples;
    s->trim_used = 0;
    s->result.lag_samples = s->cfg.lag_samples;
    for (int i = 0; i < ESP_FOC_IDENT_LAG_MAX; i++) {
        s->result.lag_r_permil[i] = -1;
    }

    go(s, ESP_FOC_MOTOR_ID_BIAS);
    outputs_idle(s);
    s->ops.set_theta(s->ops.ctx, 0);
    nap(s, s->cfg.dt_ms);
    if (faulted(s)) {
        return bail(s, ESP_ERR_INVALID_STATE);
    }

    if (!s->cfg.skip_terminal) {
        go(s, ESP_FOC_MOTOR_ID_TERMINAL);
        err = run_terminal(s);
        if (err != ESP_OK) {
            return err;
        }
    }

    /*
     * Rs before the AC probes, not after. The two-point DC slope is the only
     * measurement here that does not depend on the bridge being linear, because
     * subtracting two operating points cancels the offset instead of modelling
     * it. Its slope sizes the DC bias the AC probes need to stay out of the dead
     * zone, and its intercept is that dead zone.
     */
    if (!s->cfg.skip_rs) {
        go(s, ESP_FOC_MOTOR_ID_RS);
        err = run_rs(s);
        if (err != ESP_OK) {
            return err;
        }
        /* A DC vector digs a well. On a free shaft the ring is BEMF into the
         * next AC window; wait it out when a rotor sensor is what made Rs safe
         * to run in the first place. */
        if (s->ops.fetch_rotor != NULL) {
            nap(s, 400);
        }
    }

    if (s->cfg.known_r_loop_ohm > 0.0f && s->cfg.known_ls_h > 0.0f) {
        s->result.r_loop_ohm = s->cfg.known_r_loop_ohm;
        s->result.ls_h = s->cfg.known_ls_h;
        s->result.roverl_rad_s = s->cfg.known_r_loop_ohm / s->cfg.known_ls_h;
        s->result.valid_mask |= ESP_FOC_MOTOR_ID_VALID_R_LOOP |
                                ESP_FOC_MOTOR_ID_VALID_LS;
    } else {
        go(s, ESP_FOC_MOTOR_ID_ROVERL_COARSE);
        err = run_roverl(s);
        if (err != ESP_OK) {
            return err;
        }
    }

    go(s, ESP_FOC_MOTOR_ID_TUNE_I);
    err = esp_foc_motor_id_seq_gains_for(&s->cfg, s->result.r_loop_ohm,
                                         s->result.ls_h, &s->result.kp,
                                         &s->result.ki);
    if (err != ESP_OK) {
        return bail(s, err);
    }
    s->ops.apply_gains(s->ops.ctx, s->result.kp, s->result.ki);
    s->result.valid_mask |= ESP_FOC_MOTOR_ID_VALID_GAINS;

    if (s->cfg.do_flux) {
        go(s, ESP_FOC_MOTOR_ID_RAMPUP);
        err = run_flux(s);
        if (err != ESP_OK) {
            return err;
        }
    }

    outputs_idle(s);
    go(s, ESP_FOC_MOTOR_ID_DONE);
    return ESP_OK;
}
