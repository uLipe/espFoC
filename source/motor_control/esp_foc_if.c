/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Lock-in spin: align the rotor at ω=0, then slew ω at constant accel, Iq=0.
 * The align and the ramp do not overlap — see the header for what overlapping
 * them cost on silicon.
 */
#include "espFoC/motor_control/esp_foc_if.h"

#include <stddef.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define ESP_FOC_IF_TWO_PI (2.0f * (float)M_PI)

static void apply(esp_foc_if_t *s)
{
    s->ops.set_idq(s->ops.ctx, s->id, s->iq);
    s->ops.set_fe_hz(s->ops.ctx, s->fe_hz);
}

static bool ops_ok(const esp_foc_if_ops_t *o)
{
    return o != NULL && o->set_idq != NULL && o->set_fe_hz != NULL &&
           o->sleep_ms != NULL && o->get_dang != NULL && o->lock_ok != NULL &&
           o->do_handoff != NULL;
}

static bool cfg_ok(const esp_foc_if_config_t *c)
{
    if (c == NULL) {
        return false;
    }
    if (c->i_align_a <= 0.0f || c->f_max_hz <= 0) {
        return false;
    }
    if (!(c->accel_rads2 > 0.0f) && (c->creep_ramp_ms == 0u)) {
        return false;
    }
    return true;
}

static float clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

esp_err_t esp_foc_if_run(esp_foc_if_t *s, int sign)
{
    if (s == NULL || !ops_ok(&s->ops) || !cfg_ok(&s->cfg)) {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_foc_if_config_t *c = &s->cfg;
    const uint32_t dt = (c->dt_ms == 0u) ? 20u : c->dt_ms;
    const float dts = (float)dt * 0.001f;
    const float i_tgt = c->i_align_a;
    const float w_tgt = ESP_FOC_IF_TWO_PI * (float)c->f_max_hz;
    float accel = c->accel_rads2;
    if (!(accel > 0.0f)) {
        accel = w_tgt / ((float)c->creep_ramp_ms * 0.001f);
    }
    const float id_rate =
        (c->align_ms == 0u) ? i_tgt / dts : i_tgt / ((float)c->align_ms * 0.001f);
    const float sgn = (sign < 0) ? -1.0f : 1.0f;

    /* A settle gate with no deadline would sit here with i_align_a in the
     * winding for as long as the rotor keeps twitching. Make the caller say when
     * to give up. */
    if ((s->ops.at_rest != NULL) && (c->align_timeout_ms == 0u)) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Asking for the V/f stage without the op that drives it, or with a crossover
     * at or above the plateau, would silently skip it or skip the current-mode
     * ramp. Both are worth failing the build of the sequence over. */
    const bool vf_on = (c->vf_hz > 0.0f);
    if (vf_on && ((s->ops.set_vdq == NULL) || (c->vf_ramp_ms == 0u) ||
                  (c->vf_hz >= (float)c->f_max_hz))) {
        return ESP_ERR_INVALID_ARG;
    }

    float id = 0.0f;
    float w_abs = 0.0f;
    uint32_t wait_ms = 0;
    uint32_t align_ms = 0;
    uint32_t quiet_ms = 0;
    bool aligned = false;
    bool lock_to_notified = false;
    bool plateau_notified = false;

    s->phase = ESP_FOC_IF_PHASE_ALIGN;
    s->id = 0;
    s->iq = 0;
    s->fe_hz = 0;
    apply(s);

    /*
     * Stage 1: pull the rotor onto θ_ol with the field standing still. Nothing
     * here knows where the rotor started, so the only way the ramp can begin
     * from a known load angle is to spend the time putting it there.
     */
    while (!aligned) {
        if (s->ops.poll != NULL) {
            s->ops.poll(s->ops.ctx);
        }

        id = clampf(id + id_rate * dts, 0.0f, i_tgt);
        s->id = q16_from_float(id);
        s->iq = 0;
        s->fe_hz = 0;

        if (id < i_tgt) {
            /* The slew itself is bounded by align_ms, so the deadline below does
             * not apply to it — cutting it short would start the ramp on a
             * current that never reached the value the pull-out torque assumes. */
            s->phase = ESP_FOC_IF_PHASE_ALIGN;
            quiet_ms = 0;
        } else {
            s->phase = ESP_FOC_IF_PHASE_ALIGN_SETTLE;
            /* Test the quiet already elapsed, then credit this tick: crediting
             * first would count a dt that has not been slept yet and make an
             * align_settle_ms of N deliver N-dt of actual standstill. */
            if ((s->ops.at_rest == NULL) || s->ops.at_rest(s->ops.ctx)) {
                if (quiet_ms >= c->align_settle_ms) {
                    aligned = true;
                }
                quiet_ms += dt;
            } else {
                quiet_ms = 0;
            }
            align_ms += dt;
            if ((c->align_timeout_ms != 0u) &&
                (align_ms >= c->align_timeout_ms)) {
                aligned = true;
            }
        }
        apply(s);

        if (!aligned) {
            s->ops.sleep_ms(s->ops.ctx, dt);
        }
    }

    /*
     * The align window is closed, by settle or by deadline, and those two are not
     * the same outcome. Everything downstream — the break-away, the pull-in, the
     * load angle the handoff gate judges — assumes the rotor is on the commanded
     * d axis. Give the caller, who owns the only measurement, one veto before
     * anything rotates: failing here costs an align, whereas ramping a rotor that
     * was never captured costs the launch.
     */
    if ((s->ops.align_ok != NULL) && !s->ops.align_ok(s->ops.ctx)) {
        s->phase = ESP_FOC_IF_PHASE_FAIL;
        s->id = 0;
        s->iq = 0;
        s->fe_hz = 0;
        apply(s);
        return ESP_FAIL;
    }

    /*
     * The align just parked the rotor on the commanded d axis, where Id makes no
     * torque at all. Step the frame ahead before the ramp so the first tick has
     * Id·sin(lead_rad) pulling the rotor instead of waiting for the field to
     * outrun it.
     */
    if ((s->ops.advance_theta != NULL) && (c->lead_rad != 0.0f)) {
        s->ops.advance_theta(s->ops.ctx, q16_from_float(sgn * c->lead_rad));
    }

    /*
     * Stage 1b: break away on a voltage command, not a current one.
     *
     * The current loop is at its worst exactly here. At standstill there is no
     * BEMF for it to work against, the shunt sampling is at its noisiest, and the
     * bridge has a dead zone — measured, not assumed, ~0.6 V on this bench — where
     * the small-signal gain from duty to current is zero. An integrator with no
     * gain to push on winds up, breaks through, and overshoots: a supply current
     * spike on a rotor that never moved.
     *
     * Open loop the current is (v − e − v_dead)/Z, which is bounded and monotonic
     * with no state to wind up. The cost is no current limit, which is why this
     * stage stops at vf_hz: down here the BEMF term is small, so v stays near the
     * boost and a rotor that refuses to move draws boost/R and nothing worse.
     * Above that the voltage a following rotor needs would be an overcurrent on a
     * stalled one, so the current loop takes over — by then it has real BEMF and
     * real current to measure.
     */
    const float vf_w = ESP_FOC_IF_TWO_PI * c->vf_hz;
    if (vf_on) {
        const float vf_rate = vf_w / ((float)c->vf_ramp_ms * 0.001f);

        s->phase = ESP_FOC_IF_PHASE_VF_BREAKAWAY;
        while (w_abs < vf_w) {
            if (s->ops.poll != NULL) {
                s->ops.poll(s->ops.ctx);
            }

            w_abs = clampf(w_abs + vf_rate * dts, 0.0f, vf_w);
            const float fe = w_abs / ESP_FOC_IF_TWO_PI;
            const float v = c->vf_boost + c->vf_per_hz * fe;

            /*
             * On q, and that is the whole lead. The align left the rotor on the d
             * axis, so a q voltage stands a quarter turn ahead of it and makes
             * torque from the first tick — which is why lead_rad has nothing to do
             * while this stage is on.
             */
            s->ops.set_vdq(s->ops.ctx, 0, q16_from_float(sgn * v));
            s->id = q16_from_float(i_tgt);
            s->iq = 0;
            s->fe_hz = q16_from_float(sgn * fe);
            apply(s);

            s->ops.sleep_ms(s->ops.ctx, dt);
        }
        /* Hand the loop back its integrator before it is asked to hold anything. */
        if (s->ops.to_current_mode != NULL) {
            s->ops.to_current_mode(s->ops.ctx);
        }
    }

    /* Stage 2: drag it up to the plateau, on current now. */
    for (;;) {
        if (s->ops.poll != NULL) {
            s->ops.poll(s->ops.ctx);
        }
        if ((s->ops.ramp_ok != NULL) && !s->ops.ramp_ok(s->ops.ctx)) {
            s->phase = ESP_FOC_IF_PHASE_FAIL;
            return ESP_FAIL;
        }

        w_abs = clampf(w_abs + accel * dts, 0.0f, w_tgt);
        s->id = q16_from_float(i_tgt);
        s->iq = 0;
        s->fe_hz = q16_from_float(sgn * w_abs / ESP_FOC_IF_TWO_PI);
        s->phase = ESP_FOC_IF_PHASE_CREEP;
        apply(s);

        if ((w_abs >= w_tgt) && !plateau_notified) {
            s->phase = ESP_FOC_IF_PHASE_CREEP;
            plateau_notified = true;
            if (s->ops.on_plateau != NULL) {
                s->ops.on_plateau(s->ops.ctx);
            }
        }
        if (plateau_notified && s->ops.lock_ok(s->ops.ctx)) {
            s->phase = ESP_FOC_IF_PHASE_HANDOFF;
            if (s->ops.poll != NULL) {
                s->ops.poll(s->ops.ctx);
            }
            break;
        }
        if (plateau_notified) {
            wait_ms += dt;
            if (!lock_to_notified && (wait_ms >= c->lock_timeout_ms)) {
                lock_to_notified = true;
                if (s->ops.lock_timeout != NULL) {
                    s->ops.lock_timeout(s->ops.ctx);
                }
            }
        }

        s->ops.sleep_ms(s->ops.ctx, dt);
    }

    if (!s->ops.do_handoff(s->ops.ctx)) {
        s->phase = ESP_FOC_IF_PHASE_FAIL;
        return ESP_FAIL;
    }
    s->phase = ESP_FOC_IF_PHASE_DONE;
    return ESP_OK;
}
