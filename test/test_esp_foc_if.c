/*
 * Unit tests for lock-in spin (align at ω=0, then ω slew, then handoff).
 */
#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "esp_err.h"
#include "espFoC/motor_control/esp_foc_if.h"
#include "espFoC/utils/esp_foc_q16.h"

#define HIST_N 256

typedef struct {
    q16_t dang;
    bool lock_force;
    bool lock_on_plateau;
    bool handoff_ok;
    int handoff_calls;
    int lock_to_calls;
    int plateau_calls;
    int plateau_fe;
    uint32_t plateau_ms;
    uint32_t lock_after_ms;
    /* at_rest answers no until this instant, so a test can make the rotor take a
     * known time to settle. 0 = at rest from the first ask. rest_seq overrides
     * it when non-empty, for scripting a rotor that goes quiet and twitches
     * again — the last entry repeats. */
    uint32_t rest_after_ms;
    bool rest_seq[16];
    int rest_n;
    int rest_idx;
    int rest_calls;
    bool align_verdict;
    int align_ok_calls;
    /* ramp_ok answers yes for this many calls, then no. -1 = always yes. */
    int ramp_ok_until;
    int ramp_ok_calls;
    uint32_t ramp_start_ms;
    int lead_calls;
    q16_t lead_arg;
    uint32_t lead_ms;
    q16_t lead_fe_at_call;
    int vdq_calls;
    q16_t vdq_vd;
    q16_t vdq_vq;
    q16_t vdq_vq_first;
    q16_t vdq_fe_max;
    int to_current_calls;
    uint32_t to_current_ms;
    q16_t fe_at_to_current;
    int vdq_after_current;
    q16_t fe;
    q16_t id;
    q16_t iq;
    uint32_t t_ms;
    int n;
    q16_t fe_hist[HIST_N];
    q16_t id_hist[HIST_N];
    q16_t iq_hist[HIST_N];
    uint32_t t_hist[HIST_N];
    esp_foc_if_phase_t phase_hist[HIST_N];
    esp_foc_if_t *ifs;
} fake_t;

static void rec(fake_t *f)
{
    if (f->n >= HIST_N) {
        return;
    }
    int i = f->n++;
    f->fe_hist[i] = f->fe;
    f->id_hist[i] = f->id;
    f->iq_hist[i] = f->iq;
    f->t_hist[i] = f->t_ms;
    f->phase_hist[i] =
        (f->ifs != NULL) ? f->ifs->phase : ESP_FOC_IF_PHASE_ALIGN;
}

static void set_vdq(void *ctx, q16_t vd, q16_t vq)
{
    fake_t *f = (fake_t *)ctx;
    if (f->vdq_calls == 0) {
        f->vdq_vq_first = vq;
    }
    f->vdq_calls++;
    f->vdq_vd = vd;
    f->vdq_vq = vq;
    if (f->to_current_calls > 0) {
        f->vdq_after_current++;
    }
}

static void to_current_mode(void *ctx)
{
    fake_t *f = (fake_t *)ctx;
    f->to_current_calls++;
    f->to_current_ms = f->t_ms;
    f->fe_at_to_current = f->fe;
}

static void advance_theta(void *ctx, q16_t d_rad)
{
    fake_t *f = (fake_t *)ctx;
    f->lead_calls++;
    f->lead_arg = d_rad;
    f->lead_ms = f->t_ms;
    f->lead_fe_at_call = f->fe;
}

static bool align_ok(void *ctx)
{
    fake_t *f = (fake_t *)ctx;
    f->align_ok_calls++;
    return f->align_verdict;
}

static bool ramp_ok(void *ctx)
{
    fake_t *f = (fake_t *)ctx;
    f->ramp_ok_calls++;
    if (f->ramp_ok_until < 0) {
        return true;
    }
    return f->ramp_ok_calls <= f->ramp_ok_until;
}

static bool at_rest(void *ctx)
{
    fake_t *f = (fake_t *)ctx;
    f->rest_calls++;
    if (f->rest_n > 0) {
        const int i =
            (f->rest_idx < f->rest_n) ? f->rest_idx : (f->rest_n - 1);
        f->rest_idx++;
        return f->rest_seq[i];
    }
    return f->t_ms >= f->rest_after_ms;
}

static void set_idq(void *ctx, q16_t id, q16_t iq)
{
    fake_t *f = (fake_t *)ctx;
    f->id = id;
    f->iq = iq;
}

static void set_fe(void *ctx, q16_t fe)
{
    fake_t *f = (fake_t *)ctx;
    if ((fe != 0) && (f->ramp_start_ms == 0u)) {
        f->ramp_start_ms = f->t_ms;
    }
    const q16_t mag = (fe < 0) ? -fe : fe;
    if ((f->to_current_calls == 0) && (mag > f->vdq_fe_max)) {
        f->vdq_fe_max = mag;
    }
    f->fe = fe;
}

static int fe_int(q16_t fe)
{
    int32_t v = (int32_t)fe;
    if (v >= 0) {
        return (int)((v + (Q16_ONE / 2)) / Q16_ONE);
    }
    return (int)((v - (Q16_ONE / 2)) / Q16_ONE);
}

static void sleep_ms(void *ctx, uint32_t ms)
{
    fake_t *f = (fake_t *)ctx;
    f->t_ms += ms;
    rec(f);
}

static void poll(void *ctx)
{
    fake_t *f = (fake_t *)ctx;
    if (f->lock_after_ms > 0u && f->t_ms >= f->lock_after_ms) {
        f->lock_force = true;
    }
}

static void on_plateau(void *ctx)
{
    fake_t *f = (fake_t *)ctx;
    f->plateau_calls++;
    if (f->plateau_calls == 1) {
        int a = fe_int(f->fe);
        f->plateau_fe = (a < 0) ? -a : a;
        f->plateau_ms = f->t_ms;
    }
}

static q16_t get_dang(void *ctx)
{
    return ((fake_t *)ctx)->dang;
}

static bool lock_ok(void *ctx)
{
    fake_t *f = (fake_t *)ctx;
    if (f->lock_force) {
        return true;
    }
    if (f->lock_on_plateau && f->plateau_calls > 0) {
        return true;
    }
    return false;
}

static bool do_handoff(void *ctx)
{
    fake_t *f = (fake_t *)ctx;
    f->handoff_calls++;
    return f->handoff_ok;
}

static void on_lock_timeout(void *ctx)
{
    fake_t *f = (fake_t *)ctx;
    f->lock_to_calls++;
}

static esp_foc_if_config_t base_cfg(void)
{
    esp_foc_if_config_t c;
    memset(&c, 0, sizeof(c));
    c.i_align_a = 0.30f;
    c.align_ms = 40;
    c.f_max_hz = 50;
    c.creep_ramp_ms = 200;
    c.lock_timeout_ms = 200;
    c.dt_ms = 20;
    return c;
}

static void bind(esp_foc_if_t *s, fake_t *f, const esp_foc_if_config_t *c)
{
    memset(f, 0, sizeof(*f));
    f->ifs = s;
    f->handoff_ok = true;
    f->ramp_ok_until = -1;
    s->cfg = *c;
    s->ops.ctx = f;
    s->ops.set_idq = set_idq;
    s->ops.set_fe_hz = set_fe;
    s->ops.sleep_ms = sleep_ms;
    s->ops.poll = poll;
    s->ops.get_dang = get_dang;
    s->ops.lock_ok = lock_ok;
    s->ops.do_handoff = do_handoff;
    s->ops.lock_timeout = on_lock_timeout;
    s->ops.on_plateau = NULL;
    s->ops.at_rest = NULL;
    s->ops.align_ok = NULL;
    s->ops.ramp_ok = NULL;
    s->ops.advance_theta = NULL;
    s->ops.set_vdq = NULL;
    s->ops.to_current_mode = NULL;
}

TEST_CASE("if run rejects bad args", "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_if_run(NULL, 1));
    esp_foc_if_config_t c = base_cfg();
    bind(&s, &f, &c);
    s.ops.do_handoff = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_if_run(&s, 1));
}

TEST_CASE("if refuses a settle gate with no deadline", "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    c.align_settle_ms = 40;
    c.align_timeout_ms = 0;
    bind(&s, &f, &c);
    s.ops.at_rest = at_rest;
    /* Waiting on a rotor with no deadline parks i_align_a in the winding for as
     * long as it keeps twitching. */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_if_run(&s, 1));
    TEST_ASSERT_EQUAL(0, f.handoff_calls);
}

TEST_CASE("if holds the field still for the whole align", "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    c.align_ms = 200;
    c.align_settle_ms = 100;
    c.align_timeout_ms = 1000;
    bind(&s, &f, &c);
    f.lock_on_plateau = true;
    s.ops.on_plateau = on_plateau;
    s.ops.at_rest = at_rest;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_if_run(&s, 1));

    /* The regression this whole change exists for: while the rotor is being
     * pulled onto the d axis the commanded frequency must be exactly zero, or
     * the field walks away from a rotor that has not been captured yet. */
    const q16_t i_align = q16_from_float(0.30f);
    bool saw_align = false;
    bool saw_settle = false;
    for (int i = 0; i < f.n; i++) {
        if (f.phase_hist[i] == ESP_FOC_IF_PHASE_ALIGN) {
            saw_align = true;
            TEST_ASSERT_EQUAL(0, f.fe_hist[i]);
        } else if (f.phase_hist[i] == ESP_FOC_IF_PHASE_ALIGN_SETTLE) {
            saw_settle = true;
            TEST_ASSERT_EQUAL(0, f.fe_hist[i]);
            /* Nothing rotates until the current is all the way up. */
            TEST_ASSERT_INT32_WITHIN(Q16_ONE / 64, i_align, f.id_hist[i]);
        }
    }
    TEST_ASSERT_TRUE(saw_align);
    TEST_ASSERT_TRUE(saw_settle);
    /* 0.30 A at 1.5 A/s puts Id on target at t=180, then 100 ms of standstill. */
    TEST_ASSERT_EQUAL_UINT32(280u, f.ramp_start_ms);
}

TEST_CASE("if align waits for the rotor to stop moving", "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    c.align_ms = 40;
    c.align_settle_ms = 40;
    c.align_timeout_ms = 2000;
    bind(&s, &f, &c);
    f.lock_on_plateau = true;
    f.rest_after_ms = 300u;
    s.ops.on_plateau = on_plateau;
    s.ops.at_rest = at_rest;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_if_run(&s, 1));

    TEST_ASSERT_TRUE(f.rest_calls > 0);
    /* Quiet from 300 ms, 40 ms of it required: the ramp starts at 340 and does
     * not sit around waiting for the deadline. */
    TEST_ASSERT_EQUAL_UINT32(340u, f.ramp_start_ms);
}

TEST_CASE("if a vetoed align fails before anything rotates", "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    c.align_ms = 40;
    c.align_settle_ms = 40;
    c.align_timeout_ms = 2000;
    bind(&s, &f, &c);
    f.align_verdict = false;
    s.ops.at_rest = at_rest;
    s.ops.align_ok = align_ok;
    s.ops.on_plateau = on_plateau;
    f.lock_on_plateau = true;

    TEST_ASSERT_EQUAL(ESP_FAIL, esp_foc_if_run(&s, 1));
    TEST_ASSERT_EQUAL(1, f.align_ok_calls);
    TEST_ASSERT_EQUAL(ESP_FOC_IF_PHASE_FAIL, s.phase);
    /* The whole point is that the veto lands before the frame moves. */
    TEST_ASSERT_EQUAL(0, f.handoff_calls);
    TEST_ASSERT_EQUAL(0, f.plateau_calls);
    for (int i = 0; i < f.n; i++) {
        TEST_ASSERT_EQUAL_INT32(0, f.fe_hist[i]);
    }
    /* And it leaves nothing energised behind. */
    TEST_ASSERT_EQUAL_INT32(0, s.id);
    TEST_ASSERT_EQUAL_INT32(0, s.iq);
    TEST_ASSERT_EQUAL_INT32(0, s.fe_hz);
}

TEST_CASE("if a passed align is asked once and proceeds", "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    c.align_ms = 40;
    c.align_settle_ms = 40;
    c.align_timeout_ms = 2000;
    bind(&s, &f, &c);
    f.align_verdict = true;
    f.lock_on_plateau = true;
    s.ops.at_rest = at_rest;
    s.ops.align_ok = align_ok;
    s.ops.on_plateau = on_plateau;

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_if_run(&s, 1));
    TEST_ASSERT_EQUAL(1, f.align_ok_calls);
    TEST_ASSERT_EQUAL(1, f.handoff_calls);
}

TEST_CASE("if the align veto is asked after a timed-out align too",
          "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    c.align_ms = 20;
    c.align_settle_ms = 40;
    c.align_timeout_ms = 200;
    bind(&s, &f, &c);
    /* A rotor that never goes quiet: the deadline is what ends the align, and
     * that is exactly the case the caller most needs to be able to refuse. */
    f.rest_after_ms = 100000u;
    f.align_verdict = false;
    s.ops.at_rest = at_rest;
    s.ops.align_ok = align_ok;

    TEST_ASSERT_EQUAL(ESP_FAIL, esp_foc_if_run(&s, 1));
    TEST_ASSERT_EQUAL(1, f.align_ok_calls);
    TEST_ASSERT_EQUAL(0, f.handoff_calls);
}

TEST_CASE("if the ramp veto aborts the creep before the plateau",
          "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    bind(&s, &f, &c);
    f.lock_on_plateau = true;
    s.ops.on_plateau = on_plateau;
    s.ops.ramp_ok = ramp_ok;
    /* Two steps of a machine that takes current, then a loop that stops driving
     * it. The plateau must never be announced. */
    f.ramp_ok_until = 2;

    TEST_ASSERT_EQUAL(ESP_FAIL, esp_foc_if_run(&s, 1));
    TEST_ASSERT_EQUAL(ESP_FOC_IF_PHASE_FAIL, s.phase);
    TEST_ASSERT_EQUAL(3, f.ramp_ok_calls);
    TEST_ASSERT_EQUAL(0, f.plateau_calls);
    TEST_ASSERT_EQUAL(0, f.handoff_calls);
}

TEST_CASE("if the ramp veto is polled every creep step and can stay quiet",
          "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    bind(&s, &f, &c);
    f.lock_on_plateau = true;
    s.ops.on_plateau = on_plateau;
    s.ops.ramp_ok = ramp_ok;

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_if_run(&s, 1));
    TEST_ASSERT_EQUAL(1, f.handoff_calls);
    /* One ask per step of the ramp it supervises, not one per run. */
    TEST_ASSERT_GREATER_THAN_INT(1, f.ramp_ok_calls);
}

TEST_CASE("if a NULL ramp veto never blocks the creep", "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    bind(&s, &f, &c);
    f.lock_on_plateau = true;
    s.ops.on_plateau = on_plateau;
    s.ops.ramp_ok = NULL;

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_if_run(&s, 1));
    TEST_ASSERT_EQUAL(0, f.ramp_ok_calls);
    TEST_ASSERT_EQUAL(1, f.handoff_calls);
}

TEST_CASE("if align settle needs consecutive quiet", "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    c.align_ms = 20;
    c.align_settle_ms = 60;
    c.align_timeout_ms = 2000;
    bind(&s, &f, &c);
    f.lock_on_plateau = true;
    s.ops.on_plateau = on_plateau;
    s.ops.at_rest = at_rest;
    /* Quiet, quiet, twitch, then quiet for good. Two ticks of quiet before the
     * twitch must not count toward the three the config asks for. */
    f.rest_seq[0] = true;
    f.rest_seq[1] = true;
    f.rest_seq[2] = false;
    f.rest_seq[3] = true;
    f.rest_n = 4;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_if_run(&s, 1));

    /* Id is on target at the first tick, so the asks land at t=0,20,40(twitch),
     * 60,80,100,120 and the run of quiet that satisfies 60 ms ends at t=120. An
     * implementation that let the counter survive the twitch would release at
     * t=60, so this number is the whole test. */
    TEST_ASSERT_EQUAL_UINT32(120u, f.ramp_start_ms);
}

TEST_CASE("if align deadline starts the ramp on a rotor that never settles",
          "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    c.align_ms = 40;
    c.align_settle_ms = 100;
    c.align_timeout_ms = 200;
    bind(&s, &f, &c);
    f.lock_on_plateau = true;
    f.rest_seq[0] = false;
    f.rest_n = 1;
    s.ops.on_plateau = on_plateau;
    s.ops.at_rest = at_rest;
    /* Standing still with current in the winding is worse than ramping on a bad
     * angle, which the caller can still refuse at handoff. */
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_if_run(&s, 1));
    TEST_ASSERT_EQUAL(1, f.plateau_calls);
    TEST_ASSERT_EQUAL(50, f.plateau_fe);
    TEST_ASSERT_EQUAL_UINT32(200u, f.ramp_start_ms);
}

TEST_CASE("if align deadline never cuts the Id slew short", "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    c.align_ms = 200;
    c.align_settle_ms = 0;
    c.align_timeout_ms = 20;
    bind(&s, &f, &c);
    f.lock_on_plateau = true;
    s.ops.on_plateau = on_plateau;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_if_run(&s, 1));

    /* The deadline covers the wait for the rotor, not the current ramp: starting
     * the sweep at a fraction of i_align_a is the failure being fixed, so a
     * short timeout must not reintroduce it. */
    const q16_t i_align = q16_from_float(0.30f);
    /* A 20 ms deadline against a 200 ms slew: release at 180, not at 20. */
    TEST_ASSERT_EQUAL_UINT32(180u, f.ramp_start_ms);
    for (int i = 0; i < f.n; i++) {
        if (f.fe_hist[i] != 0) {
            TEST_ASSERT_INT32_WITHIN(Q16_ONE / 64, i_align, f.id_hist[i]);
        }
    }
}

TEST_CASE("if breaks away on voltage and hands back at the crossover",
          "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    c.f_max_hz = 40;
    c.vf_hz = 10.0f;
    c.vf_ramp_ms = 200u;
    c.vf_boost = 0.05f;
    c.vf_per_hz = 0.003f;
    bind(&s, &f, &c);
    f.lock_on_plateau = true;
    s.ops.on_plateau = on_plateau;
    s.ops.set_vdq = set_vdq;
    s.ops.to_current_mode = to_current_mode;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_if_run(&s, 1));

    TEST_ASSERT_TRUE(f.vdq_calls > 0);
    TEST_ASSERT_EQUAL(1, f.to_current_calls);
    /* The whole point is that the crossover is a boundary, not a blend: nothing
     * may drive voltage after the current loop has been handed the plant. */
    TEST_ASSERT_EQUAL(0, f.vdq_after_current);
    /* On q, so the excitation is already a quarter turn ahead of the rotor the
     * align parked on d — no separate lead step needed. */
    TEST_ASSERT_EQUAL(0, f.vdq_vd);
    TEST_ASSERT_TRUE(f.vdq_vq > 0);
    /* First point is the boost alone: fe is still 0 there, and a boost that only
     * arrived later would leave the first ticks under the bridge's dead zone. */
    TEST_ASSERT_INT32_WITHIN(Q16_ONE / 256, q16_from_float(0.05f),
                             f.vdq_vq_first);
    /* Last point is boost + slope·crossover. */
    TEST_ASSERT_INT32_WITHIN(Q16_ONE / 256, q16_from_float(0.05f + 0.003f * 10.0f),
                             f.vdq_vq);
    /* Voltage mode never carries the frame past the crossover, which is what
     * bounds the stall current it can produce. */
    TEST_ASSERT_INT32_WITHIN(Q16_ONE / 8, q16_from_float(10.0f), f.vdq_fe_max);
    TEST_ASSERT_EQUAL_INT(10, fe_int(f.fe_at_to_current));
}

TEST_CASE("if V/f sign follows the run direction", "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    c.f_max_hz = 40;
    c.vf_hz = 10.0f;
    c.vf_ramp_ms = 200u;
    c.vf_boost = 0.05f;
    c.vf_per_hz = 0.003f;
    bind(&s, &f, &c);
    f.lock_on_plateau = true;
    s.ops.on_plateau = on_plateau;
    s.ops.set_vdq = set_vdq;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_if_run(&s, -1));
    /* A positive q voltage on a reverse run would brake, not break away. */
    TEST_ASSERT_TRUE(f.vdq_vq < 0);
}

TEST_CASE("if refuses a V/f stage it cannot drive or that swallows the ramp",
          "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    c.f_max_hz = 40;
    c.vf_hz = 10.0f;
    c.vf_ramp_ms = 200u;

    /* Configured but undrivable: silently skipping it would run the startup the
     * config says is not safe on this bridge. */
    bind(&s, &f, &c);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_if_run(&s, 1));

    c.vf_ramp_ms = 0u;
    bind(&s, &f, &c);
    s.ops.set_vdq = set_vdq;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_if_run(&s, 1));

    /* A crossover at or above the plateau leaves no current-mode ramp, and the
     * voltage a following rotor needs up there is a stall overcurrent. */
    c.vf_ramp_ms = 200u;
    c.vf_hz = 40.0f;
    bind(&s, &f, &c);
    s.ops.set_vdq = set_vdq;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_if_run(&s, 1));
}

TEST_CASE("if runs on current alone when V/f is off", "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    c.vf_hz = 0.0f;
    bind(&s, &f, &c);
    f.lock_on_plateau = true;
    s.ops.on_plateau = on_plateau;
    s.ops.set_vdq = set_vdq;
    s.ops.to_current_mode = to_current_mode;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_if_run(&s, 1));
    TEST_ASSERT_EQUAL(0, f.vdq_calls);
    TEST_ASSERT_EQUAL(0, f.to_current_calls);
}

TEST_CASE("if leads the frame once, after the align and before the ramp",
          "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    c.align_ms = 40;
    c.align_settle_ms = 60;
    c.align_timeout_ms = 2000;
    c.lead_rad = 1.5707963f;
    bind(&s, &f, &c);
    f.lock_on_plateau = true;
    s.ops.on_plateau = on_plateau;
    s.ops.advance_theta = advance_theta;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_if_run(&s, 1));

    /* Once, and on the boundary: after the align released and while the frame is
     * still standing still, because a lead applied into a moving frame is just a
     * phase glitch and one applied during the align would be dragged out by the
     * rotor settling onto it. */
    TEST_ASSERT_EQUAL(1, f.lead_calls);
    TEST_ASSERT_EQUAL(0, f.lead_fe_at_call);
    TEST_ASSERT_INT32_WITHIN(Q16_ONE / 512, q16_from_float(1.5707963f),
                             f.lead_arg);
    TEST_ASSERT_EQUAL_UINT32(f.lead_ms, f.ramp_start_ms);
}

TEST_CASE("if lead follows the sign of the run", "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    c.lead_rad = 1.5707963f;
    bind(&s, &f, &c);
    f.lock_on_plateau = true;
    s.ops.on_plateau = on_plateau;
    s.ops.advance_theta = advance_theta;
    /* Leading the wrong way would put the torque against the requested
     * direction, which is worse than no lead at all. */
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_if_run(&s, -1));
    TEST_ASSERT_EQUAL(1, f.lead_calls);
    TEST_ASSERT_TRUE(f.lead_arg < 0);
}

TEST_CASE("if skips the lead when it is not configured", "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    c.lead_rad = 0.0f;
    bind(&s, &f, &c);
    f.lock_on_plateau = true;
    s.ops.on_plateau = on_plateau;
    s.ops.advance_theta = advance_theta;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_if_run(&s, 1));
    TEST_ASSERT_EQUAL(0, f.lead_calls);
}

TEST_CASE("if lock-in slews Id instead of stepping it", "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    bind(&s, &f, &c);
    f.lock_on_plateau = true;
    s.ops.on_plateau = on_plateau;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_if_run(&s, 1));

    TEST_ASSERT_TRUE(f.n >= 3);
    const q16_t i_align = q16_from_float(0.30f);
    TEST_ASSERT_TRUE(f.id_hist[0] < q16_from_float(0.22f));
    TEST_ASSERT_TRUE(f.id_hist[0] > 0);
    TEST_ASSERT_INT32_WITHIN(Q16_ONE / 8, i_align, f.id_hist[f.n - 1]);
    for (int i = 0; i < f.n; i++) {
        TEST_ASSERT_EQUAL(0, f.iq_hist[i]);
        TEST_ASSERT_TRUE(f.id_hist[i] >= 0);
        TEST_ASSERT_TRUE(f.id_hist[i] <= i_align + (Q16_ONE / 8));
    }
}

TEST_CASE("if lock-in ramps fe at constant accel", "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    bind(&s, &f, &c);
    f.lock_on_plateau = true;
    s.ops.on_plateau = on_plateau;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_if_run(&s, 1));

    int prev = -1;
    int dprev = 0;
    int nd = 0;
    bool saw_low = false;
    for (int i = 0; i < f.n; i++) {
        int a = fe_int(f.fe_hist[i]);
        if (a < 0) {
            a = -a;
        }
        if (a > 0 && a < 8) {
            saw_low = true;
        }
        if (prev >= 0 && a > prev && a < 50) {
            int d = a - prev;
            if (nd > 0) {
                TEST_ASSERT_INT_WITHIN(2, dprev, d);
            }
            dprev = d;
            nd++;
        }
        prev = a;
    }
    TEST_ASSERT_TRUE(saw_low);
    TEST_ASSERT_TRUE(nd >= 3);
}

TEST_CASE("if always reaches f_max before handoff", "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    bind(&s, &f, &c);
    f.lock_on_plateau = true;
    s.ops.on_plateau = on_plateau;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_if_run(&s, 1));
    TEST_ASSERT_EQUAL(1, f.handoff_calls);
    TEST_ASSERT_EQUAL(ESP_FOC_IF_PHASE_DONE, s.phase);
    TEST_ASSERT_EQUAL(1, f.plateau_calls);
    TEST_ASSERT_EQUAL(50, f.plateau_fe);
    int fea = fe_int(s.fe_hz);
    if (fea < 0) {
        fea = -fea;
    }
    TEST_ASSERT_EQUAL(50, fea);
}

TEST_CASE("if dang does not freeze the frequency ramp", "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    bind(&s, &f, &c);
    f.dang = q16_from_float(1.50f);
    f.lock_on_plateau = true;
    s.ops.on_plateau = on_plateau;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_if_run(&s, 1));
    TEST_ASSERT_EQUAL(1, f.handoff_calls);
    TEST_ASSERT_EQUAL(50, f.plateau_fe);
}

TEST_CASE("if plateau lock timeout notifies then still hands off", "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    c.lock_timeout_ms = 40;
    c.creep_ramp_ms = 80;
    bind(&s, &f, &c);
    s.ops.on_plateau = on_plateau;
    f.lock_after_ms = 160u;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_if_run(&s, 1));
    TEST_ASSERT_EQUAL(1, f.lock_to_calls);
    TEST_ASSERT_EQUAL(1, f.handoff_calls);
    TEST_ASSERT_EQUAL(ESP_FOC_IF_PHASE_DONE, s.phase);
    int fea = fe_int(s.fe_hz);
    if (fea < 0) {
        fea = -fea;
    }
    TEST_ASSERT_EQUAL(50, fea);
    TEST_ASSERT_EQUAL(1, f.plateau_calls);
}

TEST_CASE("if negative sign commands negative fe", "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    bind(&s, &f, &c);
    f.lock_on_plateau = true;
    s.ops.on_plateau = on_plateau;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_if_run(&s, -1));
    TEST_ASSERT_EQUAL(1, f.handoff_calls);

    bool saw_neg = false;
    for (int i = 0; i < f.n; i++) {
        TEST_ASSERT_TRUE(f.fe_hist[i] <= 0);
        if (f.fe_hist[i] < 0) {
            saw_neg = true;
        }
        TEST_ASSERT_EQUAL(0, f.iq_hist[i]);
    }
    TEST_ASSERT_TRUE(saw_neg);
    TEST_ASSERT_TRUE(s.fe_hz < 0);
}

TEST_CASE("if on_plateau fires once at f_max before lock_ok", "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    bind(&s, &f, &c);
    s.ops.on_plateau = on_plateau;
    f.lock_on_plateau = true;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_if_run(&s, 1));
    TEST_ASSERT_EQUAL(1, f.plateau_calls);
    TEST_ASSERT_EQUAL(50, f.plateau_fe);
    TEST_ASSERT_EQUAL(1, f.handoff_calls);
    TEST_ASSERT_EQUAL(0, f.lock_to_calls);
    TEST_ASSERT_TRUE(f.t_ms >= f.plateau_ms);
}

TEST_CASE("if accel_rads2 sets the ω slew independently of creep_ramp_ms", "[espFoC][if]")
{
    static esp_foc_if_t s;
    static fake_t f;
    esp_foc_if_config_t c = base_cfg();
    c.creep_ramp_ms = 2000;
    c.accel_rads2 = 2.0f * 3.14159265f * 50.0f / 0.10f;
    bind(&s, &f, &c);
    f.lock_on_plateau = true;
    s.ops.on_plateau = on_plateau;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_if_run(&s, 1));
    TEST_ASSERT_EQUAL(50, f.plateau_fe);
    TEST_ASSERT_TRUE(f.plateau_ms <= 140u);
}
