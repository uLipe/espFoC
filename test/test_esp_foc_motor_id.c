/*
 * Unit tests for the identification sequencer.
 *
 * The mock answers the probe ops analytically from a known machine, so these
 * tests cover ordering, gating and bookkeeping; the demodulation numerics are
 * covered in test_esp_foc_ident.c.
 */
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "esp_foc_motor_id_seq.h"
#include "espFoC/utils/esp_foc_q16.h"

#define SEQ_N 40

typedef struct {
    double r_ac_ohm;
    double r_dc_ohm;
    double l_h;
    double v_off;
    double psi_wb;
    int32_t term_ma[3];
    /** Delay the plant really has, in samples; fractional values are allowed so
     *  a residual the sweep cannot cancel can be exercised. */
    double true_lag;
    /** Scales R above 300 Hz, standing in for a winding whose R genuinely moves
     *  with frequency: no phase rotation can reconcile that. */
    double r_hi_scale;
    /** Scales the L reported at the coarse point only, standing in for how badly
     *  conditioned L is where arg(Z) is a few degrees. */
    double l_coarse_scale;
    /** Reports |Z| as R at the fine point, standing in for two windows measured
     *  at different frequencies landing within noise of each other. */
    bool z_mag_flat;
    /** Clips the excitation whenever it reverses, which is what the bridge dead
     *  zone does: R then reads low and |Z| falls below the DC resistance. */
    bool clip_on_reversal;
    int clipped_windows;
    /** Largest DC current any window drew, in mA. */
    int32_t dc_peak_ma;

    bool z_fail;
    bool term_fail;
    bool dc_fail;
    bool step_fail;
    double step_noise_a;
    int step_calls;
    /* IDLE means never; otherwise faulted() trips once this phase is entered. */
    esp_foc_motor_id_phase_t fault_at;

    float kp;
    float ki;
    int gains_calls;
    q16_t vd;
    q16_t vq;
    q16_t id;
    q16_t iq;
    q16_t fe;
    q16_t theta;
    int32_t fe_peak_hz;
    uint32_t t_ms;
    int true_pp;
    esp_foc_motor_id_phase_t phase;
    esp_foc_motor_id_phase_t seq[SEQ_N];
    int nseq;
    uint32_t probe_hz[8];
    int probe_calls;
    int32_t dc_mv[4];
    int dc_calls;
} mock_t;

static void m_theta(void *ctx, q16_t th) { ((mock_t *)ctx)->theta = th; }

static void m_vdq(void *ctx, q16_t vd, q16_t vq)
{
    mock_t *m = ctx;
    m->vd = vd;
    m->vq = vq;
}

static void m_idq(void *ctx, q16_t id, q16_t iq)
{
    mock_t *m = ctx;
    m->id = id;
    m->iq = iq;
}

static void m_fe(void *ctx, q16_t fe)
{
    mock_t *m = ctx;
    m->fe = fe;
    int32_t hz = (int32_t)(fe / 65536);
    if (hz > m->fe_peak_hz) {
        m->fe_peak_hz = hz;
    }
}

static bool m_fetch_rotor(void *ctx, q16_t *theta_m, q16_t *omega_m)
{
    mock_t *m = ctx;
    int pp = (m->true_pp > 0) ? m->true_pp : 13;
    int32_t fe_hz = (int32_t)(m->fe / 65536);
    if (fe_hz < 0) {
        fe_hz = -fe_hz;
    }
    float wm = 6.283185f * (float)fe_hz / (float)pp;
    *theta_m = 0;
    *omega_m = q16_from_float(wm);
    return true;
}

static void m_fetch(void *ctx, q16_t *vd, q16_t *vq, q16_t *id, q16_t *iq)
{
    mock_t *m = ctx;
    /* Steady-state q-axis balance of the commanded operating point. */
    double w = 2.0 * M_PI * (double)(m->fe / 65536);
    double id_a = q16_to_float(m->id);
    double iq_a = q16_to_float(m->iq);
    double vq_v = m->r_ac_ohm * iq_a + w * m->l_h * id_a + w * m->psi_wb;
    *vd = q16_from_float((float)(m->r_ac_ohm * id_a - w * m->l_h * iq_a));
    *vq = q16_from_float((float)vq_v);
    *id = m->id;
    *iq = m->iq;
}

static bool m_probe_z(void *ctx, q16_t theta, const esp_foc_ident_excite_t *e,
                      esp_foc_ident_z_t *out)
{
    mock_t *m = ctx;
    (void)theta;
    if (m->probe_calls < 8) {
        m->probe_hz[m->probe_calls] = e->f_hz;
    }
    m->probe_calls++;
    if (m->z_fail) {
        return false;
    }

    double r = m->r_ac_ohm * ((e->f_hz > 300u) ? m->r_hi_scale : 1.0);
    double w = 2.0 * M_PI * (double)e->f_hz;
    double wl = w * m->l_h;
    double z = sqrt(r * r + wl * wl);
    /* Correlating against the wrong command sample rotates the vector by the
     * mismatch times omega/fs. |Z| is untouched, only the R/L split moves. */
    double dphi = (m->true_lag - (double)e->lag_samples) * w / (double)e->fs_hz;
    /* The solve rotates the demodulated vector by the trim, adding it to the
     * angle it reports, so it lands on the residual with the same sign. */
    dphi += (double)e->phase_trim_cdeg * 0.01 * M_PI / 180.0;
    double phi = atan2(wl, r) + dphi;

    /*
     * The coarse point sits at a few degrees of arg(Z), where the quadrature term
     * is a small difference of large numbers, so its L carries far more error
     * than the fine point's. Modelled as a scale so the re-aim has something to
     * correct.
     */
    double l_seen = m->l_h;
    if (m->l_coarse_scale > 0.0 && e->f_hz <= 150u) {
        l_seen *= m->l_coarse_scale;
    }

    /*
     * The bias has to keep the current one-sided. When it does not, the bridge
     * clips the trough and the demodulated fundamental loses in-phase amplitude:
     * modelled as R and |Z| both reading low, which is what silicon showed.
     */
    const double i_ac = q16_to_float(e->v_amp) / z;
    const double i_dc = (q16_to_float(e->v_bias) - m->v_off) / m->r_dc_ohm;
    const int32_t dc_ma = (int32_t)(i_dc * 1000.0);
    if (dc_ma > m->dc_peak_ma) {
        m->dc_peak_ma = dc_ma;
    }
    double clip = 1.0;
    if (m->clip_on_reversal && i_dc < i_ac) {
        m->clipped_windows++;
        clip = 0.6;
    }

    memset(out, 0, sizeof(*out));
    out->r_mohm = (int32_t)(z * cos(phi) * clip * 1000.0);
    out->l_uh = (int32_t)(z * sin(phi) / w * 1e6 * (l_seen / m->l_h));
    out->z_mohm = (int32_t)(z * clip * 1000.0);
    if (m->z_mag_flat && e->f_hz > 300u) {
        out->z_mohm = (int32_t)(m->r_ac_ohm * 1000.0);
    }
    out->phase_cdeg = (int32_t)(phi * 18000.0 / M_PI);
    out->i_amp_ma = (int32_t)(i_ac * 1000.0);
    out->f_hz = (int32_t)e->f_hz;
    out->valid = (out->r_mohm > 0);
    return out->valid;
}

static bool m_probe_dc(void *ctx, q16_t theta, q16_t vd, int32_t *i_ma)
{
    mock_t *m = ctx;
    (void)theta;
    if (m->dc_calls < 4) {
        m->dc_mv[m->dc_calls] = (int32_t)(q16_to_float(vd) * 1000.0f);
    }
    m->dc_calls++;
    if (m->dc_fail) {
        return false;
    }
    /* Fixed bridge offset in series with the DC resistance. */
    double v = q16_to_float(vd) - m->v_off;
    if (v < 0.0) {
        v = 0.0;
    }
    *i_ma = (int32_t)(v / m->r_dc_ohm * 1000.0);
    return true;
}

/**
 * A step window with the plant's own delay in it.
 *
 * The response only has to be recognisable, not exact: what is under test is
 * whether the sequence reads the delay it was given, so this is the ZOH rise of
 * the mock's R and L, shifted by true_lag and optionally buried in noise.
 */
static bool m_probe_step(void *ctx, q16_t theta, q16_t vd, uint32_t n_pre,
                         q16_t *i_out, uint32_t n)
{
    mock_t *m = ctx;
    (void)theta;
    if (m->step_fail || i_out == NULL || n_pre == 0u || n <= n_pre) {
        return false;
    }
    m->step_calls++;

    const double fs = 20000.0;
    const double a = exp(-m->r_ac_ohm / (m->l_h * fs));
    double v = q16_to_float(vd) - m->v_off;
    if (v < 0.0) {
        v = 0.0;
    }
    const double lag = m->true_lag;
    double i = 0.0;
    double hist[64] = {0};

    for (uint32_t k = 0; k < n && k < 64u; k++) {
        hist[k] = i;
        const uint32_t src = (k + 1u >= (uint32_t)lag) ? (uint32_t)(k + 1u - (uint32_t)lag) : 0u;
        double meas = (k + 1u >= (uint32_t)lag) ? hist[src] : 0.0;
        meas += m->step_noise_a * ((double)((int)(k * 2654435761u >> 24) % 200 - 100) / 100.0);
        i_out[k] = q16_from_float((float)meas);
        i = i * a + ((k >= n_pre) ? (v / m->r_ac_ohm) * (1.0 - a) : 0.0);
    }
    return true;
}

static bool m_probe_term(void *ctx, int terminal, q16_t v, int32_t *i_ma)
{
    mock_t *m = ctx;
    (void)v;
    if (m->term_fail || terminal < 0 || terminal > 2) {
        return false;
    }
    *i_ma = m->term_ma[terminal];
    return true;
}

static void m_gains(void *ctx, float kp, float ki)
{
    mock_t *m = ctx;
    m->kp = kp;
    m->ki = ki;
    m->gains_calls++;
}

static void m_sleep(void *ctx, uint32_t ms) { ((mock_t *)ctx)->t_ms += ms; }

static bool m_faulted(void *ctx)
{
    mock_t *m = ctx;
    return (m->fault_at != ESP_FOC_MOTOR_ID_IDLE) && (m->phase == m->fault_at);
}

static void m_phase(void *ctx, esp_foc_motor_id_phase_t ph)
{
    mock_t *m = ctx;
    m->phase = ph;
    if (m->nseq < SEQ_N) {
        m->seq[m->nseq++] = ph;
    }
}

/* Reference bench machine, 1.9 ohm / 500 uH on a 12 V bus. */
static void bind(esp_foc_motor_id_seq_t *s, mock_t *m)
{
    memset(s, 0, sizeof(*s));
    memset(m, 0, sizeof(*m));

    m->r_ac_ohm = 1.9;
    m->r_dc_ohm = 1.9;
    m->l_h = 500e-6;
    m->v_off = 0.29;
    m->psi_wb = 0.0018;
    m->term_ma[0] = 300;
    m->term_ma[1] = 305;
    m->term_ma[2] = 298;
    m->fault_at = ESP_FOC_MOTOR_ID_IDLE;
    m->true_lag = 1.0;
    m->r_hi_scale = 1.0;
    m->true_pp = 13;

    esp_foc_motor_id_seq_default_config(&s->cfg);
    s->cfg.vdc = 12.0f;
    s->cfg.pwm_hz = 20000u;
    s->cfg.pole_pairs = 13;
    s->cfg.i_bw_hz = 300.0f;

    s->ops.ctx = m;
    s->ops.set_theta = m_theta;
    s->ops.set_vdq = m_vdq;
    s->ops.set_idq = m_idq;
    s->ops.set_fe_hz = m_fe;
    s->ops.fetch_dq = m_fetch;
    s->ops.probe_z = m_probe_z;
    s->ops.probe_dc = m_probe_dc;
    s->ops.probe_terminal = m_probe_term;
    s->ops.apply_gains = m_gains;
    s->ops.sleep_ms = m_sleep;
    s->ops.faulted = m_faulted;
    s->ops.on_phase = m_phase;
}

/*
 * Same machine, but the adapter can capture a step window. Silicon always can, so
 * this is the normal path; bind() without it keeps the frequency sweep covered as
 * the fallback for an adapter that cannot hand back per-sample data.
 */
static void bind_step(esp_foc_motor_id_seq_t *s, mock_t *m)
{
    bind(s, m);
    s->ops.probe_step = m_probe_step;
}

static bool saw(const mock_t *m, esp_foc_motor_id_phase_t ph)
{
    for (int i = 0; i < m->nseq; i++) {
        if (m->seq[i] == ph) {
            return true;
        }
    }
    return false;
}

static int idx_of(const mock_t *m, esp_foc_motor_id_phase_t ph)
{
    for (int i = 0; i < m->nseq; i++) {
        if (m->seq[i] == ph) {
            return i;
        }
    }
    return -1;
}

TEST_CASE("motor_id rejects incomplete wiring", "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_motor_id_seq_run(NULL));

    bind(&s, &m);
    s.ops.probe_z = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_motor_id_seq_run(&s));

    bind(&s, &m);
    s.cfg.pole_pairs = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_motor_id_seq_run(&s));

    bind(&s, &m);
    s.cfg.vdc = 0.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_motor_id_seq_run(&s));

    /* The spinning half needs ops the standstill half does not. */
    bind(&s, &m);
    s.cfg.do_flux = true;
    s.ops.fetch_dq = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_motor_id_seq_run(&s));
}

TEST_CASE("motor_id walks the standstill sequence in order", "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;
    bind(&s, &m);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_EQUAL(ESP_FOC_MOTOR_ID_DONE, s.phase);

    TEST_ASSERT_TRUE(idx_of(&m, ESP_FOC_MOTOR_ID_BIAS) <
                     idx_of(&m, ESP_FOC_MOTOR_ID_TERMINAL));
    /*
     * The DC slope comes before the AC probes: it is the only measurement that
     * does not assume a linear bridge, and both the bias that keeps the AC probes
     * out of the dead zone and the dead zone itself come from it.
     */
    TEST_ASSERT_TRUE(idx_of(&m, ESP_FOC_MOTOR_ID_TERMINAL) <
                     idx_of(&m, ESP_FOC_MOTOR_ID_RS));
    TEST_ASSERT_TRUE(idx_of(&m, ESP_FOC_MOTOR_ID_RS) <
                     idx_of(&m, ESP_FOC_MOTOR_ID_ROVERL_COARSE));
    /* The delay sweep needs the coarse R as its anchor and must settle before
     * the fine probe it protects. */
    TEST_ASSERT_TRUE(idx_of(&m, ESP_FOC_MOTOR_ID_ROVERL_COARSE) <
                     idx_of(&m, ESP_FOC_MOTOR_ID_LAGCAL));
    TEST_ASSERT_TRUE(idx_of(&m, ESP_FOC_MOTOR_ID_LAGCAL) <
                     idx_of(&m, ESP_FOC_MOTOR_ID_ROVERL_FINE));
    /* Nothing may be tuned before the impedance is known. */
    TEST_ASSERT_TRUE(idx_of(&m, ESP_FOC_MOTOR_ID_ROVERL_FINE) <
                     idx_of(&m, ESP_FOC_MOTOR_ID_TUNE_I));

    /* Standstill run must not touch the spinning states. */
    TEST_ASSERT_FALSE(saw(&m, ESP_FOC_MOTOR_ID_RAMPUP));
    TEST_ASSERT_FALSE(saw(&m, ESP_FOC_MOTOR_ID_RATED_FLUX));
    TEST_ASSERT_FALSE(saw(&m, ESP_FOC_MOTOR_ID_FAIL));
}

TEST_CASE("motor_id reports the machine it measured", "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;
    bind(&s, &m);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_run(&s));

    const esp_foc_motor_id_result_t *r = &s.result;
    TEST_ASSERT_INT_WITHIN(100, 1900, (int)(r->r_loop_ohm * 1000.0f));
    TEST_ASSERT_INT_WITHIN(20, 500, (int)(r->ls_h * 1e6f));
    TEST_ASSERT_INT_WITHIN(200, 3800, (int)r->roverl_rad_s);
    TEST_ASSERT_EQUAL_INT(13, r->pole_pairs);

    /* The DC slope must shed the 0.29 V bridge offset, which a single point
     * would have reported as roughly 1.23 ohm. */
    TEST_ASSERT_INT_WITHIN(100, 1900, (int)(r->rs_ohm * 1000.0f));

    uint32_t want = ESP_FOC_MOTOR_ID_VALID_SYMMETRY |
                    ESP_FOC_MOTOR_ID_VALID_R_LOOP |
                    ESP_FOC_MOTOR_ID_VALID_LS |
                    ESP_FOC_MOTOR_ID_VALID_RS |
                    ESP_FOC_MOTOR_ID_VALID_GAINS;
    TEST_ASSERT_EQUAL_INT32(want, r->valid_mask);
    TEST_ASSERT_EQUAL_INT(0, r->valid_mask & ESP_FOC_MOTOR_ID_VALID_PSI_F);
}

TEST_CASE("motor_id lands the fine probe where Z is well conditioned",
          "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;
    bind(&s, &m);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_run(&s));

    TEST_ASSERT_TRUE(m.probe_calls >= 2);
    TEST_ASSERT_EQUAL_INT32(100, m.probe_hz[0]);
    /*
     * Anywhere R and omega*L are comparable will do; 45 degrees is the middle of
     * that band, not a target to hit exactly. What matters is that neither part of
     * Z is a small difference of large numbers where it is read.
     */
    TEST_ASSERT_TRUE(s.result.probe_phase_cdeg > 3000);
    TEST_ASSERT_TRUE(s.result.probe_phase_cdeg < 6000);
    TEST_ASSERT_INT_WITHIN(40, 500, (int)(s.result.ls_h * 1e6f));
}

TEST_CASE("motor_id ignores the coarse L when aiming", "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;
    bind(&s, &m);
    /*
     * On silicon the coarse point reported 154, 265 and even -477 uH on
     * consecutive runs of a 457 uH winding, because the quadrature current that
     * carries L there is about 18 mA of a 176 mA response and the sense noise
     * averages down to roughly 11 mA. Aiming from that number put the fine probe
     * anywhere between 605 and 2500 Hz, so the aim must not consult it at all.
     */
    m.l_coarse_scale = 0.34;

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_run(&s));

    TEST_ASSERT_TRUE(s.result.probe_phase_cdeg > 3000);
    TEST_ASSERT_TRUE(s.result.probe_phase_cdeg < 6000);
    TEST_ASSERT_INT_WITHIN(40, 500, (int)(s.result.ls_h * 1e6f));
}

TEST_CASE("motor_id clamps the fine probe to what the carrier allows",
          "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;

    /* Air-core-ish: R/L would ask for far above fs/8. */
    bind(&s, &m);
    m.l_h = 20e-6;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_TRUE(s.result.probe_hz <= 20000.0f / 8.0f);

    /* Very inductive: R/L falls below the coarse point, which is already the
     * better-conditioned side, so the coarse frequency is kept. */
    bind(&s, &m);
    m.l_h = 40e-3;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_EQUAL_INT32(100, (int)s.result.probe_hz);
    TEST_ASSERT_INT_WITHIN(4000, 40000, (int)(s.result.ls_h * 1e6f));
}

TEST_CASE("motor_id can skip the open-loop DC states", "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;
    bind(&s, &m);
    s.cfg.skip_terminal = true;
    s.cfg.skip_rs = true;
    /* A free shaft is why these are skipped, so their ops may be absent. */
    s.ops.probe_terminal = NULL;
    s.ops.probe_dc = NULL;

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_FALSE(saw(&m, ESP_FOC_MOTOR_ID_TERMINAL));
    TEST_ASSERT_FALSE(saw(&m, ESP_FOC_MOTOR_ID_RS));

    /* The AC half still delivers everything the current loop needs. */
    TEST_ASSERT_TRUE(s.result.valid_mask & ESP_FOC_MOTOR_ID_VALID_R_LOOP);
    TEST_ASSERT_TRUE(s.result.valid_mask & ESP_FOC_MOTOR_ID_VALID_LS);
    TEST_ASSERT_TRUE(s.result.valid_mask & ESP_FOC_MOTOR_ID_VALID_GAINS);
    TEST_ASSERT_FALSE(s.result.valid_mask & ESP_FOC_MOTOR_ID_VALID_SYMMETRY);
    TEST_ASSERT_FALSE(s.result.valid_mask & ESP_FOC_MOTOR_ID_VALID_RS);
    TEST_ASSERT_INT_WITHIN(25, 500, (int)(s.result.ls_h * 1e6f));

    /* Skipping does not waive the ops the remaining states need. */
    bind(&s, &m);
    s.cfg.skip_terminal = true;
    s.cfg.skip_rs = true;
    s.ops.probe_z = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_motor_id_seq_run(&s));
}

TEST_CASE("motor_id measures a sense delay it was configured wrong for",
          "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;
    bind(&s, &m);
    m.true_lag = 2.0;
    s.cfg.lag_samples = 1u; /* the seed is wrong on purpose */

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_EQUAL_UINT32(2u, s.result.lag_samples);
    /* Having found the delay, L comes back right. */
    TEST_ASSERT_INT_WITHIN(25, 500, (int)(s.result.ls_h * 1e6f));
    TEST_ASSERT_INT_WITHIN(100, 1900, (int)(s.result.r_loop_ohm * 1000.0f));

    /*
     * The winner is the minimum among the candidates that resolved. Some do not:
     * a delay far from the truth rotates the vector past the real axis and implies
     * a negative inductance, which is refused outright rather than scored.
     */
    TEST_ASSERT_TRUE(s.result.lag_r_permil[2] >= 0);
    for (int lag = 0; lag < ESP_FOC_IDENT_LAG_MAX; lag++) {
        if (s.result.lag_r_permil[lag] >= 0) {
            TEST_ASSERT_TRUE(s.result.lag_r_permil[2] <=
                             s.result.lag_r_permil[lag]);
        }
    }
    TEST_ASSERT_TRUE(s.result.lag_r_permil[1] > 100);
}

TEST_CASE("motor_id reaches a delay beyond four samples", "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;
    bind(&s, &m);
    /*
     * The ADC chain on this bench measures about 3.5 PWM periods behind the
     * command. A sweep that stopped at 3 left a residual that read L 42 per cent
     * low at the coarse point and then aimed the fine probe at 1678 Hz, where one
     * sample is 30 degrees and nothing could reconcile.
     */
    m.true_lag = 5.0;

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_EQUAL_UINT32(5u, s.result.lag_samples);
    TEST_ASSERT_INT_WITHIN(25, 500, (int)(s.result.ls_h * 1e6f));
    /* R at the 45 degree point is the loop resistance, not the DC one, and it
     * carries the conditioning of that angle; the coarse probe is the R anchor. */
    TEST_ASSERT_INT_WITHIN(150, 1900, (int)(s.result.r_loop_ohm * 1000.0f));
}

TEST_CASE("motor_id solves the sub-sample remainder of the delay",
          "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;
    bind(&s, &m);
    /* No integer reference can cancel 0.4 of a sample, so the sweep has to solve
     * the remainder from dR/R = dphi * tan(arg Z) instead of leaving it in L. */
    m.true_lag = 2.4;

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_run(&s));
    /* 0.4 samples at the 605 Hz fine point is 4.4 degrees the reference cannot
     * remove, so the trim rotates back. First order under-corrects; the point is
     * that it moves the right way and L lands closer than the residual allows. */
    TEST_ASSERT_TRUE(s.result.phase_trim_cdeg < -100);
    TEST_ASSERT_INT_WITHIN(60, 500, (int)(s.result.ls_h * 1e6f));
}

TEST_CASE("motor_id delay sweep tolerates a sub-sample residual",
          "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;
    bind(&s, &m);
    /* Half a sample is the ZOH-order residual the solve cannot place on either
     * integer candidate; it must not be treated as a failure. */
    m.true_lag = 1.5;

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_TRUE(s.result.lag_samples == 1u || s.result.lag_samples == 2u);
    TEST_ASSERT_TRUE(s.result.valid_mask & ESP_FOC_MOTOR_ID_VALID_LS);
}

TEST_CASE("motor_id skip_lag_cal keeps the caller's delay",
          "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;
    bind(&s, &m);
    m.true_lag = 2.0;
    s.cfg.skip_lag_cal = true;
    s.cfg.lag_samples = 1u;

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_EQUAL_UINT32(1u, s.result.lag_samples);
    TEST_ASSERT_EQUAL_INT32(-1, s.result.lag_r_permil[0]);

    /*
     * L survives the wrong delay because it is taken from |Z| and R, neither of
     * which the angle enters. The sweep still earns its place — it is what makes
     * the reported angle and the trim mean anything — but a caller who skips it no
     * longer forfeits the inductance.
     */
    TEST_ASSERT_INT_WITHIN(40, 500, (int)(s.result.ls_h * 1e6f));
    TEST_ASSERT_INT_WITHIN(400, 1900, (int)(s.result.r_loop_ohm * 1000.0f));
}

TEST_CASE("motor_id refuses when no delay reconciles the two frequencies",
          "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;
    bind(&s, &m);
    /* R doubles above 300 Hz, so the disagreement is not a phase error and no
     * candidate can absorb it. Guessing L here would be worse than stopping. */
    m.r_hi_scale = 2.0;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_RESPONSE, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_EQUAL(ESP_FOC_MOTOR_ID_LAGCAL, s.result.failed_at);
    TEST_ASSERT_EQUAL_INT(0, m.gains_calls);
    TEST_ASSERT_FALSE(s.result.valid_mask & ESP_FOC_MOTOR_ID_VALID_LS);
    TEST_ASSERT_EQUAL_INT32(0, m.vd);
}

TEST_CASE("motor_id skips the delay sweep when the probes land together",
          "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;
    bind(&s, &m);
    /* 40 mH puts R/L below the coarse point, so arg(Z) there is already 86 degrees
     * and R is the fragile term at both ends: the sweep has no anchor to
     * discriminate against and must not be attempted. */
    m.l_h = 40e-3;
    m.true_lag = 2.0;
    s.cfg.lag_samples = 1u;

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_EQUAL_UINT32(1u, s.result.lag_samples);
    TEST_ASSERT_EQUAL_INT32(-1, s.result.lag_r_permil[0]);
}

TEST_CASE("motor_id refuses an open delta winding before tuning",
          "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;
    bind(&s, &m);
    /* The damaged bench motor: one terminal pulled twice the current. */
    m.term_ma[0] = 150;
    m.term_ma[1] = 150;
    m.term_ma[2] = 300;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_RESPONSE, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_EQUAL(ESP_FOC_MOTOR_ID_FAIL, s.phase);
    TEST_ASSERT_EQUAL(ESP_FOC_MOTOR_ID_TERMINAL, s.result.failed_at);

    /* No gain may be designed against a machine like this, and no probe should
     * even have been attempted. */
    TEST_ASSERT_EQUAL_INT(0, m.gains_calls);
    TEST_ASSERT_EQUAL_INT(0, m.probe_calls);
    TEST_ASSERT_EQUAL_INT32(0, s.result.valid_mask);
    TEST_ASSERT_INT_WITHIN(10, 500, (int)(s.result.symmetry_spread * 1000.0f));

    /* Outputs are parked even on refusal. */
    TEST_ASSERT_EQUAL_INT32(0, m.vd);
    TEST_ASSERT_EQUAL_INT32(0, m.vq);
}

TEST_CASE("motor_id stops at the phase that faulted", "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;

    bind(&s, &m);
    m.fault_at = ESP_FOC_MOTOR_ID_ROVERL_COARSE;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_EQUAL(ESP_FOC_MOTOR_ID_FAIL, s.phase);
    TEST_ASSERT_EQUAL(ESP_FOC_MOTOR_ID_ROVERL_COARSE, s.result.failed_at);
    TEST_ASSERT_EQUAL_INT(0, m.gains_calls);
    TEST_ASSERT_EQUAL_INT32(0, m.vd);

    /* A partial run still reports what it had already established. */
    bind(&s, &m);
    m.fault_at = ESP_FOC_MOTOR_ID_RS;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_EQUAL(ESP_FOC_MOTOR_ID_RS, s.result.failed_at);
    TEST_ASSERT_TRUE(s.result.valid_mask & ESP_FOC_MOTOR_ID_VALID_SYMMETRY);
    TEST_ASSERT_FALSE(s.result.valid_mask & ESP_FOC_MOTOR_ID_VALID_LS);
    TEST_ASSERT_FALSE(s.result.valid_mask & ESP_FOC_MOTOR_ID_VALID_GAINS);
}

TEST_CASE("motor_id propagates a refused probe", "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;

    bind(&s, &m);
    m.z_fail = true;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_RESPONSE, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_EQUAL(ESP_FOC_MOTOR_ID_ROVERL_COARSE, s.result.failed_at);

    bind(&s, &m);
    m.term_fail = true;
    TEST_ASSERT_EQUAL(ESP_FAIL, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_EQUAL(ESP_FOC_MOTOR_ID_TERMINAL, s.result.failed_at);

    bind(&s, &m);
    m.dc_fail = true;
    TEST_ASSERT_EQUAL(ESP_FAIL, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_EQUAL(ESP_FOC_MOTOR_ID_RS, s.result.failed_at);
}

TEST_CASE("motor_id refuses an open circuit on the probe", "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;
    bind(&s, &m);
    /* 500 ohm of nothing: the response falls under the usable floor. */
    m.r_ac_ohm = 500.0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_RESPONSE, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_EQUAL(ESP_FOC_MOTOR_ID_ROVERL_COARSE, s.result.failed_at);
    TEST_ASSERT_EQUAL_INT(0, m.gains_calls);
}

TEST_CASE("motor_id backs the excitation off a very low impedance",
          "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;
    bind(&s, &m);
    /* 50 mohm: the seed excitation would ask for about 17 A. */
    m.r_ac_ohm = 0.05;
    m.r_dc_ohm = 0.05;
    s.cfg.i_probe_max_a = 1.0f;

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_TRUE(s.result.probe_i_ma <= 1000);
    TEST_ASSERT_INT_WITHIN(10, 50, (int)(s.result.r_loop_ohm * 1000.0f));
    /* The search had to drop well below the seed of 7% of 12 V. */
    TEST_ASSERT_TRUE(s.result.probe_v_mv < 840);
}

TEST_CASE("motor_id lifts the excitation for a very inductive machine",
          "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;
    bind(&s, &m);
    /* 40 mH: over 25 ohm at the coarse point, where the seed yields 33 mA. */
    m.l_h = 40e-3;

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_TRUE(s.result.probe_v_mv > 840);
    TEST_ASSERT_TRUE(s.result.probe_i_ma >= 100);
    TEST_ASSERT_INT_WITHIN(2000, 40000, (int)(s.result.ls_h * 1e6f));
}

TEST_CASE("motor_id identifies flux and parks the shaft", "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;
    bind(&s, &m);
    s.cfg.do_flux = true;
    s.cfg.flux_hz = 100;
    s.cfg.ramp_ms = 200u;
    s.cfg.flux_hold_ms = 100u;

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_EQUAL(ESP_FOC_MOTOR_ID_DONE, s.phase);
    TEST_ASSERT_TRUE(saw(&m, ESP_FOC_MOTOR_ID_RAMPUP));
    TEST_ASSERT_TRUE(saw(&m, ESP_FOC_MOTOR_ID_RATED_FLUX));
    TEST_ASSERT_TRUE(saw(&m, ESP_FOC_MOTOR_ID_RAMPDOWN));
    TEST_ASSERT_TRUE(idx_of(&m, ESP_FOC_MOTOR_ID_RAMPUP) >
                     idx_of(&m, ESP_FOC_MOTOR_ID_TUNE_I));

    /* 1.8 mWb, which is what the bench voltage ceiling implied and well under
     * the 2.56 mWb the nameplate Kt suggested. */
    TEST_ASSERT_TRUE(s.result.valid_mask & ESP_FOC_MOTOR_ID_VALID_PSI_F);
    TEST_ASSERT_INT_WITHIN(150, 1800, (int)(s.result.psi_f_wb * 1e6f));

    TEST_ASSERT_EQUAL_INT32(100, m.fe_peak_hz);
    /* Nothing is left energised. */
    TEST_ASSERT_EQUAL_INT32(0, m.fe);
    TEST_ASSERT_EQUAL_INT32(0, m.id);
    TEST_ASSERT_EQUAL_INT32(0, m.iq);
    TEST_ASSERT_EQUAL_INT32(0, m.vd);
}

TEST_CASE("motor_id standstill accepts unknown pp when a rotor is bound",
          "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;
    bind(&s, &m);
    s.cfg.pole_pairs = 0;
    s.ops.fetch_rotor = m_fetch_rotor;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_EQUAL(ESP_FOC_MOTOR_ID_DONE, s.phase);
}

TEST_CASE("motor_id measures pole pairs from encoder speed", "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;
    bind(&s, &m);
    s.cfg.pole_pairs = 0;
    s.cfg.do_flux = true;
    s.cfg.flux_hz = 100;
    s.cfg.ramp_ms = 200u;
    s.cfg.flux_hold_ms = 100u;
    m.true_pp = 7;
    s.ops.fetch_rotor = m_fetch_rotor;

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_TRUE(s.result.valid_mask & ESP_FOC_MOTOR_ID_VALID_PP);
    TEST_ASSERT_EQUAL(7, s.result.pole_pairs);
    TEST_ASSERT_TRUE(s.result.valid_mask & ESP_FOC_MOTOR_ID_VALID_PSI_F);
    TEST_ASSERT_INT_WITHIN(150, 1800, (int)(s.result.psi_f_wb * 1e6f));
}

TEST_CASE("motor_id gains follow the plant and honour the backoff",
          "[espFoC][motor_id]")
{
    esp_foc_motor_id_seq_config_t cfg;
    esp_foc_motor_id_seq_default_config(&cfg);
    cfg.vdc = 12.0f;
    cfg.pwm_hz = 20000u;
    cfg.pole_pairs = 13;
    cfg.i_bw_hz = 300.0f;
    cfg.tune_backoff = 1.0f;

    float kp = 0.0f;
    float ki = 0.0f;
    TEST_ASSERT_EQUAL(ESP_OK,
                      esp_foc_motor_id_seq_gains_for(&cfg, 1.9f, 500e-6f, &kp, &ki));
    TEST_ASSERT_TRUE(kp > 0.0f);
    TEST_ASSERT_TRUE(ki > 0.0f);

    float kp_half = 0.0f;
    float ki_half = 0.0f;
    cfg.tune_backoff = 0.5f;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_gains_for(&cfg, 1.9f, 500e-6f,
                                                         &kp_half, &ki_half));
    TEST_ASSERT_FLOAT_WITHIN(kp * 0.01f, kp * 0.5f, kp_half);
    TEST_ASSERT_FLOAT_WITHIN(ki * 0.01f, ki * 0.5f, ki_half);

    /*
     * Thermal tracking hook: only R moves with winding temperature. IMC puts the
     * zero at R/L, so a hotter winding raises Ki while Kp, which is set by L,
     * stays put. A tracker can therefore redesign without re-identifying.
     */
    cfg.tune_backoff = 1.0f;
    float kp_hot = 0.0f;
    float ki_hot = 0.0f;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_gains_for(&cfg, 2.4f, 500e-6f,
                                                         &kp_hot, &ki_hot));
    TEST_ASSERT_TRUE(ki_hot > ki);
    TEST_ASSERT_FLOAT_WITHIN(kp * 0.05f, kp, kp_hot);

    /* Zero bandwidth means auto, not invalid. */
    cfg.i_bw_hz = 0.0f;
    float kp_auto = 0.0f;
    float ki_auto = 0.0f;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_gains_for(&cfg, 1.9f, 500e-6f,
                                                         &kp_auto, &ki_auto));
    TEST_ASSERT_TRUE(kp_auto > 0.0f);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      esp_foc_motor_id_seq_gains_for(&cfg, 0.0f, 500e-6f, &kp, &ki));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      esp_foc_motor_id_seq_gains_for(&cfg, 1.9f, 0.0f, &kp, &ki));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      esp_foc_motor_id_seq_gains_for(NULL, 1.9f, 500e-6f, &kp, &ki));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      esp_foc_motor_id_seq_gains_for(&cfg, 1.9f, 500e-6f, NULL, &ki));
}

TEST_CASE("motor_id phase names cover the enum", "[espFoC][motor_id]")
{
    for (int ph = ESP_FOC_MOTOR_ID_IDLE; ph <= ESP_FOC_MOTOR_ID_FAIL; ph++) {
        const char *n = esp_foc_motor_id_phase_name((esp_foc_motor_id_phase_t)ph);
        TEST_ASSERT_NOT_NULL(n);
        TEST_ASSERT_TRUE(n[0] != '?');
    }
}

TEST_CASE("motor_id times the delay off a step", "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;

    for (double lag = 1.0; lag <= 6.0; lag += 1.0) {
        bind_step(&s, &m);
        m.true_lag = lag;

        TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_run(&s));

        TEST_ASSERT_EQUAL_UINT32((uint32_t)lag, s.result.lag_samples);
        TEST_ASSERT_TRUE(m.step_calls > 0);
        /* The sampling instant is locked to the carrier, so there is no fraction
         * left for a trim to chase. */
        TEST_ASSERT_EQUAL_INT32(0, s.result.phase_trim_cdeg);
        TEST_ASSERT_INT_WITHIN(60, 500, (int)(s.result.ls_h * 1e6f));
    }
}

TEST_CASE("motor_id times the delay when R moves with frequency",
          "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;

    /*
     * The case that defeated the sweep on silicon. Iron loss raises R at the
     * higher probe point, so no rotation reconciles the two frequencies and the
     * sweep descended monotonically until L went negative. A step never asks R
     * anything.
     */
    bind_step(&s, &m);
    m.true_lag = 3.0;
    m.r_hi_scale = 1.3;

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_run(&s));

    TEST_ASSERT_EQUAL_UINT32(3u, s.result.lag_samples);
    TEST_ASSERT_TRUE(s.result.ls_h > 0.0f);
    TEST_ASSERT_INT_WITHIN(100, 500, (int)(s.result.ls_h * 1e6f));
    /* And it reports the mismatch it did not act on. */
    TEST_ASSERT_TRUE(s.result.lag_r_permil[3] > 100);
}

TEST_CASE("motor_id step timing holds under sense noise", "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;

    bind_step(&s, &m);
    m.true_lag = 2.0;
    /*
     * What is left after the adapter has averaged its repetitions, not the raw
     * floor: this bench's 200 mA of per-sample sigma over 96 steps comes to about
     * 20. The bar the kernel derives from that is 60 mA against a first sample of
     * 110 on this plant, and it is worth knowing the margin is only twice — a
     * winding several times more inductive would need more repetitions, not a
     * lower bar.
     */
    m.step_noise_a = 0.02;

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_EQUAL_UINT32(2u, s.result.lag_samples);
}

TEST_CASE("motor_id refuses when the step never answers", "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;

    bind_step(&s, &m);
    m.step_fail = true;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_RESPONSE, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_EQUAL(ESP_FOC_MOTOR_ID_LAGCAL, s.result.failed_at);
    TEST_ASSERT_EQUAL_UINT32(0u, s.result.valid_mask & ESP_FOC_MOTOR_ID_VALID_LS);
}

TEST_CASE("motor_id takes L from the angle when the magnitude cannot give one",
          "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;

    bind_step(&s, &m);
    /*
     * Silicon: |Z| at the 600 Hz fine point read 2309 mohm against an anchor R of
     * 2350 measured at 100 Hz, so sqrt(Z^2 - R^2) had no answer even though the
     * same window reported 45 degrees and 411 uH. Refusing on the fallback threw
     * away three consecutive runs whose primary estimate was in hand.
     */
    m.z_mag_flat = true;

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_TRUE(s.result.valid_mask & ESP_FOC_MOTOR_ID_VALID_LS);
    TEST_ASSERT_INT_WITHIN(60, 500, (int)(s.result.ls_h * 1e6f));
}

TEST_CASE("motor_id biases the AC probe past its own amplitude",
          "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;

    /*
     * The bias existed to keep the current one-sided but was sized from R alone,
     * while the amplitude needed to reach the target current is sized from |Z|.
     * At 833 Hz the search therefore asked for 2.25 V against a 1.49 V bias, the
     * command reversed, and the clipped window published R lower at 833 Hz than at
     * 100 with |Z| under the DC resistance.
     */
    bind_step(&s, &m);
    m.clip_on_reversal = true;

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_EQUAL_INT(0, m.clipped_windows);
    TEST_ASSERT_INT_WITHIN(60, 500, (int)(s.result.ls_h * 1e6f));
    TEST_ASSERT_INT_WITHIN(200, 1900, (int)(s.result.r_loop_ohm * 1000.0f));

    /*
     * And it must not pay |Z|/R for that on an inductive machine: 40 mH at 100 Hz
     * is 25 ohm against 1.9, so a bias equal to the amplitude would ask this
     * bridge for over 3 A of DC to probe with 250 mA of AC.
     */
    bind_step(&s, &m);
    m.clip_on_reversal = true;
    m.l_h = 40e-3;

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_EQUAL_INT(0, m.clipped_windows);
    TEST_ASSERT_TRUE(m.dc_peak_ma < 1500);
}

TEST_CASE("motor_id skip_lag_cal still bypasses the step", "[espFoC][motor_id]")
{
    static esp_foc_motor_id_seq_t s;
    static mock_t m;

    bind_step(&s, &m);
    m.true_lag = 4.0;
    s.cfg.skip_lag_cal = true;
    s.cfg.lag_samples = 2u;

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_motor_id_seq_run(&s));
    TEST_ASSERT_EQUAL_UINT32(2u, s.result.lag_samples);
    TEST_ASSERT_EQUAL_INT32(0, m.step_calls);
}
