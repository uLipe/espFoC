/*
 * Unit tests for esp_foc_mech_id (two-level K probe).
 */
#include <math.h>
#include <string.h>

#include "unity.h"
#include "esp_err.h"
#include "esp_foc_mech_id.h"
#include "espFoC/utils/esp_foc_q16.h"

#define TWO_PI 6.28318530718f

typedef struct {
    float k;
    float b;
    float t_load;
    float we;
    q16_t id;
    q16_t iq;
    q16_t id_entry;
    q16_t iq_entry;
    int n_set;
    int n_hi;
    float iq_hi_a;
    bool fault;
    int n_sleep;
    int fault_after;
    float noise_rads;
    uint32_t lcg;
    float we_peak;
} mock_t;

/* Sum of four uniforms: near-Gaussian, unit variance, deterministic. */
static float mock_gauss(mock_t *m)
{
    float s = 0.0f;
    for (int i = 0; i < 4; i++) {
        m->lcg = m->lcg * 1664525u + 1013904223u;
        s += (float)(m->lcg >> 8) / 8388608.0f - 1.0f;
    }
    return s * 0.8660254f;
}

static void mock_set_idq(void *ctx, q16_t id, q16_t iq)
{
    mock_t *m = ctx;
    m->id = id;
    m->iq = iq;
    m->n_set++;
    if (m->iq_hi_a > 0.0f &&
        fabsf(fabsf(q16_to_float(iq)) - m->iq_hi_a) < 1.0e-3f) {
        m->n_hi++;
    }
}

static void mock_get_idq(void *ctx, q16_t *id, q16_t *iq)
{
    const mock_t *m = ctx;
    *id = m->id_entry;
    *iq = m->iq_entry;
}

static q16_t mock_get_we(void *ctx)
{
    mock_t *m = ctx;
    float w = m->we;
    if (m->noise_rads > 0.0f) {
        w += m->noise_rads * mock_gauss(m);
    }
    return q16_from_float(w);
}

static bool mock_faulted(void *ctx)
{
    return ((mock_t *)ctx)->fault;
}

static void mock_sleep(void *ctx, uint32_t ms)
{
    mock_t *m = ctx;
    float iq = q16_to_float(m->iq);
    for (uint32_t i = 0; i < ms; i++) {
        float a = m->k * iq - m->b * m->we - m->t_load;
        m->we += a * 0.001f;
        if (fabsf(m->we) > m->we_peak) {
            m->we_peak = fabsf(m->we);
        }
    }
    m->n_sleep++;
    if (m->fault_after > 0 && m->n_sleep >= m->fault_after) {
        m->fault = true;
    }
}

static void bind(esp_foc_mech_id_ops_t *ops, mock_t *m)
{
    memset(ops, 0, sizeof(*ops));
    ops->ctx = m;
    ops->set_idq = mock_set_idq;
    ops->get_idq = mock_get_idq;
    ops->get_omega_e = mock_get_we;
    ops->faulted = mock_faulted;
    ops->sleep_ms = mock_sleep;
}

static void cfg_bench(esp_foc_mech_id_config_t *cfg)
{
    esp_foc_mech_id_default_config(cfg);
    cfg->i_max_a = 1.0f;
    cfg->win_ms = 50;
    cfg->settle_ms = 5;
    cfg->we_entry_min_hz = 10.0f;
    cfg->we_max_hz = 400.0f;
    cfg->pole_pairs = 13;
    cfg->psi_f_wb = 0.002564f;
}

TEST_CASE("mech_id rejects incomplete wiring", "[espFoC][mech_id]")
{
    esp_foc_mech_id_config_t cfg;
    esp_foc_mech_id_result_t out;
    cfg_bench(&cfg);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      esp_foc_mech_id_run(NULL, &cfg, &out));
    TEST_ASSERT_EQUAL(0, out.valid_mask);

    esp_foc_mech_id_ops_t ops;
    memset(&ops, 0, sizeof(ops));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      esp_foc_mech_id_run(&ops, &cfg, &out));

    cfg.i_max_a = 0.0f;
    mock_t m;
    memset(&m, 0, sizeof(m));
    bind(&ops, &m);
    cfg_bench(&cfg);
    cfg.i_max_a = 0.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      esp_foc_mech_id_run(&ops, &cfg, &out));
}

TEST_CASE("mech_id recovers K on a frictionless plant", "[espFoC][mech_id]")
{
    mock_t m;
    memset(&m, 0, sizeof(m));
    m.k = 3000.0f;
    m.we = 80.0f * TWO_PI;
    m.iq_hi_a = 0.50f;

    esp_foc_mech_id_ops_t ops;
    bind(&ops, &m);
    esp_foc_mech_id_config_t cfg;
    cfg_bench(&cfg);
    esp_foc_mech_id_result_t out;

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_mech_id_run(&ops, &cfg, &out));
    TEST_ASSERT_FLOAT_WITHIN(60.0f, 3000.0f, out.k_rad_s2_per_a);
    TEST_ASSERT_TRUE(out.valid_mask & ESP_FOC_MECH_ID_VALID_K);
    TEST_ASSERT_TRUE(out.valid_mask & ESP_FOC_MECH_ID_VALID_J);
    float j_want = 1.5f * 13.0f * 13.0f * 0.002564f / out.k_rad_s2_per_a;
    TEST_ASSERT_FLOAT_WITHIN(1.0e-8f, j_want, out.j_kgm2);
    TEST_ASSERT_EQUAL(0, m.iq);
}

TEST_CASE("mech_id differential cancels a constant load", "[espFoC][mech_id]")
{
    mock_t m;
    memset(&m, 0, sizeof(m));
    m.k = 3000.0f;
    m.t_load = 400.0f;
    m.we = 80.0f * TWO_PI;
    m.iq_hi_a = 0.50f;

    esp_foc_mech_id_ops_t ops;
    bind(&ops, &m);
    esp_foc_mech_id_config_t cfg;
    cfg_bench(&cfg);
    esp_foc_mech_id_result_t out;

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_mech_id_run(&ops, &cfg, &out));

    float k_lo = out.accel[0] / out.iq_a[0];
    TEST_ASSERT_TRUE(fabsf(k_lo - 3000.0f) > 500.0f);
    TEST_ASSERT_FLOAT_WITHIN(60.0f, 3000.0f, out.k_rad_s2_per_a);
    TEST_ASSERT_EQUAL(ESP_FOC_MECH_ID_VALID_K | ESP_FOC_MECH_ID_VALID_J,
                      out.valid_mask);
}

TEST_CASE("mech_id refuses to start below the speed band", "[espFoC][mech_id]")
{
    mock_t m;
    memset(&m, 0, sizeof(m));
    m.k = 3000.0f;
    m.we = 5.0f * TWO_PI;

    esp_foc_mech_id_ops_t ops;
    bind(&ops, &m);
    esp_foc_mech_id_config_t cfg;
    cfg_bench(&cfg);
    cfg.we_entry_min_hz = 40.0f;
    esp_foc_mech_id_result_t out;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      esp_foc_mech_id_run(&ops, &cfg, &out));
    TEST_ASSERT_EQUAL(0, out.valid_mask);
    TEST_ASSERT_EQUAL(0, m.n_hi);
}

TEST_CASE("mech_id refuses K outside the configured gate", "[espFoC][mech_id]")
{
    mock_t m;
    memset(&m, 0, sizeof(m));
    m.k = 100.0f;
    m.we = 80.0f * TWO_PI;
    m.iq_hi_a = 0.50f;

    esp_foc_mech_id_ops_t ops;
    bind(&ops, &m);
    esp_foc_mech_id_config_t cfg;
    cfg_bench(&cfg);
    esp_foc_mech_id_result_t out;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_RESPONSE,
                      esp_foc_mech_id_run(&ops, &cfg, &out));
    TEST_ASSERT_EQUAL(0, out.valid_mask);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, 0.0f, out.k_rad_s2_per_a);
    const float diq = out.iq_a[1] - out.iq_a[0];
    TEST_ASSERT_TRUE(diq > 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(15.0f, m.k, (out.accel[1] - out.accel[0]) / diq);
}

TEST_CASE("mech_id refuses rather than cross we_max when the cap is near",
          "[espFoC][mech_id]")
{
    mock_t m;
    memset(&m, 0, sizeof(m));
    m.k = 3000.0f;
    m.we = 70.0f * TWO_PI;
    m.iq_hi_a = 0.50f;

    esp_foc_mech_id_ops_t ops;
    bind(&ops, &m);
    esp_foc_mech_id_config_t cfg;
    cfg_bench(&cfg);
    cfg.win_ms = 120;
    cfg.settle_ms = 10;
    cfg.we_max_hz = 80.0f;
    esp_foc_mech_id_result_t out;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      esp_foc_mech_id_run(&ops, &cfg, &out));
    TEST_ASSERT_EQUAL(0, out.valid_mask);
    TEST_ASSERT_TRUE(m.we_peak < 80.0f * TWO_PI);
}

TEST_CASE("mech_id restores entry iq after a mid-window fault",
          "[espFoC][mech_id]")
{
    mock_t m;
    memset(&m, 0, sizeof(m));
    m.k = 3000.0f;
    m.we = 80.0f * TWO_PI;
    m.iq_entry = q16_from_float(0.22f);
    m.id_entry = 0;
    m.iq = m.iq_entry;
    m.fault_after = 2;
    m.iq_hi_a = 0.50f;

    esp_foc_mech_id_ops_t ops;
    bind(&ops, &m);
    esp_foc_mech_id_config_t cfg;
    cfg_bench(&cfg);
    esp_foc_mech_id_result_t out;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      esp_foc_mech_id_run(&ops, &cfg, &out));
    TEST_ASSERT_EQUAL(0, out.valid_mask);
    TEST_ASSERT_INT32_WITHIN(8, m.iq_entry, m.iq);
}

TEST_CASE("mech_id keeps the K solve in float", "[espFoC][mech_id]")
{
    mock_t m;
    memset(&m, 0, sizeof(m));
    m.k = 20000.0f;
    m.we = 80.0f * TWO_PI;
    m.iq_hi_a = 0.18f;

    esp_foc_mech_id_ops_t ops;
    bind(&ops, &m);
    esp_foc_mech_id_config_t cfg;
    cfg_bench(&cfg);
    cfg.iq_lo_frac = 0.15f;
    cfg.iq_hi_frac = 0.18f;
    cfg.k_max = 40000.0f;
    esp_foc_mech_id_result_t out;

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_mech_id_run(&ops, &cfg, &out));
    TEST_ASSERT_TRUE(isfinite(out.k_rad_s2_per_a));
    TEST_ASSERT_TRUE(isfinite(out.j_kgm2));
    TEST_ASSERT_FLOAT_WITHIN(800.0f, 20000.0f, out.k_rad_s2_per_a);
    TEST_ASSERT_TRUE(out.valid_mask & ESP_FOC_MECH_ID_VALID_K);
}

/*
 * Bench numbers from 2026-10-01: sigma(w) ~26 rad/s, a drag that a 0.1 A
 * level barely overcomes at ~180 Hz elec, K ~5000. Two endpoint reads gave
 * K anywhere in 846..5900 across boots; the slope fit has to hold it to a
 * few percent on every seed.
 */
static void cfg_default_bench(esp_foc_mech_id_config_t *cfg)
{
    esp_foc_mech_id_default_config(cfg);
    cfg->i_max_a = 0.70f;
    cfg->pole_pairs = 13;
    cfg->psi_f_wb = 0.002564f;
}

TEST_CASE("mech_id slope fit holds K under sensor noise", "[espFoC][mech_id]")
{
    for (uint32_t seed = 1; seed <= 8; seed++) {
        mock_t m;
        memset(&m, 0, sizeof(m));
        m.k = 5000.0f;
        m.t_load = 540.0f;
        m.we = 180.0f * TWO_PI;
        m.noise_rads = 26.0f;
        m.lcg = seed * 2654435761u;

        esp_foc_mech_id_ops_t ops;
        bind(&ops, &m);
        esp_foc_mech_id_config_t cfg;
        cfg_default_bench(&cfg);
        esp_foc_mech_id_result_t out;

        TEST_ASSERT_EQUAL(ESP_OK, esp_foc_mech_id_run(&ops, &cfg, &out));
        TEST_ASSERT_FLOAT_WITHIN(0.15f * 5000.0f, 5000.0f, out.k_rad_s2_per_a);
    }
}

TEST_CASE("mech_id tolerates viscous drag between levels", "[espFoC][mech_id]")
{
    mock_t m;
    memset(&m, 0, sizeof(m));
    m.k = 5000.0f;
    m.b = 0.40f;
    m.we = 180.0f * TWO_PI;

    esp_foc_mech_id_ops_t ops;
    bind(&ops, &m);
    esp_foc_mech_id_config_t cfg;
    cfg_default_bench(&cfg);
    esp_foc_mech_id_result_t out;

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_mech_id_run(&ops, &cfg, &out));
    TEST_ASSERT_FLOAT_WITHIN(0.10f * 5000.0f, 5000.0f, out.k_rad_s2_per_a);
}

/*
 * 2026-10-01, Park angle corrected: the shaft went 96 -> 480 Hz elec inside
 * one fixed 0.35 A window and the run hit the cap. The same config has to
 * hold both ends of the K gate without crossing we_max.
 */
TEST_CASE("mech_id spans the K gate inside the speed headroom",
          "[espFoC][mech_id]")
{
    static const float ks[] = { 1500.0f, 5000.0f, 20000.0f, 36000.0f };
    /* sigma(K) at sigma(w)=3.4 rad/s on full windows: ~21 rad/s2/A. */
    const float k_floor = 150.0f;
    for (unsigned i = 0; i < sizeof(ks) / sizeof(ks[0]); i++) {
        mock_t m;
        memset(&m, 0, sizeof(m));
        m.k = ks[i];
        m.t_load = 0.1f * ks[i];
        m.we = 96.0f * TWO_PI;
        m.noise_rads = 3.4f;
        m.lcg = 12345u + i;

        esp_foc_mech_id_ops_t ops;
        bind(&ops, &m);
        esp_foc_mech_id_config_t cfg;
        cfg_default_bench(&cfg);
        esp_foc_mech_id_result_t out;

        TEST_ASSERT_EQUAL(ESP_OK, esp_foc_mech_id_run(&ops, &cfg, &out));
        TEST_ASSERT_FLOAT_WITHIN(fmaxf(0.05f * ks[i], k_floor), ks[i],
                                 out.k_rad_s2_per_a);
        TEST_ASSERT_TRUE(m.we_peak < cfg.we_max_hz * TWO_PI);
    }
}

TEST_CASE("mech_id refuses a window too short to fit", "[espFoC][mech_id]")
{
    mock_t m;
    memset(&m, 0, sizeof(m));
    m.k = 5000.0f;
    m.we = 180.0f * TWO_PI;

    esp_foc_mech_id_ops_t ops;
    bind(&ops, &m);
    esp_foc_mech_id_config_t cfg;
    cfg_default_bench(&cfg);
    cfg.win_ms = 6;
    cfg.sample_ms = 2;
    esp_foc_mech_id_result_t out;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_mech_id_run(&ops, &cfg, &out));
    cfg.win_ms = 250;
    cfg.sample_ms = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_mech_id_run(&ops, &cfg, &out));
}
