/*
 * Unit tests for the Gopinath flux observer + tracking PLL.
 */
#include <limits.h>
#include <math.h>

#include "unity.h"
#include "esp_err.h"
#include "espFoC/motor_control/esp_foc_observer_bemf.h"
#include "espFoC/motor_control/esp_foc_observer_flux.h"
#include "espFoC/utils/esp_foc_q16.h"

static esp_foc_observer_flux_config_t flux_cfg(void)
{
    esp_foc_observer_flux_config_t c = {
        .rs_ohm = 1.9f,
        .ls_h = 0.0005f,
        .psi_f_wb = 0.002564f,
        .ts_s = 1.0f / 20000.0f,
        .obs_bw_hz = 800.0f,
        .obs_zeta = 0.70f,
        .track_bw_hz = 20.0f,
        .track_zeta = 0.70f,
        .blend_hz = 20.0f,
        .psi_lock_frac = 0.50f,
        .w_max_rads = 2.0f * (float)M_PI * 800.0f,
        .lock_count = 80u,
    };
    return c;
}

static esp_foc_observer_t *flux_ok(esp_foc_observer_flux_t *store,
                                   const esp_foc_observer_flux_config_t *c)
{
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_observer_flux_init(store, c));
    return &store->iface;
}

static void plant_step(float rs, float ls, float psi, float ts,
                       float theta, float omega,
                       float va, float vb,
                       float *ia, float *ib)
{
    float ea = -psi * omega * sinf(theta);
    float eb = psi * omega * cosf(theta);
    float dia = (va - rs * (*ia) - ea) / ls;
    float dib = (vb - rs * (*ib) - eb) / ls;
    *ia += dia * ts;
    *ib += dib * ts;
}

static float wrap_pi_f(float d)
{
    while (d > (float)M_PI) {
        d -= 2.0f * (float)M_PI;
    }
    while (d < -(float)M_PI) {
        d += 2.0f * (float)M_PI;
    }
    return d;
}

static void wrap_theta(float *theta)
{
    if (*theta > (float)M_PI) {
        *theta -= 2.0f * (float)M_PI;
    }
    if (*theta <= -(float)M_PI) {
        *theta += 2.0f * (float)M_PI;
    }
}

static void apply_vq(float rs, float psi, float theta, float omega, float iq,
                     float *va, float *vb)
{
    float s = sinf(theta);
    float co = cosf(theta);
    float vq = rs * iq + psi * omega;
    *va = -vq * s;
    *vb = vq * co;
}

static void run_flux_at_hz(esp_foc_observer_t *o,
                           const esp_foc_observer_flux_config_t *c,
                           float fe_hz,
                           int steps,
                           float theta0,
                           float *theta_out)
{
    const float omega = 2.0f * (float)M_PI * fe_hz;
    float theta = theta0;
    float ia = 0.0f;
    float ib = 0.0f;
    const float iq = 0.30f;

    esp_foc_observer_set_theta(o, q16_from_float(theta));
    esp_foc_observer_set_omega(o, q16_from_float(omega));

    for (int i = 0; i < steps; i++) {
        float va;
        float vb;
        apply_vq(c->rs_ohm, c->psi_f_wb, theta, omega, iq, &va, &vb);
        plant_step(c->rs_ohm, c->ls_h, c->psi_f_wb, c->ts_s,
                   theta, omega, va, vb, &ia, &ib);
        esp_foc_observer_update(o,
                                q16_from_float(ia), q16_from_float(ib),
                                q16_from_float(va), q16_from_float(vb));
        theta += omega * c->ts_s;
        wrap_theta(&theta);
    }
    *theta_out = theta;
}

TEST_CASE("flux observer init rejects bad params", "[espFoC][observer][flux]")
{
    esp_foc_observer_flux_t o;
    esp_foc_observer_flux_config_t c = flux_cfg();
    c.rs_ohm = 0.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_observer_flux_init(&o, &c));

    c = flux_cfg();
    c.track_bw_hz = 900.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_observer_flux_init(&o, &c));

    c = flux_cfg();
    c.obs_bw_hz = 3000.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_observer_flux_init(&o, &c));

    c = flux_cfg();
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_observer_flux_init(&o, &c));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      esp_foc_observer_flux_set_bw(&o, 1200.0f, 1300.0f, 1.0f));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_observer_flux_set_bw(&o, 1200.0f, 700.0f, 1.0f));

    c = flux_cfg();
    c.track_zeta = 0.2f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_observer_flux_init(&o, &c));
}

TEST_CASE("flux observer tracks 50/120/300 Hz electrical", "[espFoC][observer][flux]")
{
    const float freqs[] = { 50.0f, 120.0f, 300.0f };
    for (unsigned k = 0; k < 3u; k++) {
        esp_foc_observer_flux_t store;
        esp_foc_observer_flux_config_t c = flux_cfg();
        if (freqs[k] >= 200.0f) {
            c.track_bw_hz = 40.0f;
        }
        esp_foc_observer_t *o = flux_ok(&store, &c);
        float theta = 0.4f;
        run_flux_at_hz(o, &c, freqs[k], 8000, 0.4f, &theta);

        float d = wrap_pi_f(q16_to_float(esp_foc_observer_get_theta(o)) - theta);
        TEST_ASSERT_TRUE_MESSAGE(fabsf(d) < 0.45f, "theta error");
        TEST_ASSERT_TRUE(esp_foc_observer_is_locked(o));

        float pa = q16_to_float(esp_foc_observer_get_psi_alpha(o));
        float pb = q16_to_float(esp_foc_observer_get_psi_beta(o));
        float pmag = sqrtf(pa * pa + pb * pb);
        TEST_ASSERT_FLOAT_WITHIN(0.0015f, c.psi_f_wb, pmag);

        float w_est = q16_to_float(esp_foc_observer_get_omega(o));
        TEST_ASSERT_FLOAT_WITHIN(80.0f, 2.0f * (float)M_PI * freqs[k], w_est);
    }
}

TEST_CASE("flux observer tracks rotor when Park voltage is misaligned",
          "[espFoC][observer][flux]")
{
    esp_foc_observer_flux_t flux_store;
    esp_foc_observer_bemf_t bemf_store;
    esp_foc_observer_flux_config_t fc = flux_cfg();
    esp_foc_observer_t *flux = flux_ok(&flux_store, &fc);

    esp_foc_observer_bemf_config_t bc = {
        .rs_ohm = fc.rs_ohm,
        .ls_h = fc.ls_h,
        .psi_f_wb = fc.psi_f_wb,
        .ts_s = fc.ts_s,
        .current_model_hz = 900.0f,
        .emf_lpf_hz = 800.0f,
        .pll_bw_hz = 50.0f,
        .pll_zeta = 0.70f,
        .e_lock_min_v = 0.08f,
        .w_max_rads = fc.w_max_rads,
        .lock_count = 80u,
        .extract = ESP_FOC_ANGLE_PLL,
    };
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_observer_bemf_init(&bemf_store, &bc));
    esp_foc_observer_t *bemf = &bemf_store.iface;

    const float omega = 2.0f * (float)M_PI * 80.0f;
    const float park_off = 0.90f;
    float theta = 0.0f;
    float ia = 0.0f;
    float ib = 0.0f;
    const float iq = 0.30f;

    esp_foc_observer_set_theta(flux, 0);
    esp_foc_observer_set_omega(flux, q16_from_float(omega));
    esp_foc_observer_set_theta(bemf, 0);
    esp_foc_observer_set_omega(bemf, q16_from_float(omega));

    for (int i = 0; i < 8000; i++) {
        float th_park = theta + park_off;
        float va;
        float vb;
        apply_vq(fc.rs_ohm, fc.psi_f_wb, th_park, omega, iq, &va, &vb);
        plant_step(fc.rs_ohm, fc.ls_h, fc.psi_f_wb, fc.ts_s,
                   theta, omega, va, vb, &ia, &ib);
        q16_t iaq = q16_from_float(ia);
        q16_t ibq = q16_from_float(ib);
        q16_t vaq = q16_from_float(va);
        q16_t vbq = q16_from_float(vb);
        esp_foc_observer_update(flux, iaq, ibq, vaq, vbq);
        esp_foc_observer_update(bemf, iaq, ibq, vaq, vbq);
        theta += omega * fc.ts_s;
        wrap_theta(&theta);
    }

    float d_flux = wrap_pi_f(q16_to_float(esp_foc_observer_get_theta(flux)) - theta);
    float d_park = wrap_pi_f(q16_to_float(esp_foc_observer_get_theta(flux)) -
                             (theta + park_off));
    TEST_ASSERT_TRUE(fabsf(d_flux) < 0.45f);
    TEST_ASSERT_TRUE(fabsf(d_park) > 0.40f);
    TEST_ASSERT_TRUE(esp_foc_observer_is_locked(flux));
}

TEST_CASE("flux observer blend bounds DC voltage offset", "[espFoC][observer][flux]")
{
    esp_foc_observer_flux_t store;
    esp_foc_observer_flux_config_t c = flux_cfg();
    esp_foc_observer_t *o = flux_ok(&store, &c);

    const float omega = 2.0f * (float)M_PI * 80.0f;
    float theta = 0.0f;
    float ia = 0.0f;
    float ib = 0.0f;
    const float iq = 0.25f;

    esp_foc_observer_set_theta(o, 0);
    esp_foc_observer_set_omega(o, q16_from_float(omega));

    for (int i = 0; i < 10000; i++) {
        float va;
        float vb;
        apply_vq(c.rs_ohm, c.psi_f_wb, theta, omega, iq, &va, &vb);
        va += 0.20f;
        plant_step(c.rs_ohm, c.ls_h, c.psi_f_wb, c.ts_s,
                   theta, omega, va, vb, &ia, &ib);
        esp_foc_observer_update(o,
                                q16_from_float(ia), q16_from_float(ib),
                                q16_from_float(va), q16_from_float(vb));
        theta += omega * c.ts_s;
        wrap_theta(&theta);
    }

    float pa = q16_to_float(esp_foc_observer_get_psi_alpha(o));
    float pb = q16_to_float(esp_foc_observer_get_psi_beta(o));
    float pmag = sqrtf(pa * pa + pb * pb);
    TEST_ASSERT_TRUE(pmag < 0.02f);
    TEST_ASSERT_TRUE(pmag > 0.0005f);
    float d = wrap_pi_f(q16_to_float(esp_foc_observer_get_theta(o)) - theta);
    TEST_ASSERT_TRUE(fabsf(d) < 0.80f);
}

TEST_CASE("flux observer tracks reverse rotation", "[espFoC][observer][flux]")
{
    esp_foc_observer_flux_t store;
    esp_foc_observer_flux_config_t c = flux_cfg();
    esp_foc_observer_t *o = flux_ok(&store, &c);

    const float omega = -2.0f * (float)M_PI * 120.0f;
    float theta = 0.0f;
    float ia = 0.0f;
    float ib = 0.0f;
    const float iq = -0.25f;

    esp_foc_observer_set_theta(o, 0);
    esp_foc_observer_set_omega(o, q16_from_float(omega));

    for (int i = 0; i < 8000; i++) {
        float va;
        float vb;
        apply_vq(c.rs_ohm, c.psi_f_wb, theta, omega, iq, &va, &vb);
        plant_step(c.rs_ohm, c.ls_h, c.psi_f_wb, c.ts_s,
                   theta, omega, va, vb, &ia, &ib);
        esp_foc_observer_update(o,
                                q16_from_float(ia), q16_from_float(ib),
                                q16_from_float(va), q16_from_float(vb));
        theta += omega * c.ts_s;
        wrap_theta(&theta);
    }

    float d = wrap_pi_f(q16_to_float(esp_foc_observer_get_theta(o)) - theta);
    TEST_ASSERT_TRUE(fabsf(d) < 0.45f);
    TEST_ASSERT_TRUE(esp_foc_observer_is_locked(o));
    TEST_ASSERT_TRUE(q16_to_float(esp_foc_observer_get_omega(o)) < 0.0f);
}

TEST_CASE("flux observer speed step settles without oscillation",
          "[espFoC][observer][flux]")
{
    esp_foc_observer_flux_t store;
    esp_foc_observer_flux_config_t c = flux_cfg();
    c.track_bw_hz = 30.0f;
    esp_foc_observer_t *o = flux_ok(&store, &c);

    const float ts = c.ts_s;
    const float w0 = 2.0f * (float)M_PI * 80.0f;
    const float w1 = 2.0f * (float)M_PI * 160.0f;
    float theta = 0.0f;
    float ia = 0.0f;
    float ib = 0.0f;
    const float iq = 0.25f;
    float omega = w0;

    esp_foc_observer_set_theta(o, 0);
    esp_foc_observer_set_omega(o, q16_from_float(w0));

    for (int i = 0; i < 4000; i++) {
        float va;
        float vb;
        apply_vq(c.rs_ohm, c.psi_f_wb, theta, omega, iq, &va, &vb);
        plant_step(c.rs_ohm, c.ls_h, c.psi_f_wb, ts, theta, omega, va, vb, &ia, &ib);
        esp_foc_observer_update(o,
                                q16_from_float(ia), q16_from_float(ib),
                                q16_from_float(va), q16_from_float(vb));
        theta += omega * ts;
        wrap_theta(&theta);
    }

    omega = w1;
    q16_t w_prev = esp_foc_observer_get_omega(o);
    q16_t max_dw = 0;
    for (int i = 0; i < 8000; i++) {
        float va;
        float vb;
        apply_vq(c.rs_ohm, c.psi_f_wb, theta, omega, iq, &va, &vb);
        plant_step(c.rs_ohm, c.ls_h, c.psi_f_wb, ts, theta, omega, va, vb, &ia, &ib);
        esp_foc_observer_update(o,
                                q16_from_float(ia), q16_from_float(ib),
                                q16_from_float(va), q16_from_float(vb));
        q16_t w_now = esp_foc_observer_get_omega(o);
        if (i > 6000) {
            q16_t dw = q16_sub(w_now, w_prev);
            if (dw < 0) {
                dw = q16_neg(dw);
            }
            if (dw > max_dw) {
                max_dw = dw;
            }
        }
        w_prev = w_now;
        theta += omega * ts;
        wrap_theta(&theta);
    }

    TEST_ASSERT_FLOAT_WITHIN(80.0f, w1, q16_to_float(esp_foc_observer_get_omega(o)));
    TEST_ASSERT_TRUE(max_dw < q16_from_float(8.0f));
    TEST_ASSERT_TRUE(esp_foc_observer_is_locked(o));
}

TEST_CASE("flux observer numerical stability over 2 s", "[espFoC][observer][flux]")
{
    esp_foc_observer_flux_t store;
    esp_foc_observer_flux_config_t c = flux_cfg();
    esp_foc_observer_t *o = flux_ok(&store, &c);

    const float omega = 2.0f * (float)M_PI * 50.0f;
    float theta = 0.0f;
    float ia = 0.0f;
    float ib = 0.0f;
    const float iq = 0.20f;

    esp_foc_observer_set_theta(o, 0);
    esp_foc_observer_set_omega(o, q16_from_float(omega));

    for (int i = 0; i < 40000; i++) {
        float va;
        float vb;
        apply_vq(c.rs_ohm, c.psi_f_wb, theta, omega, iq, &va, &vb);
        plant_step(c.rs_ohm, c.ls_h, c.psi_f_wb, c.ts_s,
                   theta, omega, va, vb, &ia, &ib);
        esp_foc_observer_update(o,
                                q16_from_float(ia), q16_from_float(ib),
                                q16_from_float(va), q16_from_float(vb));
        theta += omega * c.ts_s;
        wrap_theta(&theta);
    }

    q16_t pa = esp_foc_observer_get_psi_alpha(o);
    q16_t pb = esp_foc_observer_get_psi_beta(o);
    q16_t ea = esp_foc_observer_get_e_alpha(o);
    q16_t eb = esp_foc_observer_get_e_beta(o);
    TEST_ASSERT_TRUE(pa > (q16_t)INT32_MIN + 1000);
    TEST_ASSERT_TRUE(pb < (q16_t)INT32_MAX - 1000);
    TEST_ASSERT_TRUE(ea > (q16_t)INT32_MIN + 1000);
    TEST_ASSERT_TRUE(eb < (q16_t)INT32_MAX - 1000);
    TEST_ASSERT_TRUE(esp_foc_observer_is_locked(o));
    float d = wrap_pi_f(q16_to_float(esp_foc_observer_get_theta(o)) - theta);
    TEST_ASSERT_TRUE(fabsf(d) < 0.45f);
}

TEST_CASE("flux PLL 1 kHz from init tracks 50 Hz and an Iq-like accel",
          "[espFoC][observer][flux]")
{
    esp_foc_observer_flux_t store;
    esp_foc_observer_flux_config_t c = flux_cfg();
    c.obs_bw_hz = 1800.0f;
    c.track_bw_hz = 1000.0f;
    esp_foc_observer_t *o = flux_ok(&store, &c);

    const float ts = c.ts_s;
    const float w1 = 2.0f * (float)M_PI * 50.0f;
    float theta = 0.0f;
    float ia = 0.0f;
    float ib = 0.0f;
    float omega = 0.0f;
    float iq = 0.0f;
    const int ramp_n = (int)(0.02f / ts);

    esp_foc_observer_set_theta(o, 0);
    esp_foc_observer_set_omega(o, 0);

    q16_t w_prev = 0;
    q16_t max_dw = 0;
    float w_hold = 0.0f;
    bool locked_hold = false;
    for (int i = 0; i < 8000; i++) {
        if (i < ramp_n) {
            omega = w1 * (float)i / (float)ramp_n;
            iq = 0.35f * (float)i / (float)ramp_n;
        } else if (i < ramp_n + 4000) {
            omega = w1;
            iq = 0.35f;
        } else {
            /* Aggressive Iq dump: current and speed fall together. */
            int k = i - (ramp_n + 4000);
            const int down_n = (int)(0.008f / ts);
            float s = (k < down_n) ? (1.0f - (float)k / (float)down_n) : 0.0f;
            omega = w1 * s;
            iq = 0.35f * s;
        }
        float va;
        float vb;
        apply_vq(c.rs_ohm, c.psi_f_wb, theta, omega, iq, &va, &vb);
        plant_step(c.rs_ohm, c.ls_h, c.psi_f_wb, ts, theta, omega, va, vb, &ia, &ib);
        esp_foc_observer_update(o,
                                q16_from_float(ia), q16_from_float(ib),
                                q16_from_float(va), q16_from_float(vb));
        q16_t w_now = esp_foc_observer_get_omega(o);
        if (i > 2000 && i < ramp_n + 4000) {
            q16_t dw = q16_sub(w_now, w_prev);
            if (dw < 0) {
                dw = q16_neg(dw);
            }
            if (dw > max_dw) {
                max_dw = dw;
            }
        }
        w_prev = w_now;
        theta += omega * ts;
        wrap_theta(&theta);
        if (i == ramp_n + 3000) {
            w_hold = q16_to_float(w_now);
            locked_hold = esp_foc_observer_is_locked(o);
        }
    }

    TEST_ASSERT_TRUE(locked_hold);
    TEST_ASSERT_FLOAT_WITHIN(80.0f, w1, w_hold);
    TEST_ASSERT_TRUE(max_dw < q16_from_float(80.0f));
    TEST_ASSERT_FLOAT_WITHIN(80.0f, 0.0f, q16_to_float(esp_foc_observer_get_omega(o)));
}

TEST_CASE("flux PLL settle freezes omega then tracks without snapping theta",
          "[espFoC][observer][flux]")
{
    esp_foc_observer_flux_t store;
    esp_foc_observer_flux_config_t c = flux_cfg();
    c.track_bw_hz = 80.0f;
    c.pll_settle_ms = 10.0f;
    esp_foc_observer_t *o = flux_ok(&store, &c);

    const float fe = 144.0f;
    const float omega = 2.0f * (float)M_PI * fe;
    const q16_t w_seed = q16_from_float(omega);
    const int settle_n = (int)(c.pll_settle_ms * 0.001f / c.ts_s + 0.5f);
    float theta = 1.00f;
    float ia = 0.0f;
    float ib = 0.0f;
    const float iq = 0.22f;

    esp_foc_observer_set_theta(o, q16_from_float(theta));
    esp_foc_observer_set_omega(o, w_seed);

    for (int i = 0; i <= settle_n; i++) {
        float va;
        float vb;
        apply_vq(c.rs_ohm, c.psi_f_wb, theta, omega, iq, &va, &vb);
        plant_step(c.rs_ohm, c.ls_h, c.psi_f_wb, c.ts_s,
                   theta, omega, va, vb, &ia, &ib);
        esp_foc_observer_update(o,
                                q16_from_float(ia), q16_from_float(ib),
                                q16_from_float(va), q16_from_float(vb));
        if (i >= 1) {
            TEST_ASSERT_INT32_WITHIN(64, w_seed, esp_foc_observer_get_omega(o));
        }
        theta += omega * c.ts_s;
        wrap_theta(&theta);
    }

    TEST_ASSERT_TRUE(fabsf(q16_to_float(esp_foc_observer_get_phase_err(o))) <= 1.0f);

    float w_max = omega;
    float w_min = omega;
    for (int i = 0; i < 4000; i++) {
        float va;
        float vb;
        apply_vq(c.rs_ohm, c.psi_f_wb, theta, omega, iq, &va, &vb);
        plant_step(c.rs_ohm, c.ls_h, c.psi_f_wb, c.ts_s,
                   theta, omega, va, vb, &ia, &ib);
        esp_foc_observer_update(o,
                                q16_from_float(ia), q16_from_float(ib),
                                q16_from_float(va), q16_from_float(vb));
        float w = q16_to_float(esp_foc_observer_get_omega(o));
        if (w > w_max) {
            w_max = w;
        }
        if (w < w_min) {
            w_min = w;
        }
        theta += omega * c.ts_s;
        wrap_theta(&theta);
    }

    TEST_ASSERT_TRUE(esp_foc_observer_is_locked(o));
    TEST_ASSERT_FLOAT_WITHIN(80.0f, omega, q16_to_float(esp_foc_observer_get_omega(o)));
    TEST_ASSERT_TRUE_MESSAGE((w_max - w_min) < 400.0f, "PLL did not hunt after settle");
    float d = wrap_pi_f(q16_to_float(esp_foc_observer_get_theta(o)) - theta);
    TEST_ASSERT_TRUE(fabsf(d) < 0.45f);
    TEST_ASSERT_TRUE(fabsf(q16_to_float(esp_foc_observer_get_phase_err(o))) < 0.35f);
}

static float psi_mag_of(esp_foc_observer_t *o)
{
    float pa = q16_to_float(esp_foc_observer_get_psi_alpha(o));
    float pb = q16_to_float(esp_foc_observer_get_psi_beta(o));
    return sqrtf(pa * pa + pb * pb);
}

/* The bench failure in a jar. A DC offset on the plant's own terminals does
 * not drift the estimate: the plant answers with a DC current and the Rs·i
 * term cancels it, which is what the blend test already shows. Drift needs the
 * voltage the observer is told about to disagree with the voltage the motor
 * saw — dead time, saturation, duty-to-volts model error. Then the integrand
 * carries an uncancelled DC, ψ̂ climbs to v_err/λ, and because the PLL error is
 * normalised by |ψ̂| the climb takes the loop gain with it. */
static esp_foc_observer_flux_config_t flux_drift_cfg(void)
{
    esp_foc_observer_flux_config_t c = flux_cfg();
    c.blend_hz = 1.0f;
    return c;
}

typedef struct {
    float theta;
    float ia;
    float ib;
} drift_state_t;

#define DRIFT_OMEGA (2.0f * (float)M_PI * 80.0f)

static void drift_start(esp_foc_observer_t *o, drift_state_t *st)
{
    st->theta = 0.0f;
    st->ia = 0.0f;
    st->ib = 0.0f;
    esp_foc_observer_set_theta(o, 0);
    esp_foc_observer_set_omega(o, q16_from_float(DRIFT_OMEGA));
}

static void run_drift(esp_foc_observer_t *o,
                      const esp_foc_observer_flux_config_t *c,
                      drift_state_t *st, float v_err, int n)
{
    for (int i = 0; i < n; i++) {
        float va;
        float vb;
        apply_vq(c->rs_ohm, c->psi_f_wb, st->theta, DRIFT_OMEGA, 0.25f, &va, &vb);
        plant_step(c->rs_ohm, c->ls_h, c->psi_f_wb, c->ts_s,
                   st->theta, DRIFT_OMEGA, va, vb, &st->ia, &st->ib);
        esp_foc_observer_update(o,
                                q16_from_float(st->ia), q16_from_float(st->ib),
                                q16_from_float(va + v_err), q16_from_float(vb));
        st->theta += DRIFT_OMEGA * c->ts_s;
        wrap_theta(&st->theta);
    }
}

static float drift_peak(esp_foc_observer_t *o,
                        const esp_foc_observer_flux_config_t *c,
                        drift_state_t *st, float v_err, int n)
{
    float peak = 0.0f;
    for (int i = 0; i < n; i++) {
        run_drift(o, c, st, v_err, 1);
        float m = psi_mag_of(o);
        if (m > peak) {
            peak = m;
        }
    }
    return peak;
}

TEST_CASE("flux observer ceiling bounds a drifting integrator",
          "[espFoC][observer][flux]")
{
    esp_foc_observer_flux_t store;
    esp_foc_observer_flux_config_t c = flux_drift_cfg();
    c.psi_max_frac = 4.0f;
    esp_foc_observer_t *o = flux_ok(&store, &c);
    drift_state_t st;

    drift_start(o, &st);
    run_drift(o, &c, &st, 0.50f, 20000);

    /* The ceiling is enforced on an L2 approximation good to ~4%, so the true
     * magnitude lands on it within that. */
    const float ceil_wb = c.psi_max_frac * c.psi_f_wb;
    float pmag = psi_mag_of(o);
    TEST_ASSERT_TRUE(esp_foc_observer_flux_get_psi_clamp_count(&store) > 0u);
    TEST_ASSERT_TRUE(pmag <= ceil_wb * 1.05f);
    TEST_ASSERT_TRUE(pmag >= ceil_wb * 0.92f);

    /* Final value is the envelope, not the magnitude. ψ̂ here is a DC drift
     * plus the rotating magnet term, so |ψ̂| traces an off-centre circle and
     * swings once per electrical revolution no matter what the ceiling does.
     * What has to converge is the peak: bounded and no longer growing. */
    float peak_early = drift_peak(o, &c, &st, 0.50f, 4000);
    (void)run_drift(o, &c, &st, 0.50f, 20000);
    float peak_late = drift_peak(o, &c, &st, 0.50f, 4000);
    TEST_ASSERT_TRUE(peak_early <= ceil_wb * 1.05f);
    TEST_ASSERT_TRUE(peak_late <= peak_early * 1.05f);
}

/* A caller that never heard of the ceiling has to get it anyway: leaving the
 * field at zero is what four of the five apps did, so "0 disables" meant the
 * guard was off on every bench that needed it. */
TEST_CASE("flux observer ceiling defaults on when the caller leaves it unset",
          "[espFoC][observer][flux]")
{
    esp_foc_observer_flux_t store;
    esp_foc_observer_flux_config_t c = flux_drift_cfg();
    c.psi_max_frac = 0.0f;
    esp_foc_observer_t *o = flux_ok(&store, &c);
    drift_state_t st;

    drift_start(o, &st);
    run_drift(o, &c, &st, 0.50f, 20000);

    const float ceil_wb = ESP_FOC_OBSERVER_PSI_MAX_FRAC_DEFAULT * c.psi_f_wb;
    TEST_ASSERT_TRUE(esp_foc_observer_flux_get_psi_clamp_count(&store) > 0u);
    TEST_ASSERT_TRUE(psi_mag_of(o) <= ceil_wb * 1.05f);
}

TEST_CASE("flux observer ceiling opted out leaves the drift unbounded",
          "[espFoC][observer][flux]")
{
    esp_foc_observer_flux_t store;
    esp_foc_observer_flux_config_t c = flux_drift_cfg();
    c.psi_max_frac = -1.0f;
    esp_foc_observer_t *o = flux_ok(&store, &c);
    drift_state_t st;

    drift_start(o, &st);
    run_drift(o, &c, &st, 0.50f, 20000);

    TEST_ASSERT_EQUAL_UINT32(0u, esp_foc_observer_flux_get_psi_clamp_count(&store));
    TEST_ASSERT_TRUE(psi_mag_of(o) > 6.0f * c.psi_f_wb);
}

TEST_CASE("flux observer ceiling is inert on a healthy track",
          "[espFoC][observer][flux]")
{
    esp_foc_observer_flux_t store;
    esp_foc_observer_flux_config_t c = flux_cfg();
    c.psi_max_frac = 4.0f;
    esp_foc_observer_t *o = flux_ok(&store, &c);

    const float omega = 2.0f * (float)M_PI * 120.0f;
    float theta = 0.0f;
    float ia = 0.0f;
    float ib = 0.0f;

    esp_foc_observer_set_theta(o, 0);
    esp_foc_observer_set_omega(o, q16_from_float(omega));

    for (int i = 0; i < 8000; i++) {
        float va;
        float vb;
        apply_vq(c.rs_ohm, c.psi_f_wb, theta, omega, 0.25f, &va, &vb);
        plant_step(c.rs_ohm, c.ls_h, c.psi_f_wb, c.ts_s,
                   theta, omega, va, vb, &ia, &ib);
        esp_foc_observer_update(o,
                                q16_from_float(ia), q16_from_float(ib),
                                q16_from_float(va), q16_from_float(vb));
        theta += omega * c.ts_s;
        wrap_theta(&theta);
    }

    TEST_ASSERT_EQUAL_UINT32(0u, esp_foc_observer_flux_get_psi_clamp_count(&store));
    TEST_ASSERT_TRUE(esp_foc_observer_is_locked(o));
    float d = wrap_pi_f(q16_to_float(esp_foc_observer_get_theta(o)) - theta);
    TEST_ASSERT_TRUE(fabsf(d) < 0.45f);
}

TEST_CASE("flux observer ceiling keeps the estimate direction",
          "[espFoC][observer][flux]")
{
    esp_foc_observer_flux_t s_on;
    esp_foc_observer_flux_t s_off;
    esp_foc_observer_flux_config_t c_on = flux_drift_cfg();
    esp_foc_observer_flux_config_t c_off = flux_drift_cfg();
    c_on.psi_max_frac = 4.0f;
    c_off.psi_max_frac = -1.0f;
    esp_foc_observer_t *on = flux_ok(&s_on, &c_on);
    esp_foc_observer_t *off = flux_ok(&s_off, &c_off);

    const float omega = 2.0f * (float)M_PI * 80.0f;
    float theta = 0.0f;
    float ia = 0.0f;
    float ib = 0.0f;
    bool checked = false;

    esp_foc_observer_set_theta(on, 0);
    esp_foc_observer_set_omega(on, q16_from_float(omega));
    esp_foc_observer_set_theta(off, 0);
    esp_foc_observer_set_omega(off, q16_from_float(omega));

    /* Same inputs, so up to the first clamp both hold the same state: the
     * rescale may only take magnitude, never turn the vector. */
    for (int i = 0; i < 20000 && !checked; i++) {
        float va;
        float vb;
        apply_vq(c_on.rs_ohm, c_on.psi_f_wb, theta, omega, 0.25f, &va, &vb);
        plant_step(c_on.rs_ohm, c_on.ls_h, c_on.psi_f_wb, c_on.ts_s,
                   theta, omega, va, vb, &ia, &ib);
        const q16_t qia = q16_from_float(ia);
        const q16_t qib = q16_from_float(ib);
        const q16_t qva = q16_from_float(va + 0.50f);
        const q16_t qvb = q16_from_float(vb);
        esp_foc_observer_update(on, qia, qib, qva, qvb);
        esp_foc_observer_update(off, qia, qib, qva, qvb);
        if (esp_foc_observer_flux_get_psi_clamp_count(&s_on) > 0u) {
            float aon = atan2f(q16_to_float(esp_foc_observer_get_psi_beta(on)),
                               q16_to_float(esp_foc_observer_get_psi_alpha(on)));
            float aoff = atan2f(q16_to_float(esp_foc_observer_get_psi_beta(off)),
                                q16_to_float(esp_foc_observer_get_psi_alpha(off)));
            TEST_ASSERT_TRUE(fabsf(wrap_pi_f(aon - aoff)) < 0.02f);
            checked = true;
        }
        theta += omega * c_on.ts_s;
        wrap_theta(&theta);
    }

    TEST_ASSERT_TRUE_MESSAGE(checked, "ceiling never engaged");
}
