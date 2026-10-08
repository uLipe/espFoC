/*
 * Unit tests for the voltage-model BEMF observer (atan2 / PLL extractors).
 */
#include <limits.h>
#include <math.h>

#include "unity.h"
#include "esp_err.h"
#include "espFoC/motor_control/esp_foc_observer_bemf.h"
#include "espFoC/utils/esp_foc_angle.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/esp_foc_trig.h"

static esp_foc_observer_bemf_config_t base_cfg(esp_foc_angle_extract_t extract)
{
    esp_foc_observer_bemf_config_t c = {
        .rs_ohm = 1.9f,
        .ls_h = 0.0005f,
        .psi_f_wb = 0.002564f,
        .ts_s = 1.0f / 20000.0f,
        .current_model_hz = 900.0f,
        .emf_lpf_hz = 800.0f,
        .pll_bw_hz = 2000.0f,
        .pll_zeta = 0.70f,
        .e_lock_min_v = 0.08f,
        .lock_count = 80u,
        .extract = extract,
    };
    return c;
}

static esp_foc_observer_t *bemf_ok(esp_foc_observer_bemf_t *store,
                                   const esp_foc_observer_bemf_config_t *c)
{
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_observer_bemf_init(store, c));
    return &store->iface;
}

/* Discrete PMSM plant in αβ: L di/dt = v - Rs i - e, eα=-ψωsin, eβ=ψωcos. */
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

TEST_CASE("observer init rejects bad params", "[espFoC][observer]")
{
    esp_foc_observer_bemf_t o;
    esp_foc_observer_bemf_config_t c = base_cfg(ESP_FOC_ANGLE_ATAN2);
    c.rs_ohm = 0.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_observer_bemf_init(&o, &c));

    c = base_cfg(ESP_FOC_ANGLE_PLL);
    c.pll_bw_hz = 20000.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_observer_bemf_init(&o, &c));
}

TEST_CASE("observer current model atan2 tracks plant angle", "[espFoC][observer]")
{
    esp_foc_observer_bemf_t store;
    esp_foc_observer_bemf_config_t c = base_cfg(ESP_FOC_ANGLE_ATAN2);
    esp_foc_observer_t *o = bemf_ok(&store, &c);

    const float rs = c.rs_ohm;
    const float ls = c.ls_h;
    const float psi = c.psi_f_wb;
    const float ts = c.ts_s;
    const float omega = 2.0f * (float)M_PI * 50.0f;
    float theta = 0.3f;
    float ia = 0.0f;
    float ib = 0.0f;
    const float iq = 0.3f;
    const float id = 0.0f;

    esp_foc_observer_set_theta(o, q16_from_float(theta));
    esp_foc_observer_set_omega(o, q16_from_float(omega));

    for (int i = 0; i < 8000; i++) {
        float s = sinf(theta);
        float co = cosf(theta);
        float vd = rs * id;
        float vq = rs * iq + psi * omega;
        float va = vd * co - vq * s;
        float vb = vd * s + vq * co;
        plant_step(rs, ls, psi, ts, theta, omega, va, vb, &ia, &ib);
        esp_foc_observer_update(o,
                                q16_from_float(ia), q16_from_float(ib),
                                q16_from_float(va), q16_from_float(vb));
        theta += omega * ts;
        if (theta > (float)M_PI) {
            theta -= 2.0f * (float)M_PI;
        }
    }

    float d = wrap_pi_f(q16_to_float(esp_foc_observer_get_theta(o)) - theta);
    TEST_ASSERT_TRUE(fabsf(d) < 0.55f);
    TEST_ASSERT_TRUE(esp_foc_observer_is_locked(o));
    float w_est = q16_to_float(esp_foc_observer_get_omega(o));
    TEST_ASSERT_FLOAT_WITHIN(80.0f, omega, w_est);
}

TEST_CASE("observer PLL tracks plant angle", "[espFoC][observer]")
{
    esp_foc_observer_bemf_t store;
    esp_foc_observer_bemf_config_t c = base_cfg(ESP_FOC_ANGLE_PLL);
    c.lock_count = 200u;
    esp_foc_observer_t *o = bemf_ok(&store, &c);

    const float rs = c.rs_ohm;
    const float ls = c.ls_h;
    const float psi = c.psi_f_wb;
    const float ts = c.ts_s;
    const float omega = 2.0f * (float)M_PI * 50.0f;
    float theta = -0.8f;
    float ia = 0.0f;
    float ib = 0.0f;
    const float iq = 0.3f;
    const float id = 0.0f;

    esp_foc_observer_set_theta(o, q16_from_float(theta + 0.4f));
    esp_foc_observer_set_omega(o, q16_from_float(omega));

    for (int i = 0; i < 6000; i++) {
        float s = sinf(theta);
        float co = cosf(theta);
        float vd = rs * id;
        float vq = rs * iq + psi * omega;
        float va = vd * co - vq * s;
        float vb = vd * s + vq * co;
        plant_step(rs, ls, psi, ts, theta, omega, va, vb, &ia, &ib);
        esp_foc_observer_update(o,
                                q16_from_float(ia), q16_from_float(ib),
                                q16_from_float(va), q16_from_float(vb));
        theta += omega * ts;
        if (theta > (float)M_PI) {
            theta -= 2.0f * (float)M_PI;
        }
        if (theta <= -(float)M_PI) {
            theta += 2.0f * (float)M_PI;
        }
    }

    float d = wrap_pi_f(q16_to_float(esp_foc_observer_get_theta(o)) - theta);
    TEST_ASSERT_TRUE(fabsf(d) < 0.40f);
    TEST_ASSERT_TRUE(esp_foc_observer_is_locked(o));
}

TEST_CASE("observer atan2 and PLL agree in steady state", "[espFoC][observer]")
{
    esp_foc_observer_bemf_t store_a;
    esp_foc_observer_bemf_t store_p;
    esp_foc_observer_bemf_config_t ca = base_cfg(ESP_FOC_ANGLE_ATAN2);
    esp_foc_observer_bemf_config_t cp = base_cfg(ESP_FOC_ANGLE_PLL);
    esp_foc_observer_t *oa = bemf_ok(&store_a, &ca);
    esp_foc_observer_t *op = bemf_ok(&store_p, &cp);

    const float rs = ca.rs_ohm;
    const float ls = ca.ls_h;
    const float psi = ca.psi_f_wb;
    const float ts = ca.ts_s;
    const float omega = 2.0f * (float)M_PI * 80.0f;
    float theta = 0.0f;
    float ia = 0.0f;
    float ib = 0.0f;
    const float iq = 0.25f;

    esp_foc_observer_set_theta(oa, 0);
    esp_foc_observer_set_theta(op, 0);
    esp_foc_observer_set_omega(oa, q16_from_float(omega));
    esp_foc_observer_set_omega(op, q16_from_float(omega));

    for (int i = 0; i < 5000; i++) {
        float s = sinf(theta);
        float co = cosf(theta);
        float va = -(rs * iq + psi * omega) * s;
        float vb = (rs * iq + psi * omega) * co;
        plant_step(rs, ls, psi, ts, theta, omega, va, vb, &ia, &ib);
        q16_t iaq = q16_from_float(ia);
        q16_t ibq = q16_from_float(ib);
        q16_t vaq = q16_from_float(va);
        q16_t vbq = q16_from_float(vb);
        esp_foc_observer_update(oa, iaq, ibq, vaq, vbq);
        esp_foc_observer_update(op, iaq, ibq, vaq, vbq);
        theta += omega * ts;
        if (theta > (float)M_PI) {
            theta -= 2.0f * (float)M_PI;
        }
    }

    float d = wrap_pi_f(q16_to_float(esp_foc_observer_get_theta(oa)) -
                        q16_to_float(esp_foc_observer_get_theta(op)));
    TEST_ASSERT_TRUE(fabsf(d) < 0.45f);
}

TEST_CASE("observer lock rises from large phase error", "[espFoC][observer]")
{
    esp_foc_observer_bemf_t store;
    esp_foc_observer_bemf_config_t c = base_cfg(ESP_FOC_ANGLE_ATAN2);
    c.lock_count = 50u;
    c.e_lock_min_v = 0.10f;
    esp_foc_observer_t *o = bemf_ok(&store, &c);

    const float rs = c.rs_ohm;
    const float ls = c.ls_h;
    const float psi = c.psi_f_wb;
    const float ts = c.ts_s;
    const float omega = 2.0f * (float)M_PI * 60.0f;
    float theta = 0.0f;
    float ia = 0.0f;
    float ib = 0.0f;
    const float iq = 0.35f;

    esp_foc_observer_set_theta(o, q16_from_float(2.6f));
    esp_foc_observer_set_omega(o, 0);
    TEST_ASSERT_FALSE(esp_foc_observer_is_locked(o));

    for (int i = 0; i < 3000; i++) {
        float s = sinf(theta);
        float co = cosf(theta);
        float va = -(rs * iq + psi * omega) * s;
        float vb = (rs * iq + psi * omega) * co;
        plant_step(rs, ls, psi, ts, theta, omega, va, vb, &ia, &ib);
        esp_foc_observer_update(o,
                                q16_from_float(ia), q16_from_float(ib),
                                q16_from_float(va), q16_from_float(vb));
        theta += omega * ts;
        if (theta > (float)M_PI) {
            theta -= 2.0f * (float)M_PI;
        }
    }
    TEST_ASSERT_TRUE(esp_foc_observer_is_locked(o));
}

TEST_CASE("observer PLL tracks 300 Hz with LPF lag compensation", "[espFoC][observer]")
{
    esp_foc_observer_bemf_t store;
    esp_foc_observer_bemf_config_t c = base_cfg(ESP_FOC_ANGLE_PLL);
    c.emf_lpf_hz = 300.0f;
    c.w_max_rads = 2.0f * (float)M_PI * 800.0f;
    c.lock_count = 200u;
    esp_foc_observer_t *o = bemf_ok(&store, &c);

    const float rs = c.rs_ohm;
    const float ls = c.ls_h;
    const float psi = c.psi_f_wb;
    const float ts = c.ts_s;
    const float omega = 2.0f * (float)M_PI * 300.0f;
    float theta = 0.0f;
    float ia = 0.0f;
    float ib = 0.0f;
    const float iq = 0.25f;

    esp_foc_observer_set_theta(o, 0);
    esp_foc_observer_set_omega(o, q16_from_float(omega));

    for (int i = 0; i < 8000; i++) {
        float s = sinf(theta);
        float co = cosf(theta);
        float va = -(rs * iq + psi * omega) * s;
        float vb = (rs * iq + psi * omega) * co;
        plant_step(rs, ls, psi, ts, theta, omega, va, vb, &ia, &ib);
        esp_foc_observer_update(o,
                                q16_from_float(ia), q16_from_float(ib),
                                q16_from_float(va), q16_from_float(vb));
        theta += omega * ts;
        if (theta > (float)M_PI) {
            theta -= 2.0f * (float)M_PI;
        }
    }

    float d = wrap_pi_f(q16_to_float(esp_foc_observer_get_theta(o)) - theta);
    TEST_ASSERT_TRUE(fabsf(d) < 0.45f);
    TEST_ASSERT_TRUE(esp_foc_observer_is_locked(o));
    float w_est = q16_to_float(esp_foc_observer_get_omega(o));
    TEST_ASSERT_FLOAT_WITHIN(200.0f, omega, w_est);
}

TEST_CASE("observer PLL tracks reverse rotation", "[espFoC][observer]")
{
    esp_foc_observer_bemf_t store;
    esp_foc_observer_bemf_config_t c = base_cfg(ESP_FOC_ANGLE_PLL);
    c.lock_count = 200u;
    esp_foc_observer_t *o = bemf_ok(&store, &c);

    const float rs = c.rs_ohm;
    const float ls = c.ls_h;
    const float psi = c.psi_f_wb;
    const float ts = c.ts_s;
    const float omega = -2.0f * (float)M_PI * 120.0f;
    float theta = 0.0f;
    float ia = 0.0f;
    float ib = 0.0f;
    const float iq = -0.25f;

    esp_foc_observer_set_theta(o, 0);
    esp_foc_observer_set_omega(o, q16_from_float(omega));

    for (int i = 0; i < 8000; i++) {
        float s = sinf(theta);
        float co = cosf(theta);
        float va = -(rs * iq + psi * omega) * s;
        float vb = (rs * iq + psi * omega) * co;
        plant_step(rs, ls, psi, ts, theta, omega, va, vb, &ia, &ib);
        esp_foc_observer_update(o,
                                q16_from_float(ia), q16_from_float(ib),
                                q16_from_float(va), q16_from_float(vb));
        theta += omega * ts;
        if (theta <= -(float)M_PI) {
            theta += 2.0f * (float)M_PI;
        }
    }

    float d = wrap_pi_f(q16_to_float(esp_foc_observer_get_theta(o)) - theta);
    TEST_ASSERT_TRUE(fabsf(d) < 0.45f);
    TEST_ASSERT_TRUE(esp_foc_observer_is_locked(o));
    TEST_ASSERT_TRUE(q16_to_float(esp_foc_observer_get_omega(o)) < 0.0f);
}

/* Open: max_step exceeds 0.25 rad on silicon. CI skips [known_fail]. */
TEST_CASE("observer current model keeps phase continuity through 50 to 300 Hz ramp",
          "[espFoC][observer][known_fail]")
{
    esp_foc_observer_bemf_t store;
    esp_foc_observer_bemf_config_t c = base_cfg(ESP_FOC_ANGLE_PLL);
    c.emf_lpf_hz = 150.0f;
    c.w_max_rads = 2.0f * (float)M_PI * 800.0f;
    c.lock_count = 200u;
    c.unlock_count = 4000u;
    esp_foc_observer_t *o = bemf_ok(&store, &c);

    const float ts = c.ts_s;
    const float w0 = 2.0f * (float)M_PI * 50.0f;
    const float w1 = 2.0f * (float)M_PI * 300.0f;
    float theta = 0.0f;
    float ia = 0.0f;
    float ib = 0.0f;
    q16_t theta_prev = 0;
    bool have_theta = false;
    q16_t max_step = 0;

    esp_foc_observer_set_theta(o, 0);
    esp_foc_observer_set_omega(o, q16_from_float(w0));

    for (int i = 0; i < 12000; i++) {
        float ratio = i < 8000 ? (float)i / 8000.0f : 1.0f;
        float omega = w0 + (w1 - w0) * ratio;
        float s = sinf(theta);
        float co = cosf(theta);
        float vd = -omega * c.ls_h * 0.25f;
        float vq = c.rs_ohm * 0.25f + c.psi_f_wb * omega;
        float va = vd * co - vq * s;
        float vb = vd * s + vq * co;
        plant_step(c.rs_ohm, c.ls_h, c.psi_f_wb, ts, theta, omega,
                   va, vb, &ia, &ib);
        esp_foc_observer_update(o,
                                q16_from_float(ia), q16_from_float(ib),
                                q16_from_float(va), q16_from_float(vb));
        q16_t theta_now = esp_foc_observer_get_theta(o);
        if (have_theta) {
            q16_t step = q16_angle_delta(theta_prev, theta_now);
            if (step < 0) {
                step = q16_neg(step);
            }
            if (step > max_step) {
                max_step = step;
            }
        }
        theta_prev = theta_now;
        have_theta = true;
        theta += omega * ts;
        if (theta > (float)M_PI) {
            theta -= 2.0f * (float)M_PI;
        }
    }

    float error = wrap_pi_f(q16_to_float(esp_foc_observer_get_theta(o)) - theta);
    TEST_ASSERT_TRUE(fabsf(error) < 0.45f);
    TEST_ASSERT_TRUE(max_step < q16_from_float(0.25f));
    TEST_ASSERT_TRUE(esp_foc_observer_is_locked(o));
}

TEST_CASE("observer numerical stability at zero excitation", "[espFoC][observer]")
{
    esp_foc_observer_bemf_t store;
    esp_foc_observer_bemf_config_t c = base_cfg(ESP_FOC_ANGLE_ATAN2);
    esp_foc_observer_t *o = bemf_ok(&store, &c);
    esp_foc_observer_reset(o);

    for (int i = 0; i < 1000; i++) {
        esp_foc_observer_update(o, 0, 0, 0, 0);
    }
    TEST_ASSERT_FALSE(esp_foc_observer_is_locked(o));
    TEST_ASSERT_INT32_WITHIN(q16_from_float(5.0f), 0, esp_foc_observer_get_omega(o));

    for (int i = 0; i < 200; i++) {
        esp_foc_observer_update(o,
                                q16_from_float(20.0f), q16_from_float(-20.0f),
                                q16_from_float(100.0f), q16_from_float(-100.0f));
    }
    q16_t ea = esp_foc_observer_get_e_alpha(o);
    q16_t eb = esp_foc_observer_get_e_beta(o);
    TEST_ASSERT_TRUE(ea > (q16_t)INT32_MIN + 1000);
    TEST_ASSERT_TRUE(eb < (q16_t)INT32_MAX - 1000);
}
