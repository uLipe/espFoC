/*
 * Unit tests for esp_foc_estimator_q16 (encoder PLL).
 */
#include <unity.h>
#include "espFoC/esp_foc_estimator_q16.h"
#include "espFoC/utils/esp_foc_q16.h"

static esp_foc_estimator_q16_t est;

static void init_est(float bw_hz, float omega_max_rev_s, int pp)
{
    esp_foc_estimator_q16_config_t cfg = {
        .dt_isr = q16_from_float(1.0f / 20000.0f),
        .pll_bw_hz = bw_hz,
        .omega_max_mech = q16_from_float(omega_max_rev_s),
        .pole_pairs = pp,
    };
    esp_foc_estimator_q16_init(&est, &cfg);
}

TEST_CASE("estimator: snap aligns state", "[espFoC][estimator]")
{
    init_est(50.0f, 100.0f, 7);
    q16_t meas = q16_from_float(0.25f);
    esp_foc_estimator_q16_snap(&est, meas);

    TEST_ASSERT_EQUAL_INT32(meas, est.theta_est_mech);
    TEST_ASSERT_EQUAL_INT32(meas, est.theta_meas_mech);
    TEST_ASSERT_EQUAL_INT32(0, est.omega_est_mech);
    TEST_ASSERT_EQUAL_INT32(0, est.pll_err);
}

TEST_CASE("estimator: wrap error at half turn", "[espFoC][estimator]")
{
    init_est(50.0f, 100.0f, 7);
    esp_foc_estimator_q16_snap(&est, q16_from_float(0.01f));
    esp_foc_estimator_q16_set_meas(&est, q16_from_float(0.99f));
    esp_foc_estimator_q16_step(&est);

    TEST_ASSERT_INT32_WITHIN(q16_from_float(0.02f), q16_from_float(-0.02f), est.pll_err);
}

TEST_CASE("estimator: integrates toward constant meas", "[espFoC][estimator]")
{
    init_est(50.0f, 100.0f, 7);
    q16_t target = q16_from_float(0.4f);
    esp_foc_estimator_q16_snap(&est, 0);

    for (int i = 0; i < 4000; i++) {
        esp_foc_estimator_q16_set_meas(&est, target);
        esp_foc_estimator_q16_step(&est);
    }

    TEST_ASSERT_INT32_WITHIN(q16_from_float(0.02f), target, est.theta_est_mech);
}

TEST_CASE("estimator: omega clamp holds integrator", "[espFoC][estimator]")
{
    init_est(50.0f, 1.0f, 7);
    esp_foc_estimator_q16_snap(&est, 0);
    esp_foc_estimator_q16_set_meas(&est, q16_from_float(0.5f));

    for (int i = 0; i < 5000; i++) {
        esp_foc_estimator_q16_step(&est);
    }

    TEST_ASSERT_TRUE(est.omega_est_mech <= q16_from_float(1.0f));
    TEST_ASSERT_TRUE(est.omega_est_mech >= q16_from_float(-1.0f));
}

TEST_CASE("estimator: theta_elec scales by pole pairs", "[espFoC][estimator]")
{
    init_est(50.0f, 100.0f, 4);
    esp_foc_estimator_q16_snap(&est, q16_from_float(0.1f));
    q16_t theta_e = esp_foc_estimator_q16_theta_elec(&est);
    TEST_ASSERT_INT32_WITHIN(4, q16_from_float(0.4f), theta_e);
}
