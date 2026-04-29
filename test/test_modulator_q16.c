/*
 * Unit tests for Q16 SVPWM (space_vector_modulator.h) and modulator.h.
 * Fixed-point only — duty bounds, consistency with foc_math_q16, edge cases.
 */
#include <math.h>
#include <unity.h>
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/foc_math_q16.h"
#include "espFoC/utils/modulator.h"
#include "espFoC/utils/space_vector_modulator.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DUTY_ENV   0.002f
#define AB_TOL     0.06f

static void assert_duty_unit(q16_t d)
{
    float f = q16_to_float(d);
    TEST_ASSERT_TRUE(f >= -DUTY_ENV && f <= 1.0f + DUTY_ENV);
}

TEST_CASE("modulator_q16: svm zero alpha beta gives duties near 0.5", "[espFoC][modulator_q16]")
{
    q16_t da, db, dc;
    esp_foc_svm_set(0, 0, q16_from_float(0.5f), &da, &db, &dc);
    TEST_ASSERT_FLOAT_WITHIN(0.025f, 0.5f, q16_to_float(da));
    TEST_ASSERT_FLOAT_WITHIN(0.025f, 0.5f, q16_to_float(db));
    TEST_ASSERT_FLOAT_WITHIN(0.025f, 0.5f, q16_to_float(dc));
}

TEST_CASE("modulator_q16: svm grid duties stay in unit interval", "[espFoC][modulator_q16]")
{
    const q16_t inv = q16_from_float(0.5f);
    for (int i = -4; i <= 4; i++) {
        for (int j = -4; j <= 4; j++) {
            q16_t va = q16_from_float((float)i * 0.12f);
            q16_t vb = q16_from_float((float)j * 0.12f);
            q16_t da, db, dc;
            esp_foc_svm_set(va, vb, inv, &da, &db, &dc);
            assert_duty_unit(da);
            assert_duty_unit(db);
            assert_duty_unit(dc);
        }
    }
}

TEST_CASE("modulator_q16: svm edge clamping large voltages", "[espFoC][modulator_q16]")
{
    q16_t da, db, dc;
    esp_foc_svm_set(Q16_ONE, Q16_ONE, q16_from_float(1.0f), &da, &db, &dc);
    assert_duty_unit(da);
    assert_duty_unit(db);
    assert_duty_unit(dc);
}

TEST_CASE("modulator_q16: svm small inv_vbus", "[espFoC][modulator_q16]")
{
    q16_t da, db, dc;
    esp_foc_svm_set(q16_from_float(0.3f), q16_from_float(-0.2f),
                    q16_from_float(0.05f), &da, &db, &dc);
    assert_duty_unit(da);
    assert_duty_unit(db);
    assert_duty_unit(dc);
}

TEST_CASE("modulator_q16: svm saturated alpha", "[espFoC][modulator_q16]")
{
    q16_t da, db, dc;
    esp_foc_svm_set(q16_from_float(0.99f), 0, q16_from_float(0.5f), &da, &db, &dc);
    assert_duty_unit(da);
    assert_duty_unit(db);
    assert_duty_unit(dc);
    esp_foc_svm_set(q16_from_float(-0.99f), 0, q16_from_float(0.5f), &da, &db, &dc);
    assert_duty_unit(da);
    assert_duty_unit(db);
    assert_duty_unit(dc);
}

TEST_CASE("modulator_q16: modulate_dq typical operating point", "[espFoC][modulator_q16]")
{
    float angle = (float)(M_PI / 6.0);
    q16_t sq = q16_from_float(sinf(angle));
    q16_t cq = q16_from_float(cosf(angle));
    q16_t vd = q16_from_float(0.2f);
    q16_t vq = q16_from_float(0.35f);
    q16_t vmax = q16_from_float(0.9f);
    q16_t exp_a, exp_b;
    esp_foc_limit_voltage_q16(&vd, &vq, vmax);
    q16_inverse_park(sq, cq, vd, vq, &exp_a, &exp_b);
    q16_t qa, qb, qu, qv, qw;
    esp_foc_modulate_dq_voltage(sq, cq,
                                q16_from_float(0.2f), q16_from_float(0.35f),
                                &qa, &qb, &qu, &qv, &qw,
                                vmax);
    TEST_ASSERT_EQUAL_INT(exp_a, qa);
    TEST_ASSERT_EQUAL_INT(exp_b, qb);
    q16_t eu, ev, ew;
    esp_foc_svm_alpha_beta_to_phase_volts(exp_a, exp_b, &eu, &ev, &ew);
    TEST_ASSERT_EQUAL_INT(eu, qu);
    TEST_ASSERT_EQUAL_INT(ev, qv);
    TEST_ASSERT_EQUAL_INT(ew, qw);
}

TEST_CASE("modulator_q16: modulate_dq voltage limit scales dq", "[espFoC][modulator_q16]")
{
    q16_t qa, qb, qu, qv, qw;
    esp_foc_modulate_dq_voltage(q16_from_float(1.0f), 0,
                                q16_from_float(0.95f), q16_from_float(0.95f),
                                &qa, &qb, &qu, &qv, &qw,
                                q16_from_float(0.5f));
    TEST_ASSERT_TRUE(fabsf(q16_to_float(qu)) < 1.0f);
    TEST_ASSERT_TRUE(fabsf(q16_to_float(qv)) < 1.0f);
    TEST_ASSERT_TRUE(fabsf(q16_to_float(qw)) < 1.0f);
}

TEST_CASE("modulator_q16: modulate_dq zero dq", "[espFoC][modulator_q16]")
{
    q16_t qa, qb, qu, qv, qw;
    esp_foc_modulate_dq_voltage(0, Q16_ONE, 0, 0, &qa, &qb, &qu, &qv, &qw,
                                Q16_ONE);
    TEST_ASSERT_FLOAT_WITHIN(AB_TOL, 0.0f, q16_to_float(qa));
    TEST_ASSERT_FLOAT_WITHIN(AB_TOL, 0.0f, q16_to_float(qu));
    TEST_ASSERT_FLOAT_WITHIN(AB_TOL, 0.0f, q16_to_float(qv));
    TEST_ASSERT_FLOAT_WITHIN(AB_TOL, 0.0f, q16_to_float(qw));
}

TEST_CASE("modulator_q16: get_dq_currents balanced currents", "[espFoC][modulator_q16]")
{
    q16_t sq = q16_from_float(sinf((float)(M_PI / 4.0)));
    q16_t cq = q16_from_float(cosf((float)(M_PI / 4.0)));
    q16_t iu = q16_from_float(0.4f);
    q16_t iv = q16_from_float(-0.2f);
    q16_t iw = q16_from_float(-0.2f);
    q16_t ia, ib, ed, eq;
    q16_clarke(iu, iv, iw, &ia, &ib);
    q16_park(sq, cq, ia, ib, &ed, &eq);
    q16_t qa, qb, qq, qd;
    esp_foc_get_dq_currents(sq, cq, iu, iv, iw, &qa, &qb, &qq, &qd);
    TEST_ASSERT_EQUAL_INT(ia, qa);
    TEST_ASSERT_EQUAL_INT(ib, qb);
    TEST_ASSERT_EQUAL_INT(ed, qd);
    TEST_ASSERT_EQUAL_INT(eq, qq);
}

TEST_CASE("modulator_q16: get_dq_currents edge zero currents", "[espFoC][modulator_q16]")
{
    q16_t qa, qb, qq, qd;
    esp_foc_get_dq_currents(Q16_ONE, 0, 0, 0, 0, &qa, &qb, &qq, &qd);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.0f, q16_to_float(qq));
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.0f, q16_to_float(qd));
}

TEST_CASE("modulator_q16: get_dq_currents near saturation", "[espFoC][modulator_q16]")
{
    q16_t qa, qb, qq, qd;
    esp_foc_get_dq_currents(q16_from_float(0.707f), q16_from_float(0.707f),
                            q16_from_float(0.9f),
                            q16_from_float(-0.45f),
                            q16_from_float(-0.45f),
                            &qa, &qb, &qq, &qd);
    TEST_ASSERT_TRUE(fabsf(q16_to_float(qq)) < 2.0f);
    TEST_ASSERT_TRUE(fabsf(q16_to_float(qd)) < 2.0f);
}
