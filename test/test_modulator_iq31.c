/*
 * Unit tests for IQ31 SVPWM (space_vector_modulator.h) and modulator.h.
 * Typical operating points, edge cases, and parity vs float reference (tests only).
 */
#include <math.h>
#include <unity.h>
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/foc_math_q16.h"
#include "espFoC/utils/modulator.h"
#include "test_float_modulator_ref.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TOL_SVM    0.025f
#define TOL_MOD    0.035f
#define TOL_DQ     0.03f

static void assert_svm_matches_float(float va, float vb, float inv, float tol)
{
    float df, ef, ff;
    q16_t dq, eq, fq;
    esp_foc_svm_set_float(va, vb, inv, &df, &ef, &ff);
    esp_foc_svm_set(q16_from_float(va), q16_from_float(vb), q16_from_float(inv),
                    &dq, &eq, &fq);
    TEST_ASSERT_FLOAT_WITHIN(tol, df, q16_to_float(dq));
    TEST_ASSERT_FLOAT_WITHIN(tol, ef, q16_to_float(eq));
    TEST_ASSERT_FLOAT_WITHIN(tol, ff, q16_to_float(fq));
}

TEST_CASE("modulator_iq31: svm zero alpha beta gives duties near 0.5", "[espFoC][modulator_iq31]")
{
    q16_t da, db, dc;
    esp_foc_svm_set(0, 0, q16_from_float(0.5f), &da, &db, &dc);
    TEST_ASSERT_FLOAT_WITHIN(TOL_SVM, 0.5f, q16_to_float(da));
    TEST_ASSERT_FLOAT_WITHIN(TOL_SVM, 0.5f, q16_to_float(db));
    TEST_ASSERT_FLOAT_WITHIN(TOL_SVM, 0.5f, q16_to_float(dc));
}

TEST_CASE("modulator_iq31: svm matches float grid of points", "[espFoC][modulator_iq31]")
{
    const float inv = 0.5f;
    for (int i = -4; i <= 4; i++) {
        for (int j = -4; j <= 4; j++) {
            float va = (float)i * 0.12f;
            float vb = (float)j * 0.12f;
            assert_svm_matches_float(va, vb, inv, TOL_SVM);
        }
    }
}

TEST_CASE("modulator_iq31: svm edge clamping large voltages", "[espFoC][modulator_iq31]")
{
    q16_t da, db, dc;
    esp_foc_svm_set(Q16_ONE, Q16_ONE, q16_from_float(1.0f), &da, &db, &dc);
    TEST_ASSERT_TRUE(q16_to_float(da) >= 0.0f && q16_to_float(da) <= 1.0f);
    TEST_ASSERT_TRUE(q16_to_float(db) >= 0.0f && q16_to_float(db) <= 1.0f);
    TEST_ASSERT_TRUE(q16_to_float(dc) >= 0.0f && q16_to_float(dc) <= 1.0f);
}

TEST_CASE("modulator_iq31: svm small inv_vbus", "[espFoC][modulator_iq31]")
{
    assert_svm_matches_float(0.3f, -0.2f, 0.05f, TOL_SVM);
}

TEST_CASE("modulator_iq31: svm saturated alpha", "[espFoC][modulator_iq31]")
{
    assert_svm_matches_float(0.99f, 0.0f, 0.5f, TOL_SVM);
    assert_svm_matches_float(-0.99f, 0.0f, 0.5f, TOL_SVM);
}

TEST_CASE("modulator_iq31: modulate_dq matches float typical operating point", "[espFoC][modulator_iq31]")
{
    float angle = (float)(M_PI / 6.0);
    float fs = sinf(angle);
    float fc = cosf(angle);
    float vd = 0.2f;
    float vq = 0.35f;
    float vmax = 0.9f;
    float norm = 0.5f;

    float fa, fb, fu, fv, fw;
    esp_foc_modulate_dq_voltage_float(fs, fc, vd, vq, &fa, &fb, &fu, &fv, &fw, vmax, 0.0f, norm);

    q16_t qa, qb, qu, qv, qw;
    esp_foc_modulate_dq_voltage(q16_from_float(fs), q16_from_float(fc),
                                     q16_from_float(vd), q16_from_float(vq),
                                     &qa, &qb, &qu, &qv, &qw,
                                     q16_from_float(vmax), 0,
                                     q16_from_float(norm));

    TEST_ASSERT_FLOAT_WITHIN(TOL_MOD, fa, q16_to_float(qa));
    TEST_ASSERT_FLOAT_WITHIN(TOL_MOD, fb, q16_to_float(qb));
    TEST_ASSERT_FLOAT_WITHIN(TOL_MOD, fu, q16_to_float(qu));
    TEST_ASSERT_FLOAT_WITHIN(TOL_MOD, fv, q16_to_float(qv));
    TEST_ASSERT_FLOAT_WITHIN(TOL_MOD, fw, q16_to_float(qw));
}

TEST_CASE("modulator_iq31: modulate_dq voltage limit scales dq", "[espFoC][modulator_iq31]")
{
    float fs = 1.0f;
    float fc = 0.0f;
    float vd = 0.95f;
    float vq = 0.95f;
    float vmax = 0.5f;
    float norm = 0.5f;

    float fa, fb, fu, fv, fw;
    esp_foc_modulate_dq_voltage_float(fs, fc, vd, vq, &fa, &fb, &fu, &fv, &fw, vmax, 0.0f, norm);

    q16_t qa, qb, qu, qv, qw;
    esp_foc_modulate_dq_voltage(q16_from_float(fs), q16_from_float(fc),
                                     q16_from_float(vd), q16_from_float(vq),
                                     &qa, &qb, &qu, &qv, &qw,
                                     q16_from_float(vmax), 0,
                                     q16_from_float(norm));

    TEST_ASSERT_FLOAT_WITHIN(TOL_MOD, fa, q16_to_float(qa));
    TEST_ASSERT_FLOAT_WITHIN(TOL_MOD, fb, q16_to_float(qb));
    TEST_ASSERT_FLOAT_WITHIN(TOL_MOD, fu, q16_to_float(qu));
}

TEST_CASE("modulator_iq31: modulate_dq zero dq", "[espFoC][modulator_iq31]")
{
    float fs = 0.0f;
    float fc = 1.0f;
    float fa, fb, fu, fv, fw;
    esp_foc_modulate_dq_voltage_float(fs, fc, 0.0f, 0.0f, &fa, &fb, &fu, &fv, &fw, 1.0f, 0.0f, 0.5f);

    q16_t qa, qb, qu, qv, qw;
    esp_foc_modulate_dq_voltage(0, Q16_ONE, 0, 0, &qa, &qb, &qu, &qv, &qw,
                                     Q16_ONE, 0, q16_from_float(0.5f));

    TEST_ASSERT_FLOAT_WITHIN(TOL_MOD, fa, q16_to_float(qa));
    TEST_ASSERT_FLOAT_WITHIN(TOL_MOD, fu, q16_to_float(qu));
}

TEST_CASE("modulator_iq31: get_dq_currents matches float balanced currents", "[espFoC][modulator_iq31]")
{
    float fs = sinf((float)(M_PI / 4.0));
    float fc = cosf((float)(M_PI / 4.0));
    float iu = 0.4f;
    float iv = -0.2f;
    float iw = -0.2f;

    float ia, ib, iq, id;
    esp_foc_get_dq_currents_float(fs, fc, iu, iv, iw, &ia, &ib, &iq, &id);

    q16_t qa, qb, qq, qd;
    esp_foc_get_dq_currents(q16_from_float(fs), q16_from_float(fc),
                                 q16_from_float(iu), q16_from_float(iv), q16_from_float(iw),
                                 &qa, &qb, &qq, &qd);

    TEST_ASSERT_FLOAT_WITHIN(TOL_DQ, ia, q16_to_float(qa));
    TEST_ASSERT_FLOAT_WITHIN(TOL_DQ, ib, q16_to_float(qb));
    TEST_ASSERT_FLOAT_WITHIN(TOL_DQ, iq, q16_to_float(qq));
    TEST_ASSERT_FLOAT_WITHIN(TOL_DQ, id, q16_to_float(qd));
}

TEST_CASE("modulator_iq31: get_dq_currents edge zero currents", "[espFoC][modulator_iq31]")
{
    q16_t qa, qb, qq, qd;
    esp_foc_get_dq_currents(Q16_ONE, 0, 0, 0, 0, &qa, &qb, &qq, &qd);
    TEST_ASSERT_FLOAT_WITHIN(TOL_DQ, 0.0f, q16_to_float(qq));
    TEST_ASSERT_FLOAT_WITHIN(TOL_DQ, 0.0f, q16_to_float(qd));
}

TEST_CASE("modulator_iq31: get_dq_currents matches float near saturation", "[espFoC][modulator_iq31]")
{
    float fs = 0.707f;
    float fc = 0.707f;
    float iu = 0.9f;
    float iv = -0.45f;
    float iw = -0.45f;
    float ia, ib, iq, id;
    esp_foc_get_dq_currents_float(fs, fc, iu, iv, iw, &ia, &ib, &iq, &id);

    q16_t qa, qb, qq, qd;
    esp_foc_get_dq_currents(q16_from_float(fs), q16_from_float(fc),
                                 q16_from_float(iu), q16_from_float(iv), q16_from_float(iw),
                                 &qa, &qb, &qq, &qd);

    TEST_ASSERT_FLOAT_WITHIN(TOL_DQ, ia, q16_to_float(qa));
    TEST_ASSERT_FLOAT_WITHIN(TOL_DQ, ib, q16_to_float(qb));
    TEST_ASSERT_FLOAT_WITHIN(TOL_DQ, iq, q16_to_float(qq));
    TEST_ASSERT_FLOAT_WITHIN(TOL_DQ, id, q16_to_float(qd));
}
