/*
 * Golden tests: manual FOC pipeline execution in IQ31 vs float baseline.
 */
#include <math.h>
#include <unity.h>
#include "espFoC/utils/esp_foc_iq31.h"
#include "espFoC/utils/foc_math.h"
#include "espFoC/utils/foc_math_iq31.h"
#include "espFoC/utils/modulator.h"
#include "espFoC/utils/modulator_iq31.h"
#include "espFoC/utils/pid_controller.h"
#include "espFoC/utils/pid_controller_iq31.h"
#include "espFoC/utils/ema_low_pass_filter.h"
#include "espFoC/utils/ema_low_pass_filter_iq31.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define GOLDEN_TOL_DUTY 0.045f
#define GOLDEN_TOL_DQ   0.05f

static void run_manual_pipeline_step(float angle_rad,
                                     float iu, float iv, float iw,
                                     float id_ref, float iq_ref,
                                     esp_foc_pid_controller_t *pid_f_d,
                                     esp_foc_pid_controller_t *pid_f_q,
                                     esp_foc_lp_filter_t *filt_f_d,
                                     esp_foc_lp_filter_t *filt_f_q,
                                     esp_foc_pid_iq31_t *pid_q_d,
                                     esp_foc_pid_iq31_t *pid_q_q,
                                     esp_foc_lp_filter_iq31_t *filt_q_d,
                                     esp_foc_lp_filter_iq31_t *filt_q_q,
                                     float *du_f, float *dv_f, float *dw_f,
                                     float *du_q, float *dv_q, float *dw_q)
{
    float s = sinf(angle_rad);
    float c = cosf(angle_rad);
    float ia_f, ib_f, id_f, iq_f;
    float ua_f, ub_f;
    float uq_f, ud_f;
    const float vmax = 0.7f;
    const float norm = 0.5f;

    esp_foc_get_dq_currents(s, c, iu, iv, iw, &ia_f, &ib_f, &iq_f, &id_f);
    uq_f = esp_foc_pid_update(pid_f_q, iq_ref, esp_foc_low_pass_filter_update(filt_f_q, iq_f));
    ud_f = esp_foc_pid_update(pid_f_d, id_ref, esp_foc_low_pass_filter_update(filt_f_d, id_f));
    esp_foc_modulate_dq_voltage(s, c, ud_f, uq_f, &ua_f, &ub_f, du_f, dv_f, dw_f, vmax, 0.0f, norm);

    iq31_t sq = iq31_sin(iq31_from_float(angle_rad / (2.0f * (float)M_PI)));
    iq31_t cq = iq31_cos(iq31_from_float(angle_rad / (2.0f * (float)M_PI)));
    iq31_t ia_q, ib_q, id_q, iq_q;
    iq31_t uq_q, ud_q;
    iq31_t ua_q, ub_q, du_iq, dv_iq, dw_iq;

    esp_foc_get_dq_currents_iq31(sq, cq,
                                 iq31_from_float(iu),
                                 iq31_from_float(iv),
                                 iq31_from_float(iw),
                                 &ia_q, &ib_q, &iq_q, &id_q);
    uq_q = esp_foc_pid_update_iq31(pid_q_q,
                                   iq31_from_float(iq_ref),
                                   esp_foc_low_pass_filter_update_iq31(filt_q_q, iq_q));
    ud_q = esp_foc_pid_update_iq31(pid_q_d,
                                   iq31_from_float(id_ref),
                                   esp_foc_low_pass_filter_update_iq31(filt_q_d, id_q));

    esp_foc_modulate_dq_voltage_iq31(sq, cq, ud_q, uq_q,
                                     &ua_q, &ub_q, &du_iq, &dv_iq, &dw_iq,
                                     iq31_from_float(vmax), 0, iq31_from_float(norm));

    TEST_ASSERT_FLOAT_WITHIN(GOLDEN_TOL_DQ, iq31_to_float(iq_q), iq_f);
    TEST_ASSERT_FLOAT_WITHIN(GOLDEN_TOL_DQ, iq31_to_float(id_q), id_f);
    *du_q = iq31_to_float(du_iq);
    *dv_q = iq31_to_float(dv_iq);
    *dw_q = iq31_to_float(dw_iq);
}

TEST_CASE("golden pipeline iq31: sinusoidal profile matches float pwm", "[espFoC][iq31][golden]")
{
    esp_foc_pid_controller_t pid_f_d = {0};
    esp_foc_pid_controller_t pid_f_q = {0};
    esp_foc_pid_iq31_t pid_q_d = {0};
    esp_foc_pid_iq31_t pid_q_q = {0};
    esp_foc_lp_filter_t filt_f_d, filt_f_q;
    esp_foc_lp_filter_iq31_t filt_q_d, filt_q_q;
    const float dt = 1.0f / 2000.0f;

    pid_f_d.kp = 0.4f; pid_f_d.ki = 60.0f; pid_f_d.kd = 0.0f; pid_f_d.dt = dt; pid_f_d.inv_dt = 1.0f / dt;
    pid_f_d.max_output_value = 0.7f; pid_f_d.min_output_value = -0.7f; pid_f_d.integrator_limit = 0.7f / 60.0f;
    pid_f_q = pid_f_d;
    esp_foc_pid_reset(&pid_f_d);
    esp_foc_pid_reset(&pid_f_q);

    pid_q_d = pid_f_d;
    pid_q_q = pid_f_q;
    esp_foc_pid_iq31_reset(&pid_q_d);
    esp_foc_pid_iq31_reset(&pid_q_q);

    esp_foc_low_pass_filter_set_cutoff(&filt_f_d, 60.0f, 2000.0f);
    esp_foc_low_pass_filter_set_cutoff(&filt_f_q, 60.0f, 2000.0f);
    esp_foc_low_pass_filter_set_cutoff_iq31(&filt_q_d, 60.0f, 2000.0f);
    esp_foc_low_pass_filter_set_cutoff_iq31(&filt_q_q, 60.0f, 2000.0f);

    for (int k = 0; k < 48; k++) {
        float t = (float)k / 48.0f;
        float angle = 2.0f * (float)M_PI * t;
        float iu = 0.55f * sinf(angle);
        float iv = 0.55f * sinf(angle - 2.0f * (float)M_PI / 3.0f);
        float iw = -iu - iv;
        float id_ref = 0.0f;
        float iq_ref = 0.35f * sinf(0.5f * angle);
        float du_f, dv_f, dw_f, du_q, dv_q, dw_q;

        run_manual_pipeline_step(angle, iu, iv, iw, id_ref, iq_ref,
                                 &pid_f_d, &pid_f_q, &filt_f_d, &filt_f_q,
                                 &pid_q_d, &pid_q_q, &filt_q_d, &filt_q_q,
                                 &du_f, &dv_f, &dw_f, &du_q, &dv_q, &dw_q);

        TEST_ASSERT_FLOAT_WITHIN(GOLDEN_TOL_DUTY, du_f, du_q);
        TEST_ASSERT_FLOAT_WITHIN(GOLDEN_TOL_DUTY, dv_f, dv_q);
        TEST_ASSERT_FLOAT_WITHIN(GOLDEN_TOL_DUTY, dw_f, dw_q);
    }
}

TEST_CASE("golden pipeline iq31: saturation and low vbus stay bounded", "[espFoC][iq31][golden]")
{
    iq31_t da, db, dc;
    esp_foc_svm_set_iq31(iq31_from_float(0.98f), iq31_from_float(-0.98f), iq31_from_float(0.05f), &da, &db, &dc);
    TEST_ASSERT_TRUE(iq31_to_float(da) >= 0.0f && iq31_to_float(da) <= 1.0f);
    TEST_ASSERT_TRUE(iq31_to_float(db) >= 0.0f && iq31_to_float(db) <= 1.0f);
    TEST_ASSERT_TRUE(iq31_to_float(dc) >= 0.0f && iq31_to_float(dc) <= 1.0f);
}

