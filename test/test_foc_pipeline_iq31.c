/*
 * Q16.16 FOC pipeline tests: DQ → LPF → PID → modulation.
 */
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <unity.h>
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/foc_math_q16.h"
#include "espFoC/utils/modulator.h"
#include "espFoC/utils/pid_controller.h"
#include "espFoC/utils/ema_low_pass_filter.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define N_PIPELINE_STEPS 48

static void run_q16_pipeline_step(float angle_rad,
                                   float iu, float iv, float iw,
                                   float id_ref, float iq_ref,
                                   esp_foc_pid_controller_t *pid_d,
                                   esp_foc_pid_controller_t *pid_q,
                                   esp_foc_lp_filter_t *filt_d,
                                   esp_foc_lp_filter_t *filt_q,
                                   q16_t *du, q16_t *dv, q16_t *dw)
{
    q16_t sq = q16_sin(q16_from_float(angle_rad));
    q16_t cq = q16_cos(q16_from_float(angle_rad));
    q16_t ia_q, ib_q, id_q, iq_q;
    q16_t uq_q, ud_q;
    q16_t ua_q, ub_q;
    const q16_t vmax = q16_from_float(0.7f);
    const q16_t norm = q16_from_float(0.5f);

    esp_foc_get_dq_currents(sq, cq,
                                 q16_from_float(iu),
                                 q16_from_float(iv),
                                 q16_from_float(iw),
                                 &ia_q, &ib_q, &iq_q, &id_q);
    uq_q = esp_foc_pid_update(pid_q,
                                   q16_from_float(iq_ref),
                                   esp_foc_low_pass_filter_update(filt_q, iq_q));
    ud_q = esp_foc_pid_update(pid_d,
                                   q16_from_float(id_ref),
                                   esp_foc_low_pass_filter_update(filt_d, id_q));

    esp_foc_modulate_dq_voltage(sq, cq, ud_q, uq_q,
                                     &ua_q, &ub_q, du, dv, dw,
                                     vmax, norm);
}

static void setup_pid_filters(esp_foc_pid_controller_t *pid_d,
                              esp_foc_pid_controller_t *pid_q,
                              esp_foc_lp_filter_t *filt_d,
                              esp_foc_lp_filter_t *filt_q)
{
    const float dt = 1.0f / 2000.0f;
    esp_foc_pid_init_from_float(pid_d, 0.4f, 60.0f, 0.0f, dt, -0.7f, 0.7f, 0.7f / 60.0f);
    esp_foc_pid_init_from_float(pid_q, 0.4f, 60.0f, 0.0f, dt, -0.7f, 0.7f, 0.7f / 60.0f);

    esp_foc_low_pass_filter_set_cutoff(filt_d, 60.0f, 2000.0f);
    esp_foc_low_pass_filter_set_cutoff(filt_q, 60.0f, 2000.0f);
}

TEST_CASE("golden pipeline q16: deterministic run and bounded duties", "[espFoC][iq31][golden]")
{
    q16_t du[N_PIPELINE_STEPS], dv[N_PIPELINE_STEPS], dw[N_PIPELINE_STEPS];
    q16_t du2[N_PIPELINE_STEPS], dv2[N_PIPELINE_STEPS], dw2[N_PIPELINE_STEPS];

    for (int pass = 0; pass < 2; pass++) {
        esp_foc_pid_controller_t pid_d = {0};
        esp_foc_pid_controller_t pid_q = {0};
        esp_foc_lp_filter_t filt_d, filt_q;
        setup_pid_filters(&pid_d, &pid_q, &filt_d, &filt_q);

        for (int k = 0; k < N_PIPELINE_STEPS; k++) {
            float t = (float)k / (float)N_PIPELINE_STEPS;
            float angle = 2.0f * (float)M_PI * t;
            float iu = 0.55f * sinf(angle);
            float iv = 0.55f * sinf(angle - 2.0f * (float)M_PI / 3.0f);
            float iw = -iu - iv;
            float id_ref = 0.0f;
            float iq_ref = 0.35f * sinf(0.5f * angle);
            q16_t a, b, c;

            run_q16_pipeline_step(angle, iu, iv, iw, id_ref, iq_ref,
                                   &pid_d, &pid_q, &filt_d, &filt_q, &a, &b, &c);

            TEST_ASSERT_TRUE(q16_to_float(a) >= 0.0f && q16_to_float(a) <= 1.0f);
            TEST_ASSERT_TRUE(q16_to_float(b) >= 0.0f && q16_to_float(b) <= 1.0f);
            TEST_ASSERT_TRUE(q16_to_float(c) >= 0.0f && q16_to_float(c) <= 1.0f);

            if (pass == 0) {
                du[k] = a;
                dv[k] = b;
                dw[k] = c;
            } else {
                du2[k] = a;
                dv2[k] = b;
                dw2[k] = c;
            }
        }
    }

    TEST_ASSERT_EQUAL_INT(0, memcmp(du, du2, sizeof(du)));
    TEST_ASSERT_EQUAL_INT(0, memcmp(dv, dv2, sizeof(dv)));
    TEST_ASSERT_EQUAL_INT(0, memcmp(dw, dw2, sizeof(dw)));
}

TEST_CASE("golden pipeline q16: saturation and low vbus stay bounded", "[espFoC][iq31][golden]")
{
    q16_t da, db, dc;
    esp_foc_svm_set(q16_from_float(0.98f), q16_from_float(-0.98f), q16_from_float(0.05f), &da, &db, &dc);
    TEST_ASSERT_TRUE(q16_to_float(da) >= 0.0f && q16_to_float(da) <= 1.0f);
    TEST_ASSERT_TRUE(q16_to_float(db) >= 0.0f && q16_to_float(db) <= 1.0f);
    TEST_ASSERT_TRUE(q16_to_float(dc) >= 0.0f && q16_to_float(dc) <= 1.0f);
}
