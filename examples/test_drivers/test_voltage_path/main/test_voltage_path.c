/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#include "esp_log.h"
#include "esp_err.h"
#include <math.h>
#include <assert.h>

#include "espFoC/esp_foc.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/foc_math_q16.h"
#include "espFoC/utils/modulator.h"

static const char *TAG = "esp-foc-example";

void test_vdq_pipeline_single(float vd_in,
                              float vq_in,
                              float theta,
                              float vbus,
                              float vmax)
{
    q16_t vd = q16_from_float(vd_in);
    q16_t vq = q16_from_float(vq_in);
    q16_t e_sin = q16_sin(q16_from_float(theta));
    q16_t e_cos = q16_cos(q16_from_float(theta));
    q16_t valpha, vbeta;
    q16_t da, db, dc;

    esp_foc_modulate_dq_voltage(e_sin, e_cos,
                                vd, vq,
                                &valpha, &vbeta,
                                &da, &db, &dc,
                                q16_from_float(vmax),
                                q16_from_float(vbus / 2.0f),
                                q16_from_float(1.0f / vbus));

    float da_f = q16_to_float(da);
    float db_f = q16_to_float(db);
    float dc_f = q16_to_float(dc);

    float va_rec = (da_f - 0.5f) * vbus;
    float vb_rec = (db_f - 0.5f) * vbus;
    float vc_rec = (dc_f - 0.5f) * vbus;

    const float INV_SQRT3 = 0.57735026919f;

    float v_alpha_rec = (2.0f/3.0f) * va_rec
                    - (1.0f/3.0f) * vb_rec
                    - (1.0f/3.0f) * vc_rec;

    float v_beta_rec  = INV_SQRT3 * (vb_rec - vc_rec);

    float e_sin_f = sinf(theta);
    float e_cos_f = cosf(theta);
    float vd_rec =  v_alpha_rec * e_cos_f + v_beta_rec * e_sin_f;
    float vq_rec = -v_alpha_rec * e_sin_f + v_beta_rec * e_cos_f;

    float r_in  = sqrtf(vd_in * vd_in + vq_in * vq_in);
    float r_rec = sqrtf(vd_rec * vd_rec + vq_rec * vq_rec);

    ESP_LOGI(TAG, "Vec expected: %f   Vec calculated: %f", (double)r_in, (double)r_rec);
    assert(fabsf(r_in - r_rec) < 0.5f);

    if (r_in > 1e-6f && r_rec > 1e-6f) {
        float udx = vd_in  / r_in;
        float udy = vq_in  / r_in;
        float urx = vd_rec / r_rec;
        float ury = vq_rec / r_rec;
        float dot = udx * urx + udy * ury;
        assert(dot > 0.95f);
    }
}


void app_main(void)
{
    float vbus = 24.0f;
    float vmax = vbus / 1.7320508075688772f;

    test_vdq_pipeline_single( 0.0f,       0.0f,        0.0f,   vbus, vmax);
    test_vdq_pipeline_single( 0.3f*vmax,  0.4f*vmax,   0.7f,   vbus, vmax);
    test_vdq_pipeline_single(-0.5f*vmax,  0.2f*vmax,   2.0f,   vbus, vmax);
    test_vdq_pipeline_single( 0.0f,       0.6f*vmax,   1.0f,   vbus, vmax);
    ESP_LOGI(TAG, "TEST PASSED!");
}
