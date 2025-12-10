/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "esp_log.h"
#include "esp_err.h"
#include <math.h>
#include <assert.h>

#include "espFoC/esp_foc.h"

static const char *TAG = "esp-foc-example";

void test_vdq_pipeline_single(float vd_in,
                              float vq_in,
                              float theta,
                              float vbus,
                              float vmax)
{
    float vd = vd_in;
    float vq = vq_in;
    float valpha;
    float vbeta;
    float da, db, dc;
    float e_sin = esp_foc_sine(theta);
    float e_cos = esp_foc_cosine(theta);

    // clamp in dq
    // Inverse Park → αβ
    // SVPWM
    esp_foc_modulate_dq_voltage(e_sin, e_cos,
                                vd, vq,
                                &valpha, &vbeta,
                                &da, &db, &dc,
                                vmax,
                                vbus / 2.0f,
                                1.0f / vbus);

    // reconstruct phase voltages from duties
    float va_rec = (da - 0.5f) * vbus;
    float vb_rec = (db - 0.5f) * vbus;
    float vc_rec = (dc - 0.5f) * vbus;

    // alpha/beta reconstruction (full Clarke: abc → αβ)
    const float INV_SQRT3 = 0.57735026919f; // 1/√3

    float v_alpha_rec = (2.0f/3.0f) * va_rec
                    - (1.0f/3.0f) * vb_rec
                    - (1.0f/3.0f) * vc_rec;

    float v_beta_rec  = INV_SQRT3 * (vb_rec - vc_rec);

    // Park back to dq
    float vd_rec =  v_alpha_rec * e_cos + v_beta_rec * e_sin;
    float vq_rec = -v_alpha_rec * e_sin + v_beta_rec * e_cos;

    // Now comparisons:

    float r_in  = sqrtf(vd * vd + vq * vq);
    float r_rec = sqrtf(vd_rec * vd_rec + vq_rec * vq_rec);

    // magnitude should match (limited)
    ESP_LOGI(TAG, "Vec expected: %f   Vec calculated: %f", r_in, r_rec);
    assert(fabsf(r_in - r_rec) < 1e-3f);

    // direction: unit vectors comparison
    if (r_in > 1e-6f && r_rec > 1e-6f) {
        float udx = vd     / r_in;
        float udy = vq     / r_in;
        float urx = vd_rec / r_rec;
        float ury = vq_rec / r_rec;
        float dot = udx * urx + udy * ury;
        // cos(angle_err) ≈ 1  → angle_err small
        assert(dot > 0.999f); // ~ <2.5° error
    }
}


void app_main(void)
{
    float vbus = 24.0f;
    float vmax = vbus / 1.7320508075688772f; // SVPWM linear

    // inside limit
    test_vdq_pipeline_single( 0.0f,       0.0f,        0.0f,   vbus, vmax);
    test_vdq_pipeline_single( 0.3f*vmax,  0.4f*vmax,   0.7f,   vbus, vmax);
    test_vdq_pipeline_single(-0.5f*vmax,  0.2f*vmax,   2.0f,   vbus, vmax);
    test_vdq_pipeline_single( 0.0f,       0.6f*vmax,   1.0f,   vbus, vmax);

    //outside limit (should clamp)
    //test_vdq_pipeline_single( 2.0f*vmax,  0.0f,        0.0f,   vbus, vmax);
    //test_vdq_pipeline_single( 1.2f*vmax,  1.2f*vmax,   1.3f,   vbus, vmax);
    //test_vdq_pipeline_single(-2.0f*vmax, -1.0f*vmax,   2.7f,   vbus, vmax);
    ESP_LOGI(TAG, "TEST PASSED!");
}