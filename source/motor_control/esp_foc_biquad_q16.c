/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include <math.h>
#include "espFoC/utils/biquad_q16.h"

void esp_foc_biquad_q16_set_bypass(esp_foc_biquad_q16_t *f)
{
    if (f == NULL) {
        return;
    }
    f->b0 = Q16_ONE;
    f->b1 = 0;
    f->b2 = 0;
    f->a1 = 0;
    f->a2 = 0;
    f->s1 = 0;
    f->s2 = 0;
}

void esp_foc_biquad_butterworth_lpf_design_q16(esp_foc_biquad_q16_t *f,
                                               float fc_hz,
                                               float fs_hz)
{
    if (f == NULL) {
        return;
    }

    /* Reject configurations that cannot produce a stable filter so
     * callers are not silently handed garbage coefficients (which on
     * a current PI would saturate the integrator inside one cycle). */
    if (!(fs_hz > 0.0f) || !(fc_hz > 0.0f) || (fc_hz >= 0.5f * fs_hz)) {
        esp_foc_biquad_q16_set_bypass(f);
        return;
    }

    /* Standard 2nd-order Butterworth LPF via bilinear transform with
     * prewarping at fc. Continuous prototype:
     *
     *     H(s) = wa^2 / (s^2 + sqrt(2)*wa*s + wa^2)
     *
     * with wa = 2/T * tan(pi*fc/fs) so the digital -3 dB point lands
     * exactly on fc. Substituting s = (2/T)*(1-z^-1)/(1+z^-1) and
     * collecting terms gives, with r = tan(pi*fc/fs):
     *
     *     a0 = 1 + sqrt(2)*r + r^2
     *     b0 = b2 = r^2 / a0
     *     b1 = 2*r^2 / a0
     *     a1 = 2*(r^2 - 1) / a0
     *     a2 = (1 - sqrt(2)*r + r^2) / a0
     *
     * DC gain checks out (H(1) = 1) and there is a perfect zero at
     * Nyquist (H(-1) = 0). All coefficients live in [-2, +2], so Q16
     * (range +-32767) has ~14 bits of headroom. */
    const float pi = 3.14159265358979323846f;
    const float sqrt2 = 1.41421356237f;
    float r = tanf(pi * fc_hz / fs_hz);
    float r2 = r * r;
    float a0 = 1.0f + sqrt2 * r + r2;

    f->b0 = q16_from_float(r2 / a0);
    f->b1 = q16_from_float(2.0f * r2 / a0);
    f->b2 = q16_from_float(r2 / a0);
    f->a1 = q16_from_float(2.0f * (r2 - 1.0f) / a0);
    f->a2 = q16_from_float((1.0f - sqrt2 * r + r2) / a0);
    f->s1 = 0;
    f->s2 = 0;
}
