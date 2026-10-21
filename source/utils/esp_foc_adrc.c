/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * First-order linear ADRC. Float only in init().
 */
#include "espFoC/utils/esp_foc_adrc.h"

#include <math.h>
#include <string.h>

#include "espFoC/utils/esp_foc_angle.h"
#include "espFoC/utils/esp_foc_iir.h"

/* g and 1/g both have to be representable with some resolution left. */
#define ADRC_G_MIN    (1.0f / 256.0f)
#define ADRC_G_MAX    256.0f
/* alpha >= exp(-4): keeps beta^2 / alpha, and so l1, inside a few units. */
#define ADRC_ATS_MAX  4.0f

esp_err_t esp_foc_adrc_init(esp_foc_adrc_t *a, float b0, float a_pole, float wc_hz, float wo_hz,
                            float ts, uint32_t delay)
{
    if ((a == NULL) || !(ts > 0.0f) || !(b0 > 0.0f) || !(a_pole >= 0.0f) ||
        !(a_pole * ts <= ADRC_ATS_MAX) || !(wc_hz > 0.0f) || !(wo_hz > wc_hz) ||
        !(wo_hz < 0.5f / ts) || (delay > ESP_FOC_ADRC_DELAY_MAX)) {
        return ESP_ERR_INVALID_ARG;
    }
    const float ats = a_pole * ts;
    const float one_m_alpha = -expm1f(-ats);
    const float alpha = 1.0f - one_m_alpha;
    const float g = (ats > 1e-6f) ? (b0 * one_m_alpha / a_pole) : (b0 * ts);
    if ((g < ADRC_G_MIN) || (g > ADRC_G_MAX)) {
        return ESP_ERR_INVALID_ARG;
    }
    const float beta = expf(-ESP_FOC_TWO_PI_F * wo_hz * ts);

    memset(a, 0, sizeof(*a));
    /* Both ESO error poles at beta: det = (1 - l1)·alpha, trace = det + 1 - l2. */
    a->l1 = q16_from_float(1.0f - beta * beta / alpha);
    a->l2 = q16_from_float((1.0f - beta) * (1.0f - beta));
    a->kp = q16_from_float(esp_foc_lpf1_coef(wc_hz, ts));
    a->alpha = q16_from_float(alpha);
    /* Complement of the quantised alpha, so y = r is the exact fixed point. */
    a->one_m_alpha = Q16_ONE - a->alpha;
    a->g = q16_from_float(g);
    a->inv_g = q16_from_float(1.0f / g);
    a->delay = delay;
    a->b0 = b0;
    a->a_pole = a_pole;
    a->wc_hz = wc_hz;
    a->wo_hz = wo_hz;
    a->ts = ts;
    return ESP_OK;
}

void esp_foc_adrc_reset(esp_foc_adrc_t *a)
{
    a->z1 = 0;
    a->z2 = 0;
    memset(a->hist, 0, sizeof(a->hist));
}
