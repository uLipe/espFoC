/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_FOC_ADRC_DELAY_MAX 3u

/**
 * First-order linear ADRC in Q16.16, for a plant dy/dt = −a·y + b0·u + f.
 * a is the known plant pole (Rs/Ls for a current loop, 0 for a pure
 * integrator) and f lumps what is not modelled (back-EMF, resistance error,
 * dead-time, cross-coupling).
 *
 * The pole has to be modelled when it is faster than the ESO: f = −a·y then
 * moves as fast as y itself, the ESO cannot follow it and only the weak
 * proportional term is left.
 *
 * Per sample, exact ZOH: y+ = α·y + g·u + z2, α = exp(−a·Ts),
 * g = b0·(1 − α)/a (b0·Ts when a = 0), z2 the unmodelled part. A two-state
 * ESO (current-estimator form, both poles at exp(−wo·Ts)) estimates y and z2.
 *
 * delay is the number of samples between the instant the measurement was
 * taken and the instant the input computed from it starts acting. The ESO
 * corrects at the measurement instant, then predicts delay samples ahead
 * with the inputs already applied; the control law places that prediction
 * on a first-order response of bandwidth wc:
 *
 *     u = (kp·(r − y_pred) + (1 − α)·y_pred − z2) / g,  kp = 1 − exp(−wc·Ts)
 *
 * update() does not saturate. Feed the input the plant really got back with
 * set_applied() once per update(); the ESO then never winds up under a clamp.
 */
typedef struct {
    q16_t l1;
    q16_t l2;
    q16_t kp;
    q16_t alpha;
    q16_t one_m_alpha;
    q16_t g;
    q16_t inv_g;
    q16_t z1;                               /* y at the last measurement */
    q16_t z2;                               /* unmodelled part, per sample */
    q16_t hist[ESP_FOC_ADRC_DELAY_MAX + 1]; /* applied inputs, newest first */
    uint32_t delay;
    float b0;
    float a_pole;
    float wc_hz;
    float wo_hz;
    float ts;
} esp_foc_adrc_t;

/**
 * b0 in output units per second per input unit, a_pole in 1/s (>= 0).
 * wc_hz is the closed-loop bandwidth, wo_hz the ESO's; wo_hz > wc_hz and
 * both below fs/2.
 */
esp_err_t esp_foc_adrc_init(esp_foc_adrc_t *a, float b0, float a_pole, float wc_hz, float wo_hz,
                            float ts, uint32_t delay);
void esp_foc_adrc_reset(esp_foc_adrc_t *a);

static inline q16_t esp_foc_adrc_sat(int64_t x)
{
    return (x > INT32_MAX) ? (q16_t)INT32_MAX : ((x < INT32_MIN) ? (q16_t)INT32_MIN : (q16_t)x);
}

/* Inline: two axes per PWM period. One saturation per state, sums in 64 bits. */
static inline q16_t esp_foc_adrc_update(esp_foc_adrc_t *a, q16_t ref, q16_t meas)
{
    const int64_t l1 = a->l1;
    const int64_t l2 = a->l2;
    const int64_t kp = a->kp;
    const int64_t alpha = a->alpha;
    const int64_t oma = a->one_m_alpha;
    const int64_t g = a->g;
    const int64_t inv_g = a->inv_g;
    const uint32_t delay = a->delay;
    const int64_t u_last = a->hist[delay];   /* acted between the last two measurements */
    const int64_t z1_old = a->z1;
    const int64_t z2_old = a->z2;

    const int64_t z1p = ((alpha * z1_old) >> 16) + z2_old + ((g * u_last) >> 16);
    const int64_t e = (int64_t)esp_foc_adrc_sat((int64_t)meas - z1p);
    const q16_t z1 = esp_foc_adrc_sat(z1p + ((l1 * e) >> 16));
    const q16_t z2 = esp_foc_adrc_sat(z2_old + ((l2 * e) >> 16));

    int64_t y = z1;
    for (uint32_t j = delay; j > 0u; j--) {
        y = (int64_t)esp_foc_adrc_sat(((alpha * y) >> 16) + ((g * a->hist[j - 1u]) >> 16) + z2);
    }
    const int64_t err = (int64_t)esp_foc_adrc_sat((int64_t)ref - y);
    const int64_t v = (int64_t)esp_foc_adrc_sat(((kp * err) >> 16) + ((oma * y) >> 16) -
                                                (int64_t)z2);

    a->z1 = z1;
    a->z2 = z2;
    return esp_foc_adrc_sat((v * inv_g) >> 16);
}

_Static_assert(ESP_FOC_ADRC_DELAY_MAX == 3u, "set_applied shifts a four-entry history");

static inline void esp_foc_adrc_set_applied(esp_foc_adrc_t *a, q16_t u_applied)
{
    const q16_t h0 = a->hist[0];
    const q16_t h1 = a->hist[1];
    const q16_t h2 = a->hist[2];
    a->hist[3] = h2;
    a->hist[2] = h1;
    a->hist[1] = h0;
    a->hist[0] = u_applied;
}

#ifdef __cplusplus
}
#endif
