/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include "espFoC/drivers/esp_foc_inverter.h"
#include "espFoC/utils/esp_foc_park.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/esp_foc_svm.h"

#ifdef __cplusplus
extern "C" {
#endif

/** (alpha, beta) in pu of Vdc through SVM to the bridge. */
static inline void esp_foc_modulate_ab(esp_foc_inverter_t *inv, q16_t a, q16_t b)
{
    q16_t du;
    q16_t dv;
    q16_t dw;
    esp_foc_svm(a, b, &du, &dv, &dw);
    inv->set_duties(inv, du, dv, dw);
}

/*
 * Output tail of every current loop: (vd, vq) in pu of Vdc, already limited
 * by the caller, through inverse Park at (sin, cos) and SVM to the bridge.
 */
static inline void esp_foc_modulate(esp_foc_inverter_t *inv, q16_t s, q16_t c, q16_t vd,
                                    q16_t vq)
{
    q16_t a;
    q16_t b;
    esp_foc_inv_park(s, c, vd, vq, &a, &b);
    esp_foc_modulate_ab(inv, a, b);
}

/** Zero voltage vector: all three legs at 50 %. */
static inline void esp_foc_bridge_neutral(esp_foc_inverter_t *inv)
{
    inv->set_duties(inv, Q16_HALF, Q16_HALF, Q16_HALF);
}

#ifdef __cplusplus
}
#endif
