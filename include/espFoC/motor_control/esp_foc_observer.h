/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct esp_foc_observer_s esp_foc_observer_t;

typedef enum {
    ESP_FOC_ANGLE_ATAN2 = 0,
    ESP_FOC_ANGLE_PLL = 1,
} esp_foc_angle_extract_t;

/**
 * Sensorless observer iface (electrical θ̂, ω̂). Hot path is O(1) Q16.16.
 *
 * Concrete objects embed this struct first and bind a vtable. Callers talk
 * only to esp_foc_observer_t * — BEMF vs flux is a create-time choice.
 *
 * update() takes measured iαβ [A] and applied vαβ [V]. Command speed ω*
 * belongs in the speed PI, not in the observer.
 */
struct esp_foc_observer_s {
    void (*update)(esp_foc_observer_t *self,
                   q16_t i_alpha,
                   q16_t i_beta,
                   q16_t v_alpha,
                   q16_t v_beta);
    void (*reset)(esp_foc_observer_t *self);
    void (*set_theta)(esp_foc_observer_t *self, q16_t theta_e);
    void (*set_omega)(esp_foc_observer_t *self, q16_t omega_e);
    void (*set_extract)(esp_foc_observer_t *self, esp_foc_angle_extract_t extract);
    void (*set_pll_enable)(esp_foc_observer_t *self, bool enable);
    q16_t (*get_theta)(const esp_foc_observer_t *self);
    q16_t (*get_omega)(const esp_foc_observer_t *self);
    q16_t (*get_e_alpha)(const esp_foc_observer_t *self);
    q16_t (*get_e_beta)(const esp_foc_observer_t *self);
    q16_t (*get_psi_alpha)(const esp_foc_observer_t *self);
    q16_t (*get_psi_beta)(const esp_foc_observer_t *self);
    q16_t (*get_phase_err)(const esp_foc_observer_t *self);
    bool (*is_locked)(const esp_foc_observer_t *self);
};

static inline void esp_foc_observer_update(esp_foc_observer_t *o,
                                           q16_t i_alpha,
                                           q16_t i_beta,
                                           q16_t v_alpha,
                                           q16_t v_beta)
{
    if (o != NULL && o->update != NULL) {
        o->update(o, i_alpha, i_beta, v_alpha, v_beta);
    }
}

static inline void esp_foc_observer_reset(esp_foc_observer_t *o)
{
    if (o != NULL && o->reset != NULL) {
        o->reset(o);
    }
}

static inline void esp_foc_observer_set_theta(esp_foc_observer_t *o, q16_t theta_e)
{
    if (o != NULL && o->set_theta != NULL) {
        o->set_theta(o, theta_e);
    }
}

static inline void esp_foc_observer_set_omega(esp_foc_observer_t *o, q16_t omega_e)
{
    if (o != NULL && o->set_omega != NULL) {
        o->set_omega(o, omega_e);
    }
}

static inline void esp_foc_observer_set_extract(esp_foc_observer_t *o,
                                                esp_foc_angle_extract_t extract)
{
    if (o != NULL && o->set_extract != NULL) {
        o->set_extract(o, extract);
    }
}

static inline void esp_foc_observer_set_pll_enable(esp_foc_observer_t *o, bool enable)
{
    if (o != NULL && o->set_pll_enable != NULL) {
        o->set_pll_enable(o, enable);
    }
}

static inline q16_t esp_foc_observer_get_theta(const esp_foc_observer_t *o)
{
    return (o != NULL && o->get_theta != NULL) ? o->get_theta(o) : 0;
}

static inline q16_t esp_foc_observer_get_omega(const esp_foc_observer_t *o)
{
    return (o != NULL && o->get_omega != NULL) ? o->get_omega(o) : 0;
}

static inline q16_t esp_foc_observer_get_e_alpha(const esp_foc_observer_t *o)
{
    return (o != NULL && o->get_e_alpha != NULL) ? o->get_e_alpha(o) : 0;
}

static inline q16_t esp_foc_observer_get_e_beta(const esp_foc_observer_t *o)
{
    return (o != NULL && o->get_e_beta != NULL) ? o->get_e_beta(o) : 0;
}

static inline q16_t esp_foc_observer_get_psi_alpha(const esp_foc_observer_t *o)
{
    return (o != NULL && o->get_psi_alpha != NULL) ? o->get_psi_alpha(o) : 0;
}

static inline q16_t esp_foc_observer_get_psi_beta(const esp_foc_observer_t *o)
{
    return (o != NULL && o->get_psi_beta != NULL) ? o->get_psi_beta(o) : 0;
}

static inline q16_t esp_foc_observer_get_phase_err(const esp_foc_observer_t *o)
{
    return (o != NULL && o->get_phase_err != NULL) ? o->get_phase_err(o) : 0;
}

static inline bool esp_foc_observer_is_locked(const esp_foc_observer_t *o)
{
    return o != NULL && o->is_locked != NULL && o->is_locked(o);
}

#ifdef __cplusplus
}
#endif
