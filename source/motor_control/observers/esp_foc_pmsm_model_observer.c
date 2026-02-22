// esp_foc_pmsm_model_observer.c
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

#include <math.h>
#include <esp_attr.h>
#include <sdkconfig.h>
#include <sys/cdefs.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "espFoC/observer/esp_foc_pmsm_model_observer.h"

static const char *TAG = "ESP-FOC-PMSM-MODEL";

typedef struct {
    // Params
    float R, Ld, Lq, lambda;
    float J, B, Tl;
    float pp;
    float dt;

    // Optional clamps
    float i_lim;
    float w_lim;

    // State
    float id, iq;        // dq currents [A]
    float omega_m;       // mechanical speed [rad/s]
    float theta_m;       // mechanical angle [rad] (0..2pi)

    // Cached outputs
    float torque;

    // Interface
    esp_foc_observer_t interface;
} esp_foc_pmsm_model_observer_t;

typedef struct {
    float did, diq, dw, dtheta;
} derivs_t;

static DRAM_ATTR esp_foc_pmsm_model_observer_t pmsm_observers[CONFIG_NOOF_AXIS];

static inline float wrap_0_2pi(float a)
{
    return esp_foc_normalize_angle(a);
}

static inline float pmsm_torque_from_states(const esp_foc_pmsm_model_observer_t *m, float id, float iq)
{
    // Te = (3/2)*p*( lambda*iq + (Ld-Lq)*id*iq )
    return 1.5f * m->pp * (m->lambda * iq + (m->Ld - m->Lq) * id * iq);
}

static inline void clamp_state(esp_foc_pmsm_model_observer_t *m)
{
    if (m->i_lim > 0.0f) {
        if (m->id >  m->i_lim) m->id =  m->i_lim;
        if (m->id < -m->i_lim) m->id = -m->i_lim;
        if (m->iq >  m->i_lim) m->iq =  m->i_lim;
        if (m->iq < -m->i_lim) m->iq = -m->i_lim;
    }
    if (m->w_lim > 0.0f) {
        if (m->omega_m >  m->w_lim) m->omega_m =  m->w_lim;
        if (m->omega_m < -m->w_lim) m->omega_m = -m->w_lim;
    }
}

static inline derivs_t pmsm_derivs(const esp_foc_pmsm_model_observer_t *m,
                                  float id, float iq, float omega_m,
                                  float vd, float vq)
{
    derivs_t d;

    const float we = m->pp * omega_m;

    // Electrical dynamics:
    d.did = (vd - m->R * id + we * m->Lq * iq) / m->Ld;
    d.diq = (vq - m->R * iq - we * (m->Ld * id + m->lambda)) / m->Lq;

    // Mechanical dynamics:
    const float Te = pmsm_torque_from_states(m, id, iq);
    d.dw = (Te - m->Tl - m->B * omega_m) / m->J;

    // Angle:
    d.dtheta = omega_m;

    return d;
}

static inline void pmsm_step_rk4(esp_foc_pmsm_model_observer_t *m, float vd, float vq)
{
    const float h = m->dt;

    // k1
    derivs_t k1 = pmsm_derivs(m, m->id, m->iq, m->omega_m, vd, vq);

    // k2
    const float id2 = m->id + 0.5f * h * k1.did;
    const float iq2 = m->iq + 0.5f * h * k1.diq;
    const float w2  = m->omega_m + 0.5f * h * k1.dw;
    derivs_t k2 = pmsm_derivs(m, id2, iq2, w2, vd, vq);

    // k3
    const float id3 = m->id + 0.5f * h * k2.did;
    const float iq3 = m->iq + 0.5f * h * k2.diq;
    const float w3  = m->omega_m + 0.5f * h * k2.dw;
    derivs_t k3 = pmsm_derivs(m, id3, iq3, w3, vd, vq);

    // k4
    const float id4 = m->id + h * k3.did;
    const float iq4 = m->iq + h * k3.diq;
    const float w4  = m->omega_m + h * k3.dw;
    derivs_t k4 = pmsm_derivs(m, id4, iq4, w4, vd, vq);

    m->id      += (h / 6.0f) * (k1.did + 2.0f * k2.did + 2.0f * k3.did + k4.did);
    m->iq      += (h / 6.0f) * (k1.diq + 2.0f * k2.diq + 2.0f * k3.diq + k4.diq);
    m->omega_m += (h / 6.0f) * (k1.dw  + 2.0f * k2.dw  + 2.0f * k3.dw  + k4.dw);
    m->theta_m += (h / 6.0f) * (k1.dtheta + 2.0f * k2.dtheta + 2.0f * k3.dtheta + k4.dtheta);

    m->theta_m = wrap_0_2pi(m->theta_m);
    m->torque  = pmsm_torque_from_states(m, m->id, m->iq);

    clamp_state(m);
}

static int pmsm_model_observer_update(esp_foc_observer_t *self, esp_foc_observer_inputs_t *in)
{
    esp_foc_pmsm_model_observer_t *m = __containerof(self, esp_foc_pmsm_model_observer_t, interface);

    // Always use u_alpha_beta (robust).
    const float theta_e = wrap_0_2pi(m->pp * m->theta_m);

    const float sinT = esp_foc_sine(theta_e);
    const float cosT = esp_foc_cosine(theta_e);

    const float u_alpha = in->u_alpha_beta[0];
    const float u_beta  = in->u_alpha_beta[1];

    // Park (alpha/beta -> d/q)
    const float vd = (u_alpha * cosT) + (u_beta * sinT);
    const float vq = (-u_alpha * sinT) + (u_beta * cosT);

    // Always integrate RK4
    pmsm_step_rk4(m, vd, vq);

    return 0;
}

static float pmsm_model_observer_get_angle(esp_foc_observer_t *self)
{
    esp_foc_pmsm_model_observer_t *m = __containerof(self, esp_foc_pmsm_model_observer_t, interface);
    return m->theta_m; // mechanical
}

static float pmsm_model_observer_get_speed(esp_foc_observer_t *self)
{
    esp_foc_pmsm_model_observer_t *m = __containerof(self, esp_foc_pmsm_model_observer_t, interface);
    return m->omega_m; // mechanical
}

static void pmsm_model_observer_reset(esp_foc_observer_t *self, float offset)
{
    esp_foc_pmsm_model_observer_t *m = __containerof(self, esp_foc_pmsm_model_observer_t, interface);
    m->theta_m = wrap_0_2pi(offset);
    m->omega_m = 0.0f;
    m->id = 0.0f;
    m->iq = 0.0f;
    m->torque = 0.0f;
}

esp_foc_observer_t *pmsm_model_observer_new(int unit, esp_foc_pmsm_model_observer_settings_t s)
{
    if (unit >= CONFIG_NOOF_AXIS) {
        ESP_LOGE(TAG, "Invalid unit!");
        return NULL;
    }

    if (s.dt <= 0.0f) {
        ESP_LOGE(TAG, "Invalid dt!");
        return NULL;
    }

    if (s.phase_resistance <= 0.0f) {
        ESP_LOGE(TAG, "Invalid phase resistance!");
        return NULL;
    }

    if (s.Ld <= 0.0f || s.Lq <= 0.0f) {
        ESP_LOGE(TAG, "Invalid inductance!");
        return NULL;
    }

    if (s.inertia <= 0.0f) {
        ESP_LOGE(TAG, "Invalid inertia!");
        return NULL;
    }

    if (s.pole_pairs <= 0.0f) {
        ESP_LOGE(TAG, "Invalid pole pairs!");
        return NULL;
    }

    if (s.lambda <= 0.0f) {
        ESP_LOGE(TAG, "Invalid lambda (flux linkage)!");
        return NULL;
    }

    if (s.friction < 0.0f) {
        ESP_LOGE(TAG, "Invalid friction!");
        return NULL;
    }

    esp_foc_pmsm_model_observer_t *m = &pmsm_observers[unit];

    m->R = s.phase_resistance;
    m->Ld = s.Ld;
    m->Lq = s.Lq;
    m->lambda = s.lambda;
    m->J = s.inertia;
    m->B = s.friction;
    m->Tl = s.load_torque;
    m->pp = s.pole_pairs;
    m->dt = s.dt;

    m->i_lim = s.current_limit;
    m->w_lim = s.omega_limit;

    // Reset state
    m->theta_m = 0.0f;
    m->omega_m = 0.0f;
    m->id = 0.0f;
    m->iq = 0.0f;
    m->torque = 0.0f;

    // Bind interface
    m->interface.update = pmsm_model_observer_update;
    m->interface.get_angle = pmsm_model_observer_get_angle;
    m->interface.get_speed = pmsm_model_observer_get_speed;
    m->interface.reset = pmsm_model_observer_reset;

    ESP_LOGI(TAG,
             "PMSM model: dt=%f Hz=%f pp=%f R=%f Ld=%f Lq=%f lambda=%f (RK4, u_ab)",
             m->dt, (1.0f / m->dt), m->pp, m->R, m->Ld, m->Lq, m->lambda);

    return &m->interface;
}