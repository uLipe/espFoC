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
#include "espFoC/esp_foc_simu_observer.h"

#define SIMUL_FLUX_LINKAGE 0.015f // Lambda_m (Weber)
#define SIMUL_INERTIA 0.00024f // J (kg⋅m²)
#define SIMUL_FRICTION 0.0001f // B (Viscous friction)
#define DRIFT_COMP_CYCLE 200000
#define SIMUL_MAX_IQ 5.0f
#define SIMUL_MINIMUM_DIQ 1e-6f

typedef struct {
    float r;
    float l;
    float angle;
    float omega;
    float estim_iq;
    float dt;
    esp_foc_observer_t interface;
}esp_foc_simul_observer_t;

static const char * TAG = "ESP-FOC-SIMU-OBS";
DRAM_ATTR static  esp_foc_simul_observer_t simu_observers[CONFIG_NOOF_AXIS];

IRAM_ATTR static int simu_observer_update(esp_foc_observer_t *self, esp_foc_observer_inputs_t * in)
{
    esp_foc_simul_observer_t *obj = __containerof(self, esp_foc_simul_observer_t, interface);

    float d_iq = ((in->u_dq[1] - obj->r * obj->estim_iq - SIMUL_FLUX_LINKAGE * obj->omega) / obj->l);

    if(isnan(d_iq)) {
        d_iq = 0.0f;
    }

    if(fabsf(d_iq) < SIMUL_MINIMUM_DIQ) {
        d_iq = 0.0f;
    }

    if(d_iq > SIMUL_MAX_IQ) {
        d_iq = SIMUL_MAX_IQ;
    } else if (d_iq < -SIMUL_MAX_IQ) {
        d_iq = -SIMUL_MAX_IQ;
    }

    obj->estim_iq +=  d_iq * obj->dt;

    if(isnan(obj->estim_iq)) {
        obj->estim_iq = 0.0f;
    }

    if(obj->estim_iq > SIMUL_MAX_IQ) {
        obj->estim_iq = SIMUL_MAX_IQ;
    } else if (obj->estim_iq < -SIMUL_MAX_IQ) {
        obj->estim_iq = -SIMUL_MAX_IQ;
    }

    float torque = SIMUL_FLUX_LINKAGE * obj->estim_iq;
    float acceleration = (torque - SIMUL_FRICTION * obj->omega) / SIMUL_INERTIA;
    obj->omega += acceleration * obj->dt;

    if(isnan(obj->omega)) {
        obj->omega = 0.0f;
    }

    obj->angle += obj->omega * obj->dt;

    if(isnan(obj->angle)) {
        obj->angle = 0.0f;
    }

    if (obj->angle > (2.0f * M_PI)) {
        obj->angle -= (2.0f * M_PI);
    } else if (obj->angle < 0.0f) {
        obj->angle += (2.0f * M_PI);
    }

    return 0;
}

IRAM_ATTR static float simu_observer_get_angle(esp_foc_observer_t *self)
{
    esp_foc_simul_observer_t *obj = __containerof(self, esp_foc_simul_observer_t, interface);
    return obj->angle;
}

IRAM_ATTR static float simu_observer_get_speed(esp_foc_observer_t *self)
{
    esp_foc_simul_observer_t *obj = __containerof(self, esp_foc_simul_observer_t, interface);
    return obj->omega;
}

IRAM_ATTR static void simu_observer_reset(esp_foc_observer_t *self)
{
    esp_foc_simul_observer_t *obj = __containerof(self, esp_foc_simul_observer_t, interface);
    obj->angle = 0.0f;
    obj->estim_iq = 0.0f;
    obj->omega = 0.0f;
}

esp_foc_observer_t *simu_observer_new(int unit, esp_foc_simu_observer_settings_t settings)
{
    if(unit >= CONFIG_NOOF_AXIS) {
        ESP_LOGE(TAG, "Invalid unit!");
        return NULL;
    }

    if(settings.dt == 0.0f) {
        ESP_LOGE(TAG, "Invalid dt!");
        return NULL;
    }

    if(settings.phase_resistance <= 0.0f) {
        ESP_LOGE(TAG, "Invalid phase resistance !");
        return NULL;
    }

    if(settings.phase_inductance <= 0.0f) {
        ESP_LOGE(TAG, "Invalid phase Inductance !");
        return NULL;
    }

    simu_observers[unit].interface.update = simu_observer_update;
    simu_observers[unit].interface.get_angle = simu_observer_get_angle;
    simu_observers[unit].interface.get_speed = simu_observer_get_speed;
    simu_observers[unit].interface.get_speed = simu_observer_get_speed;
    simu_observers[unit].interface.reset = simu_observer_reset;
    simu_observers[unit].r = settings.phase_resistance;
    simu_observers[unit].l = settings.phase_inductance;
    simu_observers[unit].dt = settings.dt;
    simu_observers[unit].omega = 0.0f;
    simu_observers[unit].angle = 0.0f;
    simu_observers[unit].estim_iq = 0.0f;

    ESP_LOGI(TAG, "Base rate of the observer: %f", 1.0f / settings.dt);

    return &simu_observers[unit].interface;
}
