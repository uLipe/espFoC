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
#include "espFoC/observer/esp_foc_simu_observer.h"


typedef struct {
    float angle;
    float omega;
    float alpha;
    float dt;
    esp_foc_observer_t interface;
}esp_foc_simul_observer_t;

static const char * TAG = "ESP-FOC-SIMU-OBS";
static  esp_foc_simul_observer_t simu_observers[CONFIG_NOOF_AXIS];

static int simu_observer_update(esp_foc_observer_t *self, esp_foc_observer_inputs_t * in)
{
    esp_foc_simul_observer_t *obj = __containerof(self, esp_foc_simul_observer_t, interface);

    obj->omega += obj->alpha * obj->dt;
    obj->angle += obj->omega * obj->dt;
    obj->angle = esp_foc_normalize_angle(obj->angle);

    return 0;
}

static float simu_observer_get_angle(esp_foc_observer_t *self)
{
    esp_foc_simul_observer_t *obj = __containerof(self, esp_foc_simul_observer_t, interface);
    return obj->angle;
}

static float simu_observer_get_speed(esp_foc_observer_t *self)
{
    esp_foc_simul_observer_t *obj = __containerof(self, esp_foc_simul_observer_t, interface);
    return obj->omega;
}

static void simu_observer_reset(esp_foc_observer_t *self, float offset)
{
    (void)offset;
    esp_foc_simul_observer_t *obj = __containerof(self, esp_foc_simul_observer_t, interface);
    obj->angle = 0.0f;
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

    if(settings.alpha <= 0.0f) {
        ESP_LOGE(TAG, "Invalid acceleration !");
        return NULL;
    }

    simu_observers[unit].interface.update = simu_observer_update;
    simu_observers[unit].interface.get_angle = simu_observer_get_angle;
    simu_observers[unit].interface.get_speed = simu_observer_get_speed;
    simu_observers[unit].interface.get_speed = simu_observer_get_speed;
    simu_observers[unit].interface.reset = simu_observer_reset;
    simu_observers[unit].dt = settings.dt;
    simu_observers[unit].omega = 0.0f;
    simu_observers[unit].angle = 0.0f;
    simu_observers[unit].alpha = settings.alpha;

    ESP_LOGI(TAG, "Base rate of the observer: %f", 1.0f / settings.dt);

    return &simu_observers[unit].interface;
}
