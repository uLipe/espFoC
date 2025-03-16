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
#include "espFoC/esp_foc_pll_observer.h"

#define PLL_OBSERVER_CONVERGE_WAIT_TIME  2.0f
#define SIMUL_FLUX_LINKAGE 0.015f // Lambda_m (Weber)
#define SIMUL_INERTIA 0.00024f // J (kg⋅m²)
#define SIMUL_FRICTION 0.0001f // B (Viscous friction

typedef struct {
    float theta_est;
    float theta_mes;
    float theta_mech;
    float omega_est;
    float omega_mech;
    float theta_error;
    float e_alpha;
    float e_beta;
    float i_alpha_prev;
    float i_beta_prev;
    float integral;
    float kp;
    float ki;
    float r;
    float l;
    float pp;
    float dt;
    float inv_dt;
    int converging_count;
    esp_foc_observer_t interface;
} angle_estimator_pll_t;

static const char * TAG = "ESP-FOC-PLL-OBS";
static DRAM_ATTR angle_estimator_pll_t pll_observers[CONFIG_NOOF_AXIS];

IRAM_ATTR static int pll_observer_update(esp_foc_observer_t *self, esp_foc_observer_inputs_t * in)
{
    angle_estimator_pll_t *est = __containerof(self, angle_estimator_pll_t, interface);

    float di_alpha_dt = (in->i_alpha_beta[0] - est->i_alpha_prev) * est->inv_dt;
    float di_beta_dt = (in->i_alpha_beta[1] - est->i_beta_prev)  * est->inv_dt;
    float torque = SIMUL_FLUX_LINKAGE * in->i_dq[1];
    float acceleration = (torque - SIMUL_FRICTION * est->omega_mech) / SIMUL_INERTIA;

    est->omega_mech += acceleration * est->dt;
    est->theta_mech += est->omega_mech * est->dt * est->pp;

    if (est->theta_mech > 2.0f * M_PI) est->theta_mech -= 2.0f * M_PI;
    if (est->theta_mech < 0.0f) est->theta_mech += 2.0f * M_PI;

    /* Estimate the angle using the motor parameters */
    est->e_alpha = in->u_alpha_beta[0] - est->r * in->i_alpha_beta[0] - est->l * di_alpha_dt;
    est->e_beta = in->u_alpha_beta[1] - est->r * in->i_alpha_beta[1]- est->l * di_beta_dt;
    est->theta_mes = atan2f(est->e_beta, est->e_alpha);

    if (est->theta_mes > 2.0f * M_PI) est->theta_mes -= 2.0f * M_PI;
    if (est->theta_mes < 0.0f) est->theta_mes += 2.0f * M_PI;

    /* Perform PLL step to filter out the noise and extracts the rotor speed as well*/
    est->theta_error = (est->theta_est * 1.0f *(est->theta_mech - est->theta_mes)) - est->theta_est;
    est->integral += est->theta_error * est->dt;
    est->omega_est = (est->kp * est->theta_error) + (est->ki * est->integral);
    est->theta_est += est->omega_est * est->dt;

    if (est->theta_est > 2.0f * M_PI) est->theta_est -= 2.0f * M_PI;
    if (est->theta_est < 0.0f) est->theta_est += 2.0f * M_PI;


    est->i_alpha_prev = in->i_alpha_beta[0];
    est->i_beta_prev = in->i_alpha_beta[1];

    if(est->converging_count) {
        est->converging_count--;
    }

    return est->converging_count;
}

IRAM_ATTR static float pll_observer_get_angle(esp_foc_observer_t *self)
{
    angle_estimator_pll_t *est = __containerof(self, angle_estimator_pll_t, interface);
    return est->theta_est;
}

IRAM_ATTR static float pll_observer_get_speed(esp_foc_observer_t *self)
{
    angle_estimator_pll_t *est = __containerof(self, angle_estimator_pll_t, interface);
    return est->omega_est;
}

IRAM_ATTR static void pll_observer_reset(esp_foc_observer_t *self, float offset)
{
    angle_estimator_pll_t *est = __containerof(self, angle_estimator_pll_t, interface);
    est->theta_est = offset;
}

esp_foc_observer_t *pll_observer_new(int unit, esp_foc_pll_observer_settings_t settings)
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

    if(settings.pll_kp <= 0.0f) {
        ESP_LOGE(TAG, "Invalid proportional gain !");
        return NULL;
    }

    if(settings.pll_ki < 0.0f) {
        ESP_LOGE(TAG, "Invalid integral gain !");
        return NULL;
    }

    angle_estimator_pll_t *est = &pll_observers[unit];
    est->theta_est = 0.0f;
    est->omega_est = 0.0f;
    est->omega_mech = 0.0f;
    est->theta_mech = 0.0f;
    est->theta_error = 0.0f;
    est->e_alpha = 0.0f;
    est->e_beta = 0.0f;
    est->i_alpha_prev = 0.0f;
    est->i_beta_prev = 0.0f;
    est->integral = 0.0f;
    est->kp = settings.pll_kp;
    est->ki = settings.pll_ki;
    est->r = settings.phase_resistance;
    est->l = settings.phase_inductance;
    est->pp = settings.pole_pairs;
    est->dt = settings.dt * 2.0f;
    est->inv_dt = (1.0f / est->dt);
    est->converging_count = (int)( PLL_OBSERVER_CONVERGE_WAIT_TIME / settings.dt);

    ESP_LOGI(TAG, "Observer sample time %f s", est->dt);

    est->interface.update = pll_observer_update;
    est->interface.get_angle = pll_observer_get_angle;
    est->interface.get_speed = pll_observer_get_speed;
    est->interface.reset = pll_observer_reset;

    return &est->interface;
}
