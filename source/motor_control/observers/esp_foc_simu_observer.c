/*
 * Simulated plant observer — IQ31 only in update() (ISR-safe).
 * Coefficients are computed in simu_observer_new() (float allowed once).
 */

#include <math.h>
#include <esp_attr.h>
#include <sdkconfig.h>
#include <sys/cdefs.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "espFoC/observer/esp_foc_observer_interface.h"
#include "espFoC/utils/foc_math_iq31.h"
#include "espFoC/observer/esp_foc_simu_observer.h"

#define SIMUL_FLUX_LINKAGE      0.015f
#define SIMUL_INERTIA           0.00024f
#define SIMUL_FRICTION          0.001f

#define SIMU_V_FULL 24.0f
#define SIMU_I_FULL 20.0f
/* Torque scale [Nm] for normalizing Te, Tf into Q1.31 */
#define SIMU_T_NOM_NM           10.0f

typedef struct {
    iq31_t r_q31;
    iq31_t k_diq_q31;
    iq31_t k_flux_w_q31;
    iq31_t k_Te_q31;
    iq31_t k_Tf_q31;
    iq31_t k_dw_q31;
    iq31_t k_angle_q31;
    iq31_t angle;
    iq31_t omega;
    iq31_t estim_iq;
    iq31_t dt_q31;
    esp_foc_observer_t interface;
} esp_foc_simul_observer_t;

static const char *TAG = "ESP-FOC-SIMU-OBS";
static esp_foc_simul_observer_t simu_observers[CONFIG_NOOF_AXIS];

static int simu_observer_update(esp_foc_observer_t *self, esp_foc_observer_inputs_t *in)
{
    esp_foc_simul_observer_t *obj = __containerof(self, esp_foc_simul_observer_t, interface);

    iq31_t uq = in->u_dq[1];
    iq31_t ri = iq31_mul(obj->r_q31, obj->estim_iq);
    iq31_t fw = iq31_mul(obj->k_flux_w_q31, obj->omega);
    iq31_t num = iq31_sub(uq, iq31_add(ri, fw));
    iq31_t d_iq = iq31_mul(obj->k_diq_q31, num);
    obj->estim_iq = iq31_add(obj->estim_iq, d_iq);

    iq31_t Te = iq31_mul(obj->k_Te_q31, obj->estim_iq);
    iq31_t Tf = iq31_mul(obj->k_Tf_q31, obj->omega);
    iq31_t dw = iq31_mul(obj->k_dw_q31, iq31_sub(Te, Tf));
    obj->omega = iq31_add(obj->omega, dw);

    obj->angle = iq31_normalize_angle(iq31_add(obj->angle, iq31_mul(obj->omega, obj->k_angle_q31)));

    (void)obj->dt_q31;
    return 0;
}

static iq31_t simu_observer_get_angle(esp_foc_observer_t *self)
{
    esp_foc_simul_observer_t *obj = __containerof(self, esp_foc_simul_observer_t, interface);
    return obj->angle;
}

static iq31_t simu_observer_get_speed(esp_foc_observer_t *self)
{
    esp_foc_simul_observer_t *obj = __containerof(self, esp_foc_simul_observer_t, interface);
    return obj->omega;
}

static void simu_observer_reset(esp_foc_observer_t *self, iq31_t offset)
{
    esp_foc_simul_observer_t *obj = __containerof(self, esp_foc_simul_observer_t, interface);
    obj->angle = iq31_normalize_angle(offset);
    obj->estim_iq = 0;
    obj->omega = 0;
}

esp_foc_observer_t *simu_observer_new(int unit, esp_foc_simu_observer_settings_t settings)
{
    if (unit >= CONFIG_NOOF_AXIS) {
        ESP_LOGE(TAG, "Invalid unit!");
        return NULL;
    }

    if (settings.dt == 0.0f) {
        ESP_LOGE(TAG, "Invalid dt!");
        return NULL;
    }

    if (settings.phase_resistance <= 0.0f) {
        ESP_LOGE(TAG, "Invalid phase resistance !");
        return NULL;
    }

    if (settings.phase_inductance <= 0.0f) {
        ESP_LOGE(TAG, "Invalid phase Inductance !");
        return NULL;
    }

    const float Vb = SIMU_V_FULL;
    const float Ib = SIMU_I_FULL;
    const float dt = settings.dt;
    const float R = settings.phase_resistance;
    const float L = settings.phase_inductance;
    const float pp = settings.pole_pairs;

    const float Te_coeff_Nm_per_A = 1.5f * pp * SIMUL_FLUX_LINKAGE * Ib;

    esp_foc_simul_observer_t *obj = &simu_observers[unit];
    obj->interface.update = simu_observer_update;
    obj->interface.get_angle = simu_observer_get_angle;
    obj->interface.get_speed = simu_observer_get_speed;
    obj->interface.reset = simu_observer_reset;

    obj->r_q31 = iq31_from_float((R * Ib) / Vb);
    obj->k_diq_q31 = iq31_from_float((dt / L) * (Vb / Ib));
    obj->k_flux_w_q31 = iq31_from_float((SIMUL_FLUX_LINKAGE * ESP_FOC_OBS_OMEGA_MAX_RAD_S) / Vb);

    obj->k_Te_q31 = iq31_from_float(Te_coeff_Nm_per_A / SIMU_T_NOM_NM);
    obj->k_Tf_q31 = iq31_from_float((SIMUL_FRICTION * ESP_FOC_OBS_OMEGA_MAX_RAD_S) / SIMU_T_NOM_NM);
    obj->k_dw_q31 = iq31_from_float((SIMU_T_NOM_NM * dt) / (SIMUL_INERTIA * ESP_FOC_OBS_OMEGA_MAX_RAD_S));

    obj->k_angle_q31 = iq31_from_float((ESP_FOC_OBS_OMEGA_MAX_RAD_S * dt) / (2.0f * (float)M_PI));
    obj->dt_q31 = iq31_from_float(dt);

    obj->omega = 0;
    obj->angle = 0;
    obj->estim_iq = 0;

    ESP_LOGI(TAG, "Simu observer Hz=%f pp=%f (IQ31)", (double)(1.0f / dt), (double)pp);

    return &obj->interface;
}
