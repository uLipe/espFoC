/*
 * Simulated plant observer — Q16 in update().
 */

#include <math.h>
#include <esp_attr.h>
#include <sdkconfig.h>
#include <sys/cdefs.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/foc_math_q16.h"
#include "espFoC/observer/esp_foc_observer_interface.h"
#include "espFoC/observer/esp_foc_simu_observer.h"

#define SIMUL_FLUX_LINKAGE      0.015f
#define SIMUL_INERTIA           0.00024f
#define SIMUL_FRICTION          0.001f

#define SIMU_V_FULL 24.0f
#define SIMU_I_FULL 20.0f
#define SIMU_T_NOM_NM           10.0f

typedef struct {
    q16_t r_q16;
    q16_t k_diq_q16;
    q16_t k_flux_w_q16;
    q16_t k_Te_q16;
    q16_t k_Tf_q16;
    q16_t k_dw_q16;
    q16_t k_dtheta_rad_q16;
    q16_t angle;
    q16_t omega;
    q16_t estim_iq;
    esp_foc_observer_t interface;
} esp_foc_simul_observer_t;

static const char *TAG = "ESP-FOC-SIMU-OBS";
static esp_foc_simul_observer_t simu_observers[CONFIG_NOOF_AXIS];

static int simu_observer_update(esp_foc_observer_t *self, esp_foc_observer_inputs_t *in)
{
    esp_foc_simul_observer_t *obj = __containerof(self, esp_foc_simul_observer_t, interface);

    q16_t uq = in->u_dq[1];
    q16_t ri = q16_mul(obj->r_q16, obj->estim_iq);
    q16_t fw = q16_mul(obj->k_flux_w_q16, obj->omega);
    q16_t num = q16_sub(uq, q16_add(ri, fw));
    q16_t d_iq = q16_mul(obj->k_diq_q16, num);
    obj->estim_iq = q16_add(obj->estim_iq, d_iq);

    q16_t Te = q16_mul(obj->k_Te_q16, obj->estim_iq);
    q16_t Tf = q16_mul(obj->k_Tf_q16, obj->omega);
    q16_t dw = q16_mul(obj->k_dw_q16, q16_sub(Te, Tf));
    obj->omega = q16_add(obj->omega, dw);

    obj->angle = q16_normalize_angle_rad(
        q16_add(obj->angle, q16_mul(obj->omega, obj->k_dtheta_rad_q16)));

    return 0;
}

static q16_t simu_observer_get_angle(esp_foc_observer_t *self)
{
    esp_foc_simul_observer_t *obj = __containerof(self, esp_foc_simul_observer_t, interface);
    return obj->angle;
}

static q16_t simu_observer_get_speed(esp_foc_observer_t *self)
{
    esp_foc_simul_observer_t *obj = __containerof(self, esp_foc_simul_observer_t, interface);
    return obj->omega;
}

static void simu_observer_reset(esp_foc_observer_t *self, q16_t offset)
{
    esp_foc_simul_observer_t *obj = __containerof(self, esp_foc_simul_observer_t, interface);
    obj->angle = q16_normalize_angle_rad(offset);
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

    obj->r_q16 = q16_from_float((R * Ib) / Vb);
    obj->k_diq_q16 = q16_from_float((dt / L) * (Vb / Ib));
    obj->k_flux_w_q16 = q16_from_float((SIMUL_FLUX_LINKAGE * ESP_FOC_OBS_OMEGA_MAX_RAD_S) / Vb);

    obj->k_Te_q16 = q16_from_float(Te_coeff_Nm_per_A / SIMU_T_NOM_NM);
    obj->k_Tf_q16 = q16_from_float((SIMUL_FRICTION * ESP_FOC_OBS_OMEGA_MAX_RAD_S) / SIMU_T_NOM_NM);
    obj->k_dw_q16 = q16_from_float((SIMU_T_NOM_NM * dt) / (SIMUL_INERTIA * ESP_FOC_OBS_OMEGA_MAX_RAD_S));

    obj->k_dtheta_rad_q16 = q16_from_float(ESP_FOC_OBS_OMEGA_MAX_RAD_S * dt);

    obj->omega = 0;
    obj->angle = 0;
    obj->estim_iq = 0;

    ESP_LOGI(TAG, "Simu observer Hz=%f pp=%f (Q16)", (double)(1.0f / dt), (double)pp);

    return &obj->interface;
}
