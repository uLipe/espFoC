/*
 * PMSM electromechanical model — Euler integration, Q16 in update().
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
#include "espFoC/observer/esp_foc_pmsm_model_observer.h"

#define PMSM_V_FULL 24.0f
#define PMSM_I_FULL 20.0f
#define PMSM_T_NOM_NM 50.0f

typedef struct {
    q16_t R_q16;
    q16_t k_Id_step_q16;
    q16_t k_Iq_step_q16;
    q16_t K_wLq_q16;
    q16_t K_wLd_q16;
    q16_t K_wlambda_q16;
    q16_t k_Te_lambda_q16;
    q16_t k_Te_salient_q16;
    q16_t k_dw_te_q16;
    q16_t Tl_q16;
    q16_t k_B_omega_q16;
    q16_t k_dtheta_rad_q16;
    q16_t omega_e;
    q16_t theta_e;
    q16_t id;
    q16_t iq;
    q16_t i_lim_q16;
    q16_t w_lim_q16;
    esp_foc_observer_t interface;
} esp_foc_pmsm_model_observer_t;

static const char *TAG = "ESP-FOC-PMSM-MODEL";
static DRAM_ATTR esp_foc_pmsm_model_observer_t pmsm_observers[CONFIG_NOOF_AXIS];

static inline void pmsm_clamp(esp_foc_pmsm_model_observer_t *m)
{
    if (m->i_lim_q16 > 0) {
        m->id = q16_clamp(m->id, q16_sub(0, m->i_lim_q16), m->i_lim_q16);
        m->iq = q16_clamp(m->iq, q16_sub(0, m->i_lim_q16), m->i_lim_q16);
    }
    if (m->w_lim_q16 > 0) {
        m->omega_e = q16_clamp(m->omega_e, q16_sub(0, m->w_lim_q16), m->w_lim_q16);
    }
}

static int pmsm_model_observer_update(esp_foc_observer_t *self, esp_foc_observer_inputs_t *in)
{
    esp_foc_pmsm_model_observer_t *m = __containerof(self, esp_foc_pmsm_model_observer_t, interface);

    q16_t sinT = q16_sin(m->theta_e);
    q16_t cosT = q16_cos(m->theta_e);

    q16_t vd;
    q16_t vq;
    q16_park(sinT, cosT, in->u_alpha_beta[0], in->u_alpha_beta[1], &vd, &vq);

    q16_t we = m->omega_e;

    q16_t cross_d = q16_mul(we, q16_mul(m->K_wLq_q16, m->iq));
    q16_t vd_term = q16_add(q16_sub(vd, q16_mul(m->R_q16, m->id)), cross_d);
    q16_t delta_id = q16_mul(m->k_Id_step_q16, vd_term);

    q16_t cross_q = q16_mul(we, q16_mul(m->K_wLd_q16, m->id));
    q16_t flux_q = q16_mul(we, m->K_wlambda_q16);
    q16_t vq_term = q16_sub(q16_sub(q16_sub(vq, q16_mul(m->R_q16, m->iq)), cross_q), flux_q);
    q16_t delta_iq = q16_mul(m->k_Iq_step_q16, vq_term);

    m->id = q16_add(m->id, delta_id);
    m->iq = q16_add(m->iq, delta_iq);

    q16_t Te = q16_add(
        q16_mul(m->k_Te_lambda_q16, m->iq),
        q16_mul(m->k_Te_salient_q16, q16_mul(m->id, m->iq)));

    q16_t dw = q16_sub(
        q16_sub(q16_mul(m->k_dw_te_q16, Te), q16_mul(m->k_dw_te_q16, m->Tl_q16)),
        q16_mul(m->k_B_omega_q16, m->omega_e));

    m->omega_e = q16_add(m->omega_e, dw);
    m->theta_e = q16_normalize_angle_rad(
        q16_add(m->theta_e, q16_mul(m->omega_e, m->k_dtheta_rad_q16)));

    pmsm_clamp(m);
    return 0;
}

static q16_t pmsm_model_observer_get_angle(esp_foc_observer_t *self)
{
    esp_foc_pmsm_model_observer_t *m = __containerof(self, esp_foc_pmsm_model_observer_t, interface);
    return m->theta_e;
}

static q16_t pmsm_model_observer_get_speed(esp_foc_observer_t *self)
{
    esp_foc_pmsm_model_observer_t *m = __containerof(self, esp_foc_pmsm_model_observer_t, interface);
    return m->omega_e;
}

static void pmsm_model_observer_reset(esp_foc_observer_t *self, q16_t offset)
{
    esp_foc_pmsm_model_observer_t *m = __containerof(self, esp_foc_pmsm_model_observer_t, interface);
    m->theta_e = q16_normalize_angle_rad(offset);
    m->omega_e = 0;
    m->id = 0;
    m->iq = 0;
}

esp_foc_observer_t *pmsm_model_observer_new(int unit, esp_foc_pmsm_model_observer_settings_t s)
{
    if (unit >= CONFIG_NOOF_AXIS) {
        ESP_LOGE(TAG, "Invalid unit!");
        return NULL;
    }

    if (s.dt <= 0.0f || s.phase_resistance <= 0.0f || s.Ld <= 0.0f || s.Lq <= 0.0f) {
        ESP_LOGE(TAG, "Invalid electrical parameters!");
        return NULL;
    }

    if (s.inertia <= 0.0f || s.pole_pairs <= 0.0f || s.lambda <= 0.0f) {
        ESP_LOGE(TAG, "Invalid mechanical parameters!");
        return NULL;
    }

    if (s.friction < 0.0f) {
        ESP_LOGE(TAG, "Invalid friction!");
        return NULL;
    }

    const float Vb = PMSM_V_FULL;
    const float Ib = PMSM_I_FULL;
    const float Tn = PMSM_T_NOM_NM;
    const float pp = s.pole_pairs;

    esp_foc_pmsm_model_observer_t *m = &pmsm_observers[unit];

    m->R_q16 = q16_from_float((s.phase_resistance * Ib) / Vb);
    m->k_Id_step_q16 = q16_from_float((s.dt * Vb) / (s.Ld * Ib));
    m->k_Iq_step_q16 = q16_from_float((s.dt * Vb) / (s.Lq * Ib));

    m->K_wLq_q16 = q16_from_float((ESP_FOC_OBS_OMEGA_MAX_RAD_S * s.Lq * Ib) / Vb);
    m->K_wLd_q16 = q16_from_float((ESP_FOC_OBS_OMEGA_MAX_RAD_S * s.Ld * Ib) / Vb);
    m->K_wlambda_q16 = q16_from_float((ESP_FOC_OBS_OMEGA_MAX_RAD_S * s.lambda) / Vb);

    m->k_Te_lambda_q16 = q16_from_float((1.5f * pp * s.lambda * Ib) / Tn);
    m->k_Te_salient_q16 = q16_from_float((1.5f * pp * (s.Ld - s.Lq) * Ib * Ib) / Tn);

    m->k_dw_te_q16 = q16_from_float((s.dt * pp * Tn) / (s.inertia * ESP_FOC_OBS_OMEGA_MAX_RAD_S));
    m->Tl_q16 = q16_from_float(s.load_torque / Tn);
    m->k_B_omega_q16 = q16_from_float((s.friction * s.dt) / s.inertia);

    m->k_dtheta_rad_q16 = q16_from_float(ESP_FOC_OBS_OMEGA_MAX_RAD_S * s.dt);

    m->theta_e = 0;
    m->omega_e = 0;
    m->id = 0;
    m->iq = 0;

    m->i_lim_q16 = (s.current_limit > 0.0f) ? q16_from_float(s.current_limit / Ib) : 0;
    if (s.omega_limit > 0.0f) {
        float omega_e_lim = pp * s.omega_limit;
        m->w_lim_q16 = q16_from_float(omega_e_lim / ESP_FOC_OBS_OMEGA_MAX_RAD_S);
    } else {
        m->w_lim_q16 = 0;
    }

    m->interface.update = pmsm_model_observer_update;
    m->interface.get_angle = pmsm_model_observer_get_angle;
    m->interface.get_speed = pmsm_model_observer_get_speed;
    m->interface.reset = pmsm_model_observer_reset;

    ESP_LOGI(TAG, "PMSM model Euler Q16 dt=%f pp=%f", (double)s.dt, (double)pp);

    return &m->interface;
}
