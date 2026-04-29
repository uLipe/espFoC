/*
 * Angle observer (EMF + pseudo-measurement + fixed-gain correction) — Q16 in update().
 */

#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <esp_attr.h>
#include <sdkconfig.h>
#include <sys/cdefs.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "espFoC/utils/biquad_q16.h"
#include "espFoC/utils/esp_foc_int_sqrt.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/foc_math_q16.h"
#include "espFoC/observer/esp_foc_observer_interface.h"
#include "espFoC/observer/esp_foc_observer_math_q16.h"
#include "espFoC/observer/esp_foc_kf_observer.h"

#define KF_DEFAULT_V_FULL 24.0f
#define KF_DEFAULT_I_FULL 20.0f

typedef struct {
    q16_t theta_est;
    q16_t omega_est;
    q16_t theta_err_emf_q16;
    esp_foc_biquad_q16_t current_filters[2];
    q16_t e_alpha;
    q16_t e_beta;
    q16_t i_alpha_prev;
    q16_t i_beta_prev;
    q16_t r_q16;
    q16_t l_q16;
    q16_t k_dtheta_rad_q16;
    q16_t mag_eps_q16;
    q16_t K0_q16;
    q16_t K1_q16;
    q16_t eps_q16;
    int converging_count;
    esp_foc_observer_t interface;
} angle_estimator_kf_t;

static const char *TAG = "ESP-FOC-KF-OBS";
static DRAM_ATTR angle_estimator_kf_t kf_observers[CONFIG_NOOF_AXIS];

static inline void kf_predict_q16(angle_estimator_kf_t *k)
{
    k->theta_est = q16_normalize_angle_rad(
        q16_add(k->theta_est, q16_mul(k->omega_est, k->k_dtheta_rad_q16)));
}

static int kf_observer_update(esp_foc_observer_t *self, esp_foc_observer_inputs_t *in)
{
    angle_estimator_kf_t *est = __containerof(self, angle_estimator_kf_t, interface);

    q16_t ia_f = esp_foc_biquad_q16_update(&est->current_filters[0], in->i_alpha_beta[0]);
    q16_t ib_f = esp_foc_biquad_q16_update(&est->current_filters[1], in->i_alpha_beta[1]);

    q16_t delta_ia = q16_sub(ia_f, est->i_alpha_prev);
    q16_t delta_ib = q16_sub(ib_f, est->i_beta_prev);
    q16_t ldi_a = q16_mul(est->l_q16, delta_ia);
    q16_t ldi_b = q16_mul(est->l_q16, delta_ib);

    est->e_alpha = q16_sub(in->u_alpha_beta[0], q16_add(q16_mul(est->r_q16, ia_f), ldi_a));
    est->e_beta = q16_sub(in->u_alpha_beta[1], q16_add(q16_mul(est->r_q16, ib_f), ldi_b));

    est->i_alpha_prev = ia_f;
    est->i_beta_prev = ib_f;

    q16_t sinT = q16_sin(est->theta_est);
    q16_t cosT = q16_cos(est->theta_est);

    q16_t I = q16_add(q16_mul(est->e_alpha, cosT), q16_mul(est->e_beta, sinT));
    q16_t Q = q16_add(q16_mul(est->e_alpha, q16_sub(0, sinT)), q16_mul(est->e_beta, cosT));

    int64_t ea64 = (int64_t)est->e_alpha;
    int64_t eb64 = (int64_t)est->e_beta;
    uint64_t av = (uint64_t)(ea64 >= 0 ? ea64 : -ea64);
    uint64_t aq = (uint64_t)(eb64 >= 0 ? eb64 : -eb64);
    uint64_t m2 = av * av + aq * aq;
    int64_t me64 = (int64_t)est->mag_eps_q16;
    uint64_t me = (uint64_t)(me64 >= 0 ? me64 : -me64);
    uint64_t m2e = m2 + me;

    uint64_t inv_u64 = esp_foc_u64_rsqrt(m2e);
    if (inv_u64 > (uint64_t)INT32_MAX) {
        inv_u64 = (uint64_t)INT32_MAX;
    }
    q16_t inv = (q16_t)inv_u64;
    q16_t In = q16_mul(I, inv);
    q16_t Qn = q16_mul(Q, inv);

    q16_t denom = (In >= 0) ? q16_add(In, est->eps_q16) : q16_sub(In, est->eps_q16);
    est->theta_err_emf_q16 = esp_foc_obs_q16_div(Qn, denom);

    kf_predict_q16(est);

    {
        q16_t theta_meas = q16_normalize_angle_rad(q16_add(est->theta_est, est->theta_err_emf_q16));
        q16_t y = esp_foc_obs_wrap_angle_err_pm_pi_rad(q16_sub(theta_meas, est->theta_est));
        est->theta_est = q16_normalize_angle_rad(q16_add(est->theta_est, q16_mul(est->K0_q16, y)));
        est->omega_est = q16_add(est->omega_est, q16_mul(est->K1_q16, y));
    }

    if (est->converging_count) {
        est->converging_count--;
    }

    return est->converging_count;
}

static q16_t kf_observer_get_angle(esp_foc_observer_t *self)
{
    angle_estimator_kf_t *est = __containerof(self, angle_estimator_kf_t, interface);
    return est->theta_est;
}

static q16_t kf_observer_get_speed(esp_foc_observer_t *self)
{
    angle_estimator_kf_t *est = __containerof(self, angle_estimator_kf_t, interface);
    return est->omega_est;
}

static void kf_observer_reset(esp_foc_observer_t *self, q16_t offset)
{
    angle_estimator_kf_t *est = __containerof(self, angle_estimator_kf_t, interface);
    est->theta_est = q16_normalize_angle_rad(offset);
    est->omega_est = 0;
    est->theta_err_emf_q16 = 0;
    est->i_alpha_prev = 0;
    est->i_beta_prev = 0;
    est->e_alpha = 0;
    est->e_beta = 0;
    est->converging_count = 0;
}

esp_foc_observer_t *kf_observer_new(int unit, esp_foc_kf_observer_settings_t s)
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

    if (s.phase_inductance <= 0.0f) {
        ESP_LOGE(TAG, "Invalid phase inductance!");
        return NULL;
    }

    if (s.sigma_theta_meas <= 0.0f) {
        ESP_LOGE(TAG, "Invalid sigma_theta_meas!");
        return NULL;
    }

    if (s.sigma_accel <= 0.0f) {
        ESP_LOGE(TAG, "Invalid sigma_accel!");
        return NULL;
    }

    const float Vb = KF_DEFAULT_V_FULL;
    const float Ib = KF_DEFAULT_I_FULL;

    angle_estimator_kf_t *est = &kf_observers[unit];

    est->theta_est = 0;
    est->omega_est = 0;
    est->theta_err_emf_q16 = 0;
    est->i_alpha_prev = 0;
    est->i_beta_prev = 0;
    est->e_alpha = 0;
    est->e_beta = 0;

    est->r_q16 = q16_from_float((s.phase_resistance * Ib) / Vb);
    est->l_q16 = q16_from_float((s.phase_inductance * Ib) / (Vb * s.dt));

    est->k_dtheta_rad_q16 = q16_from_float(ESP_FOC_OBS_OMEGA_MAX_RAD_S * s.dt);

    const float frac = (s.current_lpf_frac_fs > 0.0f) ? s.current_lpf_frac_fs : 0.2f;
    {
        float fs_hz = 1.0f / s.dt;
        esp_foc_biquad_butterworth_lpf_design_q16(&est->current_filters[0],
                                                  frac * fs_hz, fs_hz);
        esp_foc_biquad_butterworth_lpf_design_q16(&est->current_filters[1],
                                                  frac * fs_hz, fs_hz);
    }

    est->eps_q16 = q16_from_float(0.05f);
    est->mag_eps_q16 = q16_from_float(1e-3f);

    const float Rm = s.sigma_theta_meas * s.sigma_theta_meas;
    est->K0_q16 = q16_from_float(0.5f / (0.5f + Rm));
    est->K1_q16 = q16_from_float((s.sigma_accel * s.dt) / ESP_FOC_OBS_OMEGA_MAX_RAD_S);

    est->converging_count = (int)(20.0f / s.dt);

    est->interface.update = kf_observer_update;
    est->interface.get_angle = kf_observer_get_angle;
    est->interface.get_speed = kf_observer_get_speed;
    est->interface.reset = kf_observer_reset;

    ESP_LOGI(TAG, "KF-style observer dt=%f Hz=%f (Q16 fixed-gain)", (double)s.dt, (double)(1.0f / s.dt));

    return &est->interface;
}
