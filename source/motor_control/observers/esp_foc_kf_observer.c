/*
 * Angle observer (EMF + pseudo-measurement + fixed-gain correction) — IQ31 only in update().
 * Covariance propagation from the legacy float KF is omitted; gains are set at init from tuning.
 */

#include <math.h>
#include <esp_attr.h>
#include <sdkconfig.h>
#include <sys/cdefs.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "espFoC/utils/biquad_q16.h"
#include "espFoC/utils/esp_foc_iq31_q16_bridge.h"
#include "espFoC/observer/esp_foc_observer_interface.h"
#include "espFoC/observer/esp_foc_kf_observer.h"
#include "espFoC/observer/esp_foc_observer_iq31.h"

#define KF_DEFAULT_V_FULL 24.0f
#define KF_DEFAULT_I_FULL 20.0f

typedef struct {
    iq31_t theta_est;
    iq31_t omega_est;
    iq31_t theta_err_emf_q31;
    esp_foc_biquad_q16_t current_filters[2];
    iq31_t e_alpha;
    iq31_t e_beta;
    iq31_t i_alpha_prev;
    iq31_t i_beta_prev;
    iq31_t r_q31;
    iq31_t l_q31;
    iq31_t dt_q31;
    iq31_t mag_eps_q31;
    iq31_t k_theta_q31;
    iq31_t K0_q31;
    iq31_t K1_q31;
    iq31_t eps_q31;
    int converging_count;
    esp_foc_observer_t interface;
} angle_estimator_kf_t;

static const char *TAG = "ESP-FOC-KF-OBS";
static DRAM_ATTR angle_estimator_kf_t kf_observers[CONFIG_NOOF_AXIS];

static inline void kf_predict_iq31(angle_estimator_kf_t *k)
{
    k->theta_est = iq31_normalize_angle(iq31_add(k->theta_est, iq31_mul(k->omega_est, k->k_theta_q31)));
}

static int kf_observer_update(esp_foc_observer_t *self, esp_foc_observer_inputs_t *in)
{
    angle_estimator_kf_t *est = __containerof(self, angle_estimator_kf_t, interface);

    iq31_t ia_f = esp_foc_q16_per_unit_to_iq31(
        esp_foc_biquad_q16_update(&est->current_filters[0],
                                  esp_foc_iq31_per_unit_to_q16(in->i_alpha_beta[0])));
    iq31_t ib_f = esp_foc_q16_per_unit_to_iq31(
        esp_foc_biquad_q16_update(&est->current_filters[1],
                                  esp_foc_iq31_per_unit_to_q16(in->i_alpha_beta[1])));

    iq31_t delta_ia = iq31_sub(ia_f, est->i_alpha_prev);
    iq31_t delta_ib = iq31_sub(ib_f, est->i_beta_prev);
    iq31_t ldi_a = iq31_mul(est->l_q31, delta_ia);
    iq31_t ldi_b = iq31_mul(est->l_q31, delta_ib);

    est->e_alpha = iq31_sub(in->u_alpha_beta[0], iq31_add(iq31_mul(est->r_q31, ia_f), ldi_a));
    est->e_beta = iq31_sub(in->u_alpha_beta[1], iq31_add(iq31_mul(est->r_q31, ib_f), ldi_b));

    est->i_alpha_prev = ia_f;
    est->i_beta_prev = ib_f;

    iq31_t sinT = iq31_sin(est->theta_est);
    iq31_t cosT = iq31_cos(est->theta_est);

    iq31_t I = iq31_add(iq31_mul(est->e_alpha, cosT), iq31_mul(est->e_beta, sinT));
    iq31_t Q = iq31_add(iq31_mul(est->e_alpha, iq31_sub(0, sinT)), iq31_mul(est->e_beta, cosT));

    iq31_t mag2 = iq31_add(iq31_mul(est->e_alpha, est->e_alpha), iq31_mul(est->e_beta, est->e_beta));
    iq31_t rs = iq31_rsqrt_fast(iq31_add(mag2, est->mag_eps_q31));
    iq31_t In = iq31_mul_q230(I, rs);
    iq31_t Qn = iq31_mul_q230(Q, rs);

    iq31_t denom = (In >= 0) ? iq31_add(In, est->eps_q31) : iq31_sub(In, est->eps_q31);
    est->theta_err_emf_q31 = esp_foc_obs_iq31_div(Qn, denom);

    kf_predict_iq31(est);

    {
        iq31_t theta_meas = iq31_normalize_angle(iq31_add(est->theta_est, est->theta_err_emf_q31));
        iq31_t y = esp_foc_obs_wrap_angle_err_pm_pi(iq31_sub(theta_meas, est->theta_est));
        est->theta_est = iq31_normalize_angle(iq31_add(est->theta_est, iq31_mul(est->K0_q31, y)));
        est->omega_est = iq31_add(est->omega_est, iq31_mul(est->K1_q31, y));
    }

    if (est->converging_count) {
        est->converging_count--;
    }

    return est->converging_count;
}

static iq31_t kf_observer_get_angle(esp_foc_observer_t *self)
{
    angle_estimator_kf_t *est = __containerof(self, angle_estimator_kf_t, interface);
    return est->theta_est;
}

static iq31_t kf_observer_get_speed(esp_foc_observer_t *self)
{
    angle_estimator_kf_t *est = __containerof(self, angle_estimator_kf_t, interface);
    return est->omega_est;
}

static void kf_observer_reset(esp_foc_observer_t *self, iq31_t offset)
{
    angle_estimator_kf_t *est = __containerof(self, angle_estimator_kf_t, interface);
    est->theta_est = iq31_normalize_angle(offset);
    est->omega_est = 0;
    est->theta_err_emf_q31 = 0;
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
    est->theta_err_emf_q31 = 0;
    est->i_alpha_prev = 0;
    est->i_beta_prev = 0;
    est->e_alpha = 0;
    est->e_beta = 0;

    est->dt_q31 = iq31_from_float(s.dt);
    est->r_q31 = iq31_from_float((s.phase_resistance * Ib) / Vb);
    est->l_q31 = iq31_from_float((s.phase_inductance * Ib) / (Vb * s.dt));

    est->k_theta_q31 = iq31_from_float((ESP_FOC_OBS_OMEGA_MAX_RAD_S * s.dt) / (2.0f * (float)M_PI));

    const float frac = (s.current_lpf_frac_fs > 0.0f) ? s.current_lpf_frac_fs : 0.2f;
    {
        float fs_hz = 1.0f / s.dt;
        esp_foc_biquad_butterworth_lpf_design_q16(&est->current_filters[0],
                                                  frac * fs_hz, fs_hz);
        esp_foc_biquad_butterworth_lpf_design_q16(&est->current_filters[1],
                                                  frac * fs_hz, fs_hz);
    }

    est->eps_q31 = iq31_from_float(0.05f);
    est->mag_eps_q31 = iq31_from_float(1e-3f);

    /* Fixed gains from measurement noise (init-only float). */
    const float Rm = s.sigma_theta_meas * s.sigma_theta_meas;
    est->K0_q31 = iq31_from_float(0.5f / (0.5f + Rm));
    est->K1_q31 = iq31_from_float((s.sigma_accel * s.dt) / ESP_FOC_OBS_OMEGA_MAX_RAD_S);

    est->converging_count = (int)(20.0f / s.dt);

    est->interface.update = kf_observer_update;
    est->interface.get_angle = kf_observer_get_angle;
    est->interface.get_speed = kf_observer_get_speed;
    est->interface.reset = kf_observer_reset;

    ESP_LOGI(TAG, "KF-style observer dt=%f Hz=%f (IQ31 fixed-gain)", (double)s.dt, (double)(1.0f / s.dt));

    return &est->interface;
}
