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
#include "espFoC/observer/esp_foc_kf_observer.h"

static const char *TAG = "ESP-FOC-KF-OBS";

typedef struct {
    // KF state
    float theta_est;   // electrical angle [rad] 0..2pi
    float omega_est;   // electrical speed [rad/s]

    // Covariance P (2x2, symmetric)
    float P00, P01;
    float P10, P11;

    // Tuning
    float R_meas;      // measurement variance [rad^2]
    float sigma_accel; // [rad/s^2]

    // Motor params
    float r;
    float l;

    // Timing
    float dt;
    float inv_dt;

    // I/O + helper
    esp_foc_lp_filter_t current_filters[2];
    float e_alpha;
    float e_beta;
    float i_alpha_prev;
    float i_beta_prev;

    // Debug
    float theta_error_rad; // estimated delta-theta [rad] used by KF

    // Optional clamp
    float omega_limit;

    // “converging” gate (optional)
    int converging_count;

    esp_foc_observer_t interface;
} angle_estimator_kf_t;

static DRAM_ATTR angle_estimator_kf_t kf_observers[CONFIG_NOOF_AXIS];

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// Wrap error to [-pi, pi]
static inline float wrap_pm_pi(float a)
{
    while (a >  (float)M_PI) a -= 2.0f*(float)M_PI;
    while (a < -(float)M_PI) a += 2.0f*(float)M_PI;
    return a;
}

static inline void kf_predict(angle_estimator_kf_t *k)
{
    const float dt = k->dt;

    // State prediction: [theta; omega]
    // theta_k+1 = theta + dt*omega
    // omega_k+1 = omega
    k->theta_est += k->omega_est * dt;
    k->theta_est = esp_foc_normalize_angle(k->theta_est);

    // Covariance prediction:
    // A = [[1, dt],[0, 1]]
    // Q from accel noise (continuous white accel -> discretized)
    const float sa2 = k->sigma_accel * k->sigma_accel;
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt2 * dt2;

    // Standard constant-acceleration discrete Q:
    // Q = sa^2 * [[dt^4/4, dt^3/2],[dt^3/2, dt^2]]
    const float Q00 = sa2 * (0.25f * dt4);
    const float Q01 = sa2 * (0.5f  * dt3);
    const float Q10 = Q01;
    const float Q11 = sa2 * (dt2);

    // P = A P A^T + Q
    const float P00 = k->P00;
    const float P01 = k->P01;
    const float P10 = k->P10;
    const float P11 = k->P11;

    // A*P
    const float AP00 = P00 + dt * P10;
    const float AP01 = P01 + dt * P11;
    const float AP10 = P10;
    const float AP11 = P11;

    // (A*P)*A^T
    k->P00 = AP00 + dt * AP01 + Q00;
    k->P01 = AP01 + Q01;
    k->P10 = AP10 + dt * AP11 + Q10;
    k->P11 = AP11 + Q11;

    // enforce symmetry lightly
    const float sym = 0.5f * (k->P01 + k->P10);
    k->P01 = sym;
    k->P10 = sym;
}

static inline void kf_update_theta(angle_estimator_kf_t *k, float theta_meas)
{
    // Measurement model: z = theta + v
    // H = [1, 0]
    // innovation y = wrap(theta_meas - theta_pred)
    float y = wrap_pm_pi(theta_meas - k->theta_est);

    // S = HPH^T + R = P00 + R
    const float S = k->P00 + k->R_meas;
    if (S <= 1e-12f) return;

    // K = P H^T / S = [P00; P10]/S
    const float K0 = k->P00 / S;
    const float K1 = k->P10 / S;

    // State update
    k->theta_est = esp_foc_normalize_angle(k->theta_est + K0 * y);
    k->omega_est = k->omega_est + K1 * y;

    // Covariance update: P = (I - K H) P
    // (I - K H) = [[1-K0, 0],[-K1, 1]]
    const float P00 = k->P00;
    const float P01 = k->P01;
    const float P10 = k->P10;
    const float P11 = k->P11;

    k->P00 = (1.0f - K0) * P00;
    k->P01 = (1.0f - K0) * P01;
    k->P10 = P10 - K1 * P00;
    k->P11 = P11 - K1 * P01;

    // enforce symmetry
    const float sym = 0.5f * (k->P01 + k->P10);
    k->P01 = sym;
    k->P10 = sym;

    // Optional clamp omega
    if (k->omega_limit > 0.0f) {
        k->omega_est = clampf(k->omega_est, -k->omega_limit, k->omega_limit);
    }
}

static int kf_observer_update(esp_foc_observer_t *self, esp_foc_observer_inputs_t *in)
{
    angle_estimator_kf_t *est = __containerof(self, angle_estimator_kf_t, interface);

    // Filter currents at the root (same pattern you already do)
    in->i_alpha_beta[0] = esp_foc_low_pass_filter_update(&est->current_filters[0], in->i_alpha_beta[0]);
    in->i_alpha_beta[1] = esp_foc_low_pass_filter_update(&est->current_filters[1], in->i_alpha_beta[1]);

    // di/dt
    const float di_alpha_dt = (in->i_alpha_beta[0] - est->i_alpha_prev) * est->inv_dt;
    const float di_beta_dt  = (in->i_alpha_beta[1] - est->i_beta_prev)  * est->inv_dt;

    est->i_alpha_prev = in->i_alpha_beta[0];
    est->i_beta_prev  = in->i_alpha_beta[1];

    // Back-EMF estimate
    est->e_alpha = in->u_alpha_beta[0] - est->r * in->i_alpha_beta[0] - est->l * di_alpha_dt;
    est->e_beta  = in->u_alpha_beta[1] - est->r * in->i_alpha_beta[1] - est->l * di_beta_dt;

    // Phase detector: compute I/Q against NCO (local oscillator)
    // Q = cross(e, nco) ~ |e| sin(delta)
    // I = dot  (e, nco) ~ |e| cos(delta)
    const float sinT = esp_foc_sine(est->theta_est);
    const float cosT = esp_foc_cosine(est->theta_est);

    const float I = (est->e_alpha * cosT) + (est->e_beta * sinT);
    const float Q = (est->e_alpha * -sinT) + (est->e_beta *  cosT);

    // Normalize to remove speed-dependent gain (optional but helps)
    const float mag_inv = esp_foc_rsqrt_fast(est->e_alpha * est->e_alpha + est->e_beta * est->e_beta + 1e-3f);
    const float In = I * mag_inv;  // ~ cos(delta)
    const float Qn = Q * mag_inv;  // ~ sin(delta)

    // Estimate phase error in radians (cheap, no atan2):
    // delta ≈ tan(delta) = sin/cos = Qn/(In+eps) when near lock
    const float eps = 0.05f; // keep bounded when In ~ 0; tweak if needed
    est->theta_error_rad = Qn / (In + (In >= 0.0f ? eps : -eps));

    // --- Kalman step ---
    kf_predict(est);

    // Build a pseudo measurement of theta:
    // theta_meas = theta_pred + delta
    // (innovation becomes ~delta in wrap sense)
    const float theta_meas = esp_foc_normalize_angle(est->theta_est + est->theta_error_rad);
    kf_update_theta(est, theta_meas);

    // “converging” counter (placeholder like your PLL code)
    if (est->converging_count) {
        est->converging_count--;
    }

    return est->converging_count;
}

static float kf_observer_get_angle(esp_foc_observer_t *self)
{
    angle_estimator_kf_t *est = __containerof(self, angle_estimator_kf_t, interface);
    return est->theta_est; // electrical
}

static float kf_observer_get_speed(esp_foc_observer_t *self)
{
    angle_estimator_kf_t *est = __containerof(self, angle_estimator_kf_t, interface);
    return est->omega_est; // electrical rad/s
}

static void kf_observer_reset(esp_foc_observer_t *self, float offset)
{
    angle_estimator_kf_t *est = __containerof(self, angle_estimator_kf_t, interface);

    est->theta_est = esp_foc_normalize_angle(offset);
    est->omega_est = 0.0f;
    est->theta_error_rad = 0.0f;

    // Reset P (start “uncertain” but sane)
    est->P00 = 0.5f;  est->P01 = 0.0f;
    est->P10 = 0.0f;  est->P11 = 50.0f;

    est->i_alpha_prev = 0.0f;
    est->i_beta_prev  = 0.0f;
    est->e_alpha = 0.0f;
    est->e_beta  = 0.0f;

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

    angle_estimator_kf_t *est = &kf_observers[unit];

    est->theta_est = 0.0f;
    est->omega_est = 0.0f;

    // P init
    est->P00 = 0.5f;  est->P01 = 0.0f;
    est->P10 = 0.0f;  est->P11 = 50.0f;

    // noise
    est->R_meas = s.sigma_theta_meas * s.sigma_theta_meas;
    est->sigma_accel = s.sigma_accel;

    est->r = s.phase_resistance;
    est->l = s.phase_inductance;

    est->dt = s.dt;
    est->inv_dt = 1.0f / s.dt;

    est->omega_limit = s.omega_limit;

    est->i_alpha_prev = 0.0f;
    est->i_beta_prev  = 0.0f;

    const float frac = (s.current_lpf_frac_fs > 0.0f) ? s.current_lpf_frac_fs : 0.2f;
    esp_foc_low_pass_filter_set_cutoff(&est->current_filters[0], frac * est->inv_dt, est->inv_dt);
    esp_foc_low_pass_filter_set_cutoff(&est->current_filters[1], frac * est->inv_dt, est->inv_dt);

    est->converging_count = (int)(20.0f / s.dt);

    est->interface.update = kf_observer_update;
    est->interface.get_angle = kf_observer_get_angle;
    est->interface.get_speed = kf_observer_get_speed;
    est->interface.reset = kf_observer_reset;

    ESP_LOGI(TAG,
             "KF observer dt=%f Hz=%f R=%f L=%f sigma_meas=%f sigma_accel=%f",
             est->dt, (1.0f/est->dt), est->r, est->l, s.sigma_theta_meas, s.sigma_accel);

    return &est->interface;
}