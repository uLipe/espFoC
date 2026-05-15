#pragma once

#include <espFoC/esp_foc.h>

typedef struct {
    // Motor params (per-phase)
    float phase_resistance;   // R [ohm]
    float phase_inductance;   // L [H]

    // Motor geometry
    float pole_pairs;         // for interpreting omega if you want (optional)
                              // (the observer returns electrical omega by default)

    // Execution
    float dt;                 // [s]

    // Kalman tuning (intuitive)
    // Measurement noise: how noisy is your phase measurement (rad RMS)
    float sigma_theta_meas;   // [rad] e.g. 0.02 .. 0.15

    // Process noise: how “agile” omega can change (electrical rad/s^2 RMS)
    float sigma_accel;        // [rad/s^2] e.g. 200 .. 5000 depending on system

    // Optional clamps
    float omega_limit;        // [rad/s] electrical, 0 disables

    // Current LPF cutoff as fraction of Fs (0..0.49). Example: 0.2 means 0.2*Fs
    float current_lpf_frac_fs;
} esp_foc_kf_observer_settings_t;

esp_foc_observer_t *kf_observer_new(int unit, esp_foc_kf_observer_settings_t settings);