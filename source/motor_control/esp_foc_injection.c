/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include <string.h>
#include "espFoC/esp_foc_injection.h"

static inline q16_t q16_mul_inl(q16_t a, q16_t b)
{
    int64_t p = (int64_t)a * (int64_t)b;
    p += (1LL << 15);
    return (q16_t)(p >> 16);
}

static inline q16_t wrap_angle(q16_t a)
{
    /* Keep phase_acc inside [0, 2*pi) to avoid Q16 overflow on long sweeps. */
    while (a >= Q16_TWO_PI) {
        a = (q16_t)((int32_t)a - (int32_t)Q16_TWO_PI);
    }
    while (a < 0) {
        a = (q16_t)((int32_t)a + (int32_t)Q16_TWO_PI);
    }
    return a;
}

void esp_foc_injection_disable(esp_foc_injection_t *inj)
{
    if (inj == NULL) {
        return;
    }
    memset(inj, 0, sizeof(*inj));
    inj->mode = ESP_FOC_INJECTION_MODE_NONE;
    inj->enabled = false;
}

static uint32_t samples_from_ms(uint32_t duration_ms, q16_t ts_seconds)
{
    if (ts_seconds <= 0 || duration_ms == 0) {
        return 0;
    }
    /* samples = duration_ms * 1e-3 / ts_seconds
     *         = (duration_ms * Q16_ONE) / (ts_seconds * 1000)
     * using int64 to avoid overflow for long durations / tight loops. */
    int64_t num = (int64_t)duration_ms * (int64_t)Q16_ONE;
    int64_t den = (int64_t)ts_seconds * 1000LL;
    if (den <= 0) {
        return 0;
    }
    int64_t n = num / den;
    if (n < 0) {
        return 0;
    }
    if (n > 0x7FFFFFFFLL) {
        n = 0x7FFFFFFFLL;
    }
    return (uint32_t)n;
}

void esp_foc_injection_step_setup(esp_foc_injection_t *inj,
                                  q16_t amplitude,
                                  uint32_t duration_ms,
                                  q16_t ts_seconds)
{
    if (inj == NULL) {
        return;
    }
    esp_foc_injection_disable(inj);
    inj->mode = ESP_FOC_INJECTION_MODE_STEP;
    inj->amplitude = amplitude;
    inj->ts_seconds = ts_seconds;
    inj->samples_total = samples_from_ms(duration_ms, ts_seconds);
    inj->enabled = true;
}

void esp_foc_injection_chirp_setup(esp_foc_injection_t *inj,
                                   q16_t amplitude,
                                   q16_t freq_start_hz,
                                   q16_t freq_end_hz,
                                   uint32_t duration_ms,
                                   q16_t ts_seconds)
{
    if (inj == NULL) {
        return;
    }
    esp_foc_injection_disable(inj);
    inj->mode = ESP_FOC_INJECTION_MODE_CHIRP;
    inj->amplitude = amplitude;
    inj->ts_seconds = ts_seconds;
    inj->freq_current_hz = freq_start_hz;
    inj->freq_end_hz = freq_end_hz;
    inj->samples_total = samples_from_ms(duration_ms, ts_seconds);

    /* Linear sweep: slope = (f_end - f_start) / total_time (Hz/s) */
    if (inj->samples_total > 0 && ts_seconds > 0) {
        int64_t df = (int64_t)freq_end_hz - (int64_t)freq_start_hz;
        int64_t total_us = (int64_t)inj->samples_total * (int64_t)ts_seconds; /* Q16 s * samples */
        /* slope_q16_per_s = df / total_seconds. total_seconds = samples * ts_seconds (Q16).
         * slope = (df << 16) / total; total here is already Q16 units for s,
         * giving slope_q16 / Q16 normalization to cancel. */
        int64_t total_scaled = (int64_t)inj->samples_total * (int64_t)ts_seconds;
        if (total_scaled != 0) {
            int64_t slope = (df << 16) / total_scaled;
            inj->freq_slope_hz_per_s = (q16_t)slope;
        } else {
            inj->freq_slope_hz_per_s = 0;
        }
        (void)total_us;
    } else {
        inj->freq_slope_hz_per_s = 0;
    }
    inj->enabled = true;
}

q16_t esp_foc_injection_apply_q16(esp_foc_injection_t *inj, q16_t base_reference)
{
    if (inj == NULL || !inj->enabled) {
        return base_reference;
    }

    q16_t contribution = 0;

    switch (inj->mode) {
    case ESP_FOC_INJECTION_MODE_STEP:
        contribution = inj->amplitude;
        break;

    case ESP_FOC_INJECTION_MODE_CHIRP: {
        /* Sinusoid sample: amplitude * sin(phase). */
        contribution = q16_mul_inl(inj->amplitude, q16_sin(inj->phase_acc_rad));
        /* Advance phase: delta = 2*pi*f*Ts */
        q16_t omega = q16_mul_inl(Q16_TWO_PI, inj->freq_current_hz);
        q16_t delta = q16_mul_inl(omega, inj->ts_seconds);
        inj->phase_acc_rad = wrap_angle((q16_t)((int32_t)inj->phase_acc_rad + (int32_t)delta));
        /* Advance frequency: f += slope * Ts */
        q16_t df = q16_mul_inl(inj->freq_slope_hz_per_s, inj->ts_seconds);
        inj->freq_current_hz = (q16_t)((int32_t)inj->freq_current_hz + (int32_t)df);
        break;
    }

    default:
        break;
    }

    inj->samples_elapsed++;
    if (inj->samples_total > 0 && inj->samples_elapsed >= inj->samples_total) {
        inj->enabled = false;
    }

    return (q16_t)((int32_t)base_reference + (int32_t)contribution);
}
