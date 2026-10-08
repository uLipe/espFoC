/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_FOC_MECH_ID_VALID_K (1u << 0)
#define ESP_FOC_MECH_ID_VALID_J (1u << 1)

/**
 * Sensored mechanical-plant probe: K = dω̇_e / di_q from a two-level iq step.
 *
 * Leaf, same shape as motor_id: thread-context sequencer, no peripherals, no
 * FreeRTOS. The caller owns Park, the current loop, and the ω_e estimate.
 * J is a sanity print from K, pp, and ψ_f — not a gain.
 */
typedef struct {
    void *ctx;
    void (*set_idq)(void *ctx, q16_t id, q16_t iq);
    /* Optional. Snapshot at entry so a refuse restores the caller's iq*. */
    void (*get_idq)(void *ctx, q16_t *id, q16_t *iq);
    q16_t (*get_omega_e)(void *ctx);
    bool (*faulted)(void *ctx);
    void (*sleep_ms)(void *ctx, uint32_t ms);
} esp_foc_mech_id_ops_t;

typedef struct {
    float i_max_a;
    float iq_lo_frac;
    float iq_hi_frac;
    /*
     * Longest fit window. A window closes early, after at least 8 reads, once
     * it has spent its share of the speed headroom to 0.9*we_max (or decayed
     * to we_entry_min): K spans 50:1 across shafts, so a fixed window that
     * resolves a slow plant runs a stiff one into the cap.
     */
    uint32_t win_ms;
    /*
     * ω_e read period inside a window; each level's acceleration is the
     * least-squares slope over those reads. Two endpoint reads 120 ms apart
     * put σ(ω)·√2/T ≈ 300 rad/s² of noise on a ~250 rad/s² signal, which is
     * how K wandered 846..5900 between boots on one shaft.
     */
    uint32_t sample_ms;
    uint32_t settle_ms;
    float we_entry_min_hz;
    float we_max_hz;
    float k_min;
    float k_max;
    int pole_pairs;
    float psi_f_wb;
} esp_foc_mech_id_config_t;

typedef struct {
    float k_rad_s2_per_a;
    float j_kgm2;
    /* Kept on ESP_ERR_INVALID_RESPONSE (K outside the gate) so it can be reported. */
    float accel[2];
    float iq_a[2];
    uint32_t valid_mask;
} esp_foc_mech_id_result_t;

void esp_foc_mech_id_default_config(esp_foc_mech_id_config_t *cfg);

esp_err_t esp_foc_mech_id_run(const esp_foc_mech_id_ops_t *ops,
                              const esp_foc_mech_id_config_t *cfg,
                              esp_foc_mech_id_result_t *out);

#ifdef __cplusplus
}
#endif
