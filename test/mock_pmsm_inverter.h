/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Test-only PMSM behind the inverter vtable: RL stator with BEMF, rigid rotor
 * with viscous and Coulomb friction. A ticker task calls the PWM callback,
 * integrates the plant over one period and posts the DMA callback, so the
 * controller sees the same one-sample delay as on the bridge.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "espFoC/drivers/esp_foc_inverter.h"
#include "espFoC/drivers/esp_foc_rotor_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float vdc;
    float rs;
    float ls;
    float psi;
    int pp;
    float j;
    float b_visc;   /* N·m·s/rad, mechanical */
    float t_coul;   /* N·m */
    uint32_t pwm_hz; /* multiple of 1 kHz: the ticker runs pwm_hz/1000 periods per ms */
    float theta0;
    /* Position-locked torque t_cog·sin(cog_n·θm), N·m; 0 = none. */
    float t_cog;
    int cog_n;
} mock_pmsm_params_t;

typedef struct {
    esp_foc_inverter_t base;
    mock_pmsm_params_t p;
    float a_decay;
    float dt;

    esp_foc_inverter_cb_t pwm_cb;
    void *pwm_arg;
    esp_foc_inverter_cb_t dma_cb;
    void *dma_arg;
    esp_foc_fault_cb_t fault_cb;
    void *fault_arg;

    volatile bool enabled;
    volatile bool faulted;
    volatile bool locked;
    esp_foc_fault_reason_t reason;
    esp_foc_phase_map_t map;
    float duty[3];
    float i_a;
    float i_b;
    float theta_e;
    float theta_m;
    float w_m;
    q16_t i_log[3];
    volatile bool ready;

    volatile int enable_count;
    volatile int disable_count;
    volatile int idle_disables;
    volatile int cal_count;
    volatile int clear_count;
    volatile bool wd_on;
    volatile uint32_t ticks;

    volatile uint32_t cb_n;
    volatile uint64_t cb_us_sum;
    volatile uint32_t cb_us_max;
} mock_pmsm_t;

/*
 * Absolute encoder on the plant's shaft. fetch() latches the plant angle, so
 * the reading ages by however long the caller takes to use it, as on a bus
 * sensor. offset_e is a mounting error added to the reported θe.
 */
typedef struct {
    esp_foc_rotor_sensor_t base;
    mock_pmsm_t *plant;
    float offset_e;
    q16_t theta_e;
    q16_t theta_m;
    q16_t omega_m;
    uint32_t seq;
    volatile int n_fetch;
    /* freeze: fetch succeeds and latches nothing (stale). fail: fetch errors. */
    volatile bool freeze;
    volatile bool fail;
    /* extrapolate: step() advances the latched angle by the latched speed
     * each PWM period between fetches, as the AS5600 driver does. */
    bool extrapolate;
    q16_t adv_e;
    q16_t adv_m;
} mock_pmsm_rotor_t;

void mock_pmsm_init(mock_pmsm_t *m, const mock_pmsm_params_t *p);
void mock_pmsm_rotor_init(mock_pmsm_rotor_t *r, mock_pmsm_t *m, float offset_e);
/* Adds the plant to the shared ticker (up to two), starting it if needed. */
void mock_pmsm_start(mock_pmsm_t *m);
/* Stops the ticker and forgets every plant. */
void mock_pmsm_stop(void);
/* Latch a fault the way the driver does: outputs off, then the callback. */
void mock_pmsm_trip(mock_pmsm_t *m, esp_foc_fault_reason_t reason);
float mock_pmsm_fe_hz(const mock_pmsm_t *m);
void mock_pmsm_timing_reset(mock_pmsm_t *m);

#ifdef __cplusplus
}
#endif
