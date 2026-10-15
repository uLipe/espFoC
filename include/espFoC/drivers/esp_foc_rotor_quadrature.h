/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "espFoC/drivers/esp_foc_rotor_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Counts per revolution are 4 × ppr and must fit the 16-bit PCNT limit. */
#define ESP_FOC_ROTOR_QUADRATURE_MAX_PPR 8191u

/**
 * Incremental A/B encoder on one PCNT unit, decoded x4 in hardware.
 *
 * The unit counts within ±CPR and its limit interrupt carries whole turns, so
 * the count is cumulative without touching every edge. step() reads the
 * counter every hot-path period: there is no extrapolation and no PREDICT cap.
 * fetch() reads it too and derives ω_m from the count delta over dt_seconds.
 *
 * θ is relative to the count at init until calibrate_offset() zeroes it; Z
 * (index) is not used.
 */
typedef struct {
    int gpio_a;
    int gpio_b;
    int pcnt_group;     /* 0 on single-instance chips */
    int pcnt_unit;      /* owned by this sensor, no other user on the group */
    uint32_t ppr;       /* pulses per rev per channel, 1..MAX_PPR */
    /*
     * Pulses shorter than this are ignored [ns]. 0 → filter off. Keep it below
     * half the shortest edge spacing at top speed: 2048 PPR at 6000 rpm puts
     * edges 1.2 µs apart.
     */
    uint32_t glitch_ns;
    float dt_seconds;   /* fetch() period, sets the ω_m scale */
    bool invert;
    /* 0 → get_electrical_position() stays 0 and ELEC_ABS is off. */
    unsigned pole_pairs;
    /* Limit interrupt level, 1 or 2; must stay below the inverter's. 0 → 1. */
    int irq_level;
} esp_foc_rotor_quadrature_config_t;

esp_foc_rotor_sensor_t *esp_foc_rotor_quadrature_acquire(unsigned index);
void esp_foc_rotor_quadrature_release(esp_foc_rotor_sensor_t *s);

esp_err_t esp_foc_rotor_quadrature_init(esp_foc_rotor_sensor_t *s,
                                        const esp_foc_rotor_quadrature_config_t *cfg);
esp_err_t esp_foc_rotor_quadrature_deinit(esp_foc_rotor_sensor_t *s);

/**
 * Cumulative signed count since init or the last calibrate_offset(), invert
 * applied. Reads the hardware. Task context.
 */
int64_t esp_foc_rotor_quadrature_get_count(const esp_foc_rotor_sensor_t *s);
/** Counts per revolution (4 × ppr). */
uint32_t esp_foc_rotor_quadrature_get_cpr(const esp_foc_rotor_sensor_t *s);

void esp_foc_rotor_quadrature_set_invert(esp_foc_rotor_sensor_t *s, bool invert);
void esp_foc_rotor_quadrature_set_pole_pairs(esp_foc_rotor_sensor_t *s,
                                             unsigned pole_pairs);

#ifdef __cplusplus
}
#endif
