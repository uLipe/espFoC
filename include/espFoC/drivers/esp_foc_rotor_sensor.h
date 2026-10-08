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

typedef struct esp_foc_rotor_sensor_s esp_foc_rotor_sensor_t;

/** Unknown sector, for sensors with no 60° electrical quantization. */
#define ESP_FOC_ROTOR_SECTOR_UNKNOWN 0xFFu

/**
 * What a sensor actually delivers. caps() is the contract: a getter for an
 * unsupported quantity returns 0 rather than failing, so the bit is the only
 * way to tell "zero because the rotor is there" from "zero because I cannot
 * know".
 */
typedef enum {
    ESP_FOC_ROTOR_CAP_MECH_ABS       = 1u << 0, /* θ_m absolute over a mech rev  */
    ESP_FOC_ROTOR_CAP_ELEC_ABS       = 1u << 1, /* θ_e absolute over an elec rev */
    ESP_FOC_ROTOR_CAP_MECH_MULTITURN = 1u << 2, /* θ_m accumulates past 2π       */
    ESP_FOC_ROTOR_CAP_PREDICT        = 1u << 3, /* step() extrapolates           */
    ESP_FOC_ROTOR_CAP_NEEDS_MAP      = 1u << 4, /* unusable until a map is set   */
} esp_foc_rotor_caps_t;

/**
 * One coherent read of the sensor. A supervisor needs θ_e, ω and valid from
 * the *same* period; five getters racing each other cannot promise that.
 *
 * age_periods counts hot-path steps since the last raw measurement, so a
 * caller can tell a fresh angle from an extrapolated one without a timebase.
 */
typedef struct {
    q16_t theta_e;      /* electrical angle [rad], wrapped to (−π, +π] */
    q16_t omega_e;      /* electrical speed [rad/s] */
    q16_t theta_m;      /* mechanical angle [rad], meaning declared by caps */
    q16_t omega_m;      /* mechanical speed [rad/s] */
    uint32_t seq;       /* increments once per raw measurement */
    uint32_t age_periods;
    uint8_t sector;     /* 0..5, ESP_FOC_ROTOR_SECTOR_UNKNOWN when n/a */
    bool valid;
    bool moving;
} esp_foc_rotor_state_t;

/**
 * Rotor sensor mechanism. Electrical-first: θ_e is what Park consumes, and a
 * hall sensor only knows θ_m modulo 2π/pp, so demanding a mechanical angle
 * from every implementation costs information.
 *
 * Position getters wrap to (−π, +π]. Getters are ISR-safe and return the last
 * latch only.
 *
 * No slot is ever NULL. An implementation binds all of them and reports what
 * it cannot do: esp_err_t slots return ESP_ERR_NOT_SUPPORTED, q16 getters
 * return 0 with the matching caps bit absent.
 */
struct esp_foc_rotor_sensor_s {
    q16_t (*get_position)(esp_foc_rotor_sensor_t *self);
    q16_t (*get_velocity)(esp_foc_rotor_sensor_t *self);
    /**
     * Blocking sample update. Programs HW and waits for the I2C ISR post.
     * Fail holds the last latch (ZOH). Task context only.
     */
    esp_err_t (*fetch)(esp_foc_rotor_sensor_t *self);
    /**
     * Kick one async hardware sample. Returns ESP_ERR_INVALID_STATE if busy.
     * Getters keep returning the previous latch until the transfer completes.
     */
    esp_err_t (*fetch_start)(esp_foc_rotor_sensor_t *self);
    /**
     * Task context. samples >= 1 (recommend 8..32).
     * Performs N hardware reads, averages, installs offset, latches θ≈0 / ω≈0.
     * Caller must not fetch beforehand.
     */
    esp_err_t (*calibrate_offset)(esp_foc_rotor_sensor_t *self, int samples);
    /** Bitwise OR of esp_foc_rotor_caps_t. */
    uint32_t (*caps)(const esp_foc_rotor_sensor_t *self);
    /**
     * Advance the estimate by one hot-path period. dt is fixed at init.
     * ISR-safe, O(1), no division, no timebase read. Sensors with no
     * extrapolator still bind it as a no-op that ages the sample.
     */
    void (*step)(esp_foc_rotor_sensor_t *self);
    /** Fill the whole struct; unsupported fields zeroed. */
    void (*snapshot)(const esp_foc_rotor_sensor_t *self, esp_foc_rotor_state_t *out);
    q16_t (*get_electrical_position)(esp_foc_rotor_sensor_t *self);
};

static inline q16_t esp_foc_rotor_sensor_get_position(esp_foc_rotor_sensor_t *s)
{
    return (s != NULL && s->get_position != NULL) ? s->get_position(s) : 0;
}

static inline q16_t esp_foc_rotor_sensor_get_velocity(esp_foc_rotor_sensor_t *s)
{
    return (s != NULL && s->get_velocity != NULL) ? s->get_velocity(s) : 0;
}

static inline q16_t esp_foc_rotor_sensor_get_electrical_position(esp_foc_rotor_sensor_t *s)
{
    return (s != NULL && s->get_electrical_position != NULL)
               ? s->get_electrical_position(s)
               : 0;
}

static inline esp_err_t esp_foc_rotor_sensor_fetch(esp_foc_rotor_sensor_t *s)
{
    if (s == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return (s->fetch != NULL) ? s->fetch(s) : ESP_ERR_NOT_SUPPORTED;
}

static inline esp_err_t esp_foc_rotor_sensor_fetch_start(esp_foc_rotor_sensor_t *s)
{
    if (s == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return (s->fetch_start != NULL) ? s->fetch_start(s) : ESP_ERR_NOT_SUPPORTED;
}

static inline esp_err_t esp_foc_rotor_sensor_calibrate_offset(esp_foc_rotor_sensor_t *s,
                                                             int samples)
{
    if (s == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return (s->calibrate_offset != NULL) ? s->calibrate_offset(s, samples)
                                         : ESP_ERR_NOT_SUPPORTED;
}

static inline uint32_t esp_foc_rotor_sensor_caps(const esp_foc_rotor_sensor_t *s)
{
    return (s != NULL && s->caps != NULL) ? s->caps(s) : 0u;
}

static inline bool esp_foc_rotor_sensor_has_cap(const esp_foc_rotor_sensor_t *s,
                                               esp_foc_rotor_caps_t cap)
{
    return (esp_foc_rotor_sensor_caps(s) & (uint32_t)cap) != 0u;
}

static inline void esp_foc_rotor_sensor_step(esp_foc_rotor_sensor_t *s)
{
    if (s != NULL && s->step != NULL) {
        s->step(s);
    }
}

static inline void esp_foc_rotor_sensor_snapshot(const esp_foc_rotor_sensor_t *s,
                                                 esp_foc_rotor_state_t *out)
{
    if (out == NULL) {
        return;
    }
    if (s != NULL && s->snapshot != NULL) {
        s->snapshot(s, out);
        return;
    }
    out->theta_e = 0;
    out->omega_e = 0;
    out->theta_m = 0;
    out->omega_m = 0;
    out->seq = 0;
    out->age_periods = 0;
    out->sector = ESP_FOC_ROTOR_SECTOR_UNKNOWN;
    out->valid = false;
    out->moving = false;
}

#ifdef __cplusplus
}
#endif
