/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Adapts esp_foc_observer → esp_foc_rotor_sensor. θe/ωe are published as the
 * observer gives them; θm = θe/pp and ωm = ωe/pp are derived and are only
 * valid modulo 2π/pp, which is why caps() withholds MECH_ABS.
 */
#include <string.h>

#include "esp_check.h"
#include "esp_macros.h"
#include "sdkconfig.h"

#include "espFoC/drivers/esp_foc_rotor_sensorless.h"
#include "espFoC/utils/esp_foc_angle.h"
#include "espFoC/utils/esp_foc_q16.h"

typedef struct {
    esp_foc_rotor_sensor_t iface;
    bool acquired;
    bool inited;
    unsigned pole_pairs;
    esp_foc_observer_t *obs;
    q16_t theta_e;
    q16_t omega_e;
    q16_t pos_q16;
    q16_t vel_q16;
    q16_t inv_pp;
    uint32_t seq;
    uint32_t age_periods;
} esp_foc_sensorless_obj_t;

static esp_foc_sensorless_obj_t s_pool[CONFIG_ESP_FOC_MAX_ROTOR_SENSORS];

static esp_foc_sensorless_obj_t *obj_from(esp_foc_rotor_sensor_t *self)
{
    return __containerof(self, esp_foc_sensorless_obj_t, iface);
}

static const esp_foc_sensorless_obj_t *obj_from_const(const esp_foc_rotor_sensor_t *self)
{
    return __containerof(self, esp_foc_sensorless_obj_t, iface);
}

static q16_t get_position(esp_foc_rotor_sensor_t *self)
{
    return obj_from(self)->pos_q16;
}

static q16_t get_velocity(esp_foc_rotor_sensor_t *self)
{
    return obj_from(self)->vel_q16;
}

static q16_t get_electrical_position(esp_foc_rotor_sensor_t *self)
{
    return obj_from(self)->theta_e;
}

static esp_err_t fetch(esp_foc_rotor_sensor_t *self)
{
    esp_foc_rotor_sensorless_latch(self);
    return ESP_OK;
}

static esp_err_t fetch_start(esp_foc_rotor_sensor_t *self)
{
    (void)self;
    /* The observer is updated by the hot path; there is no transfer to kick. */
    return ESP_ERR_NOT_SUPPORTED;
}

static uint32_t caps(const esp_foc_rotor_sensor_t *self)
{
    (void)self;
    /*
     * Electrical only. θ_m is published as a convenience division by pp, but
     * an electrical observer cannot resolve which of the pp mechanical
     * positions the shaft is in, so MECH_ABS would be a lie.
     */
    return (uint32_t)ESP_FOC_ROTOR_CAP_ELEC_ABS;
}

static void step(esp_foc_rotor_sensor_t *self)
{
    esp_foc_sensorless_obj_t *o = obj_from(self);
    if (o->age_periods < UINT32_MAX) {
        o->age_periods++;
    }
}

static void snapshot(const esp_foc_rotor_sensor_t *self, esp_foc_rotor_state_t *out)
{
    const esp_foc_sensorless_obj_t *o = obj_from_const(self);
    out->theta_e = o->theta_e;
    out->omega_e = o->omega_e;
    out->theta_m = o->pos_q16;
    out->omega_m = o->vel_q16;
    out->seq = o->seq;
    out->age_periods = o->age_periods;
    out->sector = ESP_FOC_ROTOR_SECTOR_UNKNOWN;
    out->valid = o->inited && o->obs != NULL;
    out->moving = o->omega_e != 0;
}

static esp_err_t calibrate_offset(esp_foc_rotor_sensor_t *self, int samples)
{
    (void)samples;
    esp_foc_sensorless_obj_t *o = obj_from(self);
    if (o->obs != NULL) {
        esp_foc_observer_set_theta(o->obs, 0);
        esp_foc_observer_set_omega(o->obs, 0);
    }
    o->theta_e = 0;
    o->omega_e = 0;
    o->pos_q16 = 0;
    o->vel_q16 = 0;
    return ESP_OK;
}

esp_foc_rotor_sensor_t *esp_foc_rotor_sensorless_acquire(unsigned index)
{
    if (index >= CONFIG_ESP_FOC_MAX_ROTOR_SENSORS) {
        return NULL;
    }
    esp_foc_sensorless_obj_t *o = &s_pool[index];
    if (o->acquired) {
        return NULL;
    }
    memset(o, 0, sizeof(*o));
    o->acquired = true;
    o->iface.get_position = get_position;
    o->iface.get_velocity = get_velocity;
    o->iface.fetch = fetch;
    o->iface.fetch_start = fetch_start;
    o->iface.calibrate_offset = calibrate_offset;
    o->iface.caps = caps;
    o->iface.step = step;
    o->iface.snapshot = snapshot;
    o->iface.get_electrical_position = get_electrical_position;
    return &o->iface;
}

void esp_foc_rotor_sensorless_release(esp_foc_rotor_sensor_t *s)
{
    if (s == NULL) {
        return;
    }
    esp_foc_sensorless_obj_t *o = obj_from(s);
    (void)esp_foc_rotor_sensorless_deinit(s);
    o->acquired = false;
}

esp_err_t esp_foc_rotor_sensorless_init(esp_foc_rotor_sensor_t *s,
                                        const esp_foc_rotor_sensorless_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(s != NULL && cfg != NULL, ESP_ERR_INVALID_ARG, "foc_sl",
                        "null");
    ESP_RETURN_ON_FALSE(cfg->observer != NULL, ESP_ERR_INVALID_ARG, "foc_sl",
                        "obs");
    ESP_RETURN_ON_FALSE(cfg->pole_pairs >= 1u, ESP_ERR_INVALID_ARG, "foc_sl",
                        "pp");

    esp_foc_sensorless_obj_t *o = obj_from(s);
    ESP_RETURN_ON_FALSE(o->acquired, ESP_ERR_INVALID_STATE, "foc_sl", "acq");

    o->obs = cfg->observer;
    o->pole_pairs = cfg->pole_pairs;
    o->inv_pp = q16_div(Q16_ONE, q16_from_float((float)cfg->pole_pairs));
    o->theta_e = 0;
    o->omega_e = 0;
    o->pos_q16 = 0;
    o->vel_q16 = 0;
    o->age_periods = 0;
    o->inited = true;
    return ESP_OK;
}

esp_err_t esp_foc_rotor_sensorless_deinit(esp_foc_rotor_sensor_t *s)
{
    if (s == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_foc_sensorless_obj_t *o = obj_from(s);
    o->inited = false;
    o->obs = NULL;
    o->theta_e = 0;
    o->omega_e = 0;
    o->pos_q16 = 0;
    o->vel_q16 = 0;
    return ESP_OK;
}

void esp_foc_rotor_sensorless_latch(esp_foc_rotor_sensor_t *s)
{
    if (s == NULL) {
        return;
    }
    esp_foc_sensorless_obj_t *o = obj_from(s);
    if (!o->inited || o->obs == NULL) {
        return;
    }
    q16_t th_e = esp_foc_observer_get_theta(o->obs);
    q16_t w_e = esp_foc_observer_get_omega(o->obs);
    o->theta_e = q16_wrap_pi(th_e);
    o->omega_e = w_e;
    o->pos_q16 = q16_wrap_pi(q16_mul(th_e, o->inv_pp));
    o->vel_q16 = q16_mul(w_e, o->inv_pp);
    o->seq++;
    o->age_periods = 0;
}
