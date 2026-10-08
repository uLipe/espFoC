/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#include "esp_foc_hall_soc_gate.h"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_macros.h"
#include "sdkconfig.h"

#include "espFoC/drivers/esp_foc_rotor_hall.h"
#include "espFoC/motor_control/esp_foc_rotor_est.h"
#include "espFoC/utils/esp_foc_angle.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"

#include "hall_timestamp.h"

static const char *TAG = "foc_hall";

#define HALL_SECTORS 6

/*
 * The six legal codes in travel order. Every neighbouring pair differs in
 * exactly one bit, which is the property the whole validity and direction test
 * rests on: it needs no wiring knowledge and no calibration, so it is usable
 * from the first power-up. 000 and 111 are off the cycle entirely — an open
 * line or a dead sensor.
 */
static const uint8_t k_hall_seq[HALL_SECTORS] = {1u, 3u, 2u, 6u, 4u, 5u};

typedef struct {
    esp_foc_rotor_sensor_t iface;
    bool acquired;
    bool inited;

    int gpio[3];
    unsigned pole_pairs;
    q16_t inv_pp;

    /* code (0..7) → sector index (0..5), 0xFF for the two illegal codes. */
    uint8_t code_to_sector[8];
    /* sector index → electrical angle of the boundary that opens it. */
    q16_t theta_edge[HALL_SECTORS];
    bool have_map;

    esp_foc_hall_ts_t ts;
    esp_foc_rotor_est_t est;

    uint8_t code;
    uint8_t sector;
    int8_t dir;

    /* Published state, refreshed once per step(). */
    q16_t theta_e;
    q16_t omega_e;
    q16_t theta_m;
    q16_t omega_m;
    uint32_t seq;
    uint32_t age_periods;
    bool valid;

    uint64_t last_ticks;
    bool have_last_ticks;
    /*
     * A capture seen before the level read caught up with it. The two reads are
     * tens of nanoseconds apart, so an edge can land between them; holding the
     * timestamp for exactly one step turns what would be a lost measurement
     * plus two bogus health counts into the correct measurement.
     */
    uint64_t pend_ticks;
    bool pend_valid;

    esp_foc_rotor_hall_health_t health;
} esp_foc_hall_obj_t;

static esp_foc_hall_obj_t s_pool[CONFIG_ESP_FOC_MAX_ROTOR_SENSORS];

static esp_foc_hall_obj_t *obj_from(esp_foc_rotor_sensor_t *self)
{
    return __containerof(self, esp_foc_hall_obj_t, iface);
}

static const esp_foc_hall_obj_t *obj_from_const(const esp_foc_rotor_sensor_t *self)
{
    return __containerof(self, esp_foc_hall_obj_t, iface);
}

static void build_sector_table(esp_foc_hall_obj_t *o, int dir_sign)
{
    memset(o->code_to_sector, 0xFF, sizeof(o->code_to_sector));
    for (int i = 0; i < HALL_SECTORS; i++) {
        /* Reversing the cycle is what dir_sign means: it changes which way
         * round counts as forward without touching the angle ladder. */
        int src = (dir_sign < 0) ? ((HALL_SECTORS - i) % HALL_SECTORS) : i;
        o->code_to_sector[k_hall_seq[src]] = (uint8_t)i;
    }
}

static void install_nominal_map(esp_foc_hall_obj_t *o)
{
    for (int i = 0; i < HALL_SECTORS; i++) {
        o->theta_edge[i] = q16_wrap_pi((q16_t)(((int64_t)Q16_TWO_PI * i) / HALL_SECTORS));
    }
    o->have_map = false;
}

static void install_map(esp_foc_hall_obj_t *o, const float theta_edge_rad[6])
{
    for (int i = 0; i < HALL_SECTORS; i++) {
        o->theta_edge[i] = q16_wrap_pi(q16_from_float(theta_edge_rad[i]));
    }
    o->have_map = true;
}

static uint8_t read_code(const esp_foc_hall_obj_t *o)
{
    uint32_t c = 0;
    c |= (uint32_t)(gpio_ll_get_level(&GPIO, o->gpio[0]) != 0) << 0;
    c |= (uint32_t)(gpio_ll_get_level(&GPIO, o->gpio[1]) != 0) << 1;
    c |= (uint32_t)(gpio_ll_get_level(&GPIO, o->gpio[2]) != 0) << 2;
    return (uint8_t)c;
}

/**
 * Angle of the boundary the rotor just crossed.
 *
 * Which boundary that is depends on the direction, not only on where it landed:
 * entering a sector forwards crosses the boundary that opens it, entering the
 * same sector backwards crosses the one that closes it — which is the next
 * sector's opening boundary. Getting this wrong costs a fixed 60° of angle
 * error in reverse, so it is worth the extra term.
 */
static q16_t boundary_angle(const esp_foc_hall_obj_t *o, uint8_t sector, int dir)
{
    if (dir >= 0) {
        return o->theta_edge[sector];
    }
    return o->theta_edge[(sector + 1u) % HALL_SECTORS];
}

static void publish(esp_foc_hall_obj_t *o)
{
    q16_t th_e = esp_foc_rotor_est_get_theta(&o->est);
    q16_t w_e = esp_foc_rotor_est_get_omega(&o->est);

    o->theta_e = th_e;
    o->omega_e = w_e;
    o->theta_m = q16_wrap_pi(q16_mul(th_e, o->inv_pp));
    o->omega_m = q16_mul(w_e, o->inv_pp);

    o->health.bounce = o->est.rejected;
    o->health.stale = o->est.stale;
    o->health.clamped = o->est.clamped;
}

static void api_step(esp_foc_rotor_sensor_t *self)
{
    esp_foc_hall_obj_t *o = obj_from(self);
    if (!o->inited) {
        return;
    }

    /* Load: the two things a transition is judged by. */
    uint8_t code = read_code(o);
    uint64_t ticks = 0;
    bool captured = esp_foc_hall_ts_poll(&o->ts, &ticks);

    if (code != o->code) {
        if (!captured && o->pend_valid) {
            ticks = o->pend_ticks;
            captured = true;
        }
        o->pend_valid = false;

        uint8_t sector = o->code_to_sector[code];

        if (sector == ESP_FOC_ROTOR_SECTOR_UNKNOWN) {
            o->health.illegal_code++;
            o->valid = false;
            o->code = code;
            o->sector = ESP_FOC_ROTOR_SECTOR_UNKNOWN;
        } else if (o->sector == ESP_FOC_ROTOR_SECTOR_UNKNOWN) {
            /* Recovering from an illegal code. Position is known to a sector
             * again; direction and speed are not. */
            o->code = code;
            o->sector = sector;
        } else {
            uint8_t diff = (uint8_t)((sector + HALL_SECTORS - o->sector) % HALL_SECTORS);
            int dir = o->dir;

            if (diff == 1u) {
                dir = 1;
            } else if (diff == HALL_SECTORS - 1u) {
                dir = -1;
            } else {
                /*
                 * More than one bit moved, so at least one edge was missed.
                 * ±2 is still resolvable by carrying the last known direction,
                 * and Δθ then covers the whole jump, so ω stays right — but
                 * the health counter has to rise either way.
                 */
                o->health.multi_bit++;
                if (diff == 3u || dir == 0) {
                    dir = 0;
                }
            }

            if (dir != 0) {
                q16_t theta_meas = boundary_angle(o, sector, dir);

                if (captured) {
                    if (o->have_last_ticks) {
                        uint64_t d = ticks - o->last_ticks;
                        if (d > o->health.worst_dticks) {
                            o->health.worst_dticks = d;
                        }
                    }
                    o->last_ticks = ticks;
                    o->have_last_ticks = true;

                    esp_foc_rotor_est_on_edge(&o->est, theta_meas, ticks, dir);
                    o->health.edges++;
                    o->seq++;
                    o->age_periods = 0;
                    o->valid = true;
                } else {
                    /*
                     * The code moved and the capture did not. The timestamp
                     * hardware is not answering — a cleared ETM channel, a
                     * wrong channel index, a swapped pin. Refusing the
                     * measurement is deliberate: a loud gap beats a quiet
                     * wrong angle during bring-up.
                     */
                    o->health.capture_lost++;
                }
            }

            o->dir = (int8_t)dir;
            o->code = code;
            o->sector = sector;
        }
    } else {
        /* A capture with the code unchanged is either the read race above, in
         * which case the next step claims it, or a glitch that came back. */
        if (o->pend_valid) {
            o->health.spurious++;
        }
        o->pend_valid = captured;
        o->pend_ticks = ticks;
    }

    esp_foc_rotor_est_step(&o->est);
    if (o->age_periods < UINT32_MAX) {
        o->age_periods++;
    }
    publish(o);
}

static q16_t api_get_position(esp_foc_rotor_sensor_t *self)
{
    return obj_from(self)->theta_m;
}

static q16_t api_get_velocity(esp_foc_rotor_sensor_t *self)
{
    return obj_from(self)->omega_m;
}

static q16_t api_get_electrical_position(esp_foc_rotor_sensor_t *self)
{
    return obj_from(self)->theta_e;
}

static esp_err_t api_fetch(esp_foc_rotor_sensor_t *self)
{
    (void)self;
    /* Event driven: there is no blocking sample to take. */
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t api_fetch_start(esp_foc_rotor_sensor_t *self)
{
    (void)self;
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t api_calibrate_offset(esp_foc_rotor_sensor_t *self, int samples)
{
    (void)self;
    (void)samples;
    /* Zeroing an angle is not what this sensor needs; it needs the six
     * boundary angles, which is map discovery's job. */
    return ESP_ERR_NOT_SUPPORTED;
}

static uint32_t api_caps(const esp_foc_rotor_sensor_t *self)
{
    const esp_foc_hall_obj_t *o = obj_from_const(self);
    uint32_t c = (uint32_t)ESP_FOC_ROTOR_CAP_ELEC_ABS |
                 (uint32_t)ESP_FOC_ROTOR_CAP_PREDICT;
    if (!o->have_map) {
        c |= (uint32_t)ESP_FOC_ROTOR_CAP_NEEDS_MAP;
    }
    return c;
}

static void api_snapshot(const esp_foc_rotor_sensor_t *self, esp_foc_rotor_state_t *out)
{
    const esp_foc_hall_obj_t *o = obj_from_const(self);
    out->theta_e = o->theta_e;
    out->omega_e = o->omega_e;
    out->theta_m = o->theta_m;
    out->omega_m = o->omega_m;
    out->seq = o->seq;
    out->age_periods = o->age_periods;
    out->sector = o->sector;
    out->valid = o->valid;
    out->moving = esp_foc_rotor_est_is_moving(&o->est);
}

static void bind_vtable(esp_foc_hall_obj_t *o)
{
    o->iface.get_position = api_get_position;
    o->iface.get_velocity = api_get_velocity;
    o->iface.fetch = api_fetch;
    o->iface.fetch_start = api_fetch_start;
    o->iface.calibrate_offset = api_calibrate_offset;
    o->iface.caps = api_caps;
    o->iface.step = api_step;
    o->iface.snapshot = api_snapshot;
    o->iface.get_electrical_position = api_get_electrical_position;
}

esp_foc_rotor_sensor_t *esp_foc_rotor_hall_acquire(unsigned index)
{
    if (index >= CONFIG_ESP_FOC_MAX_ROTOR_SENSORS) {
        return NULL;
    }
    esp_foc_hall_obj_t *o = &s_pool[index];
    if (o->acquired) {
        return NULL;
    }
    memset(o, 0, sizeof(*o));
    o->acquired = true;
    o->sector = ESP_FOC_ROTOR_SECTOR_UNKNOWN;
    bind_vtable(o);
    return &o->iface;
}

void esp_foc_rotor_hall_release(esp_foc_rotor_sensor_t *s)
{
    if (s == NULL) {
        return;
    }
    esp_foc_hall_obj_t *o = obj_from(s);
    if (o->inited) {
        (void)esp_foc_rotor_hall_deinit(s);
    }
    o->acquired = false;
}

esp_err_t esp_foc_rotor_hall_init(esp_foc_rotor_sensor_t *s,
                                  const esp_foc_rotor_hall_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(s != NULL && cfg != NULL, ESP_ERR_INVALID_ARG, TAG, "null");
    esp_foc_hall_obj_t *o = obj_from(s);
    ESP_RETURN_ON_FALSE(o->acquired && !o->inited, ESP_ERR_INVALID_STATE, TAG, "state");
    ESP_RETURN_ON_FALSE(cfg->pole_pairs >= 1u, ESP_ERR_INVALID_ARG, TAG, "pp");
    ESP_RETURN_ON_FALSE(cfg->pwm_hz > 0u, ESP_ERR_INVALID_ARG, TAG, "pwm_hz");
    for (int i = 0; i < 3; i++) {
        ESP_RETURN_ON_FALSE(cfg->gpio[i] >= 0, ESP_ERR_INVALID_ARG, TAG, "gpio");
        for (int j = i + 1; j < 3; j++) {
            ESP_RETURN_ON_FALSE(cfg->gpio[i] != cfg->gpio[j],
                                ESP_ERR_INVALID_ARG, TAG, "gpio dup");
        }
    }

    for (int i = 0; i < 3; i++) {
        o->gpio[i] = cfg->gpio[i];
    }
    o->pole_pairs = cfg->pole_pairs;
    o->inv_pp = q16_div(Q16_ONE, q16_from_float((float)cfg->pole_pairs));

    build_sector_table(o, cfg->dir_sign);
    if (cfg->have_map) {
        install_map(o, cfg->theta_edge_rad);
    } else {
        install_nominal_map(o);
    }

    esp_foc_hall_ts_cfg_t tscfg = {
        .kind = cfg->ts_kind,
        .timer_group = cfg->timer_group,
        .require_etm_ready = cfg->require_etm_ready,
        .irq_level = cfg->irq_level,
    };
    for (int i = 0; i < 3; i++) {
        tscfg.gpio[i] = cfg->gpio[i];
        tscfg.etm_channel[i] = cfg->etm_channel[i];
    }
    esp_err_t err = esp_foc_hall_ts_init(&o->ts, &tscfg);
    if (err != ESP_OK) {
        return err;
    }

    esp_foc_rotor_est_config_t ecfg;
    esp_foc_rotor_est_config_default(&ecfg,
                                     esp_foc_hall_ts_tick_hz(&o->ts),
                                     cfg->pwm_hz,
                                     (float)(2.0 * 3.14159265358979 / 6.0),
                                     cfg->standstill_ms > 0.0f ? cfg->standstill_ms
                                                               : 150.0f);
    if (cfg->lambda_theta > 0.0f) {
        ecfg.lambda_theta = q16_from_float(cfg->lambda_theta);
    }
    if (cfg->lambda_omega > 0.0f) {
        ecfg.lambda_omega = q16_from_float(cfg->lambda_omega);
    }

    err = esp_foc_rotor_est_init(&o->est, &ecfg);
    if (err != ESP_OK) {
        esp_foc_hall_ts_deinit(&o->ts);
        return err;
    }

    o->code = read_code(o);
    o->sector = o->code_to_sector[o->code];
    o->dir = 0;
    o->valid = false;
    o->health.etm_cold_start = esp_foc_hall_ts_etm_cold_start(&o->ts);
    o->inited = true;

    ESP_LOGI(TAG, "hall ready gpio=%d/%d/%d pp=%u pwm=%luHz code=%u sector=%u map=%d",
             cfg->gpio[0], cfg->gpio[1], cfg->gpio[2], cfg->pole_pairs,
             (unsigned long)cfg->pwm_hz, o->code, o->sector, o->have_map ? 1 : 0);
    return ESP_OK;
}

esp_err_t esp_foc_rotor_hall_deinit(esp_foc_rotor_sensor_t *s)
{
    if (s == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_foc_hall_obj_t *o = obj_from(s);
    if (!o->inited) {
        return ESP_OK;
    }
    esp_foc_hall_ts_deinit(&o->ts);
    o->inited = false;
    return ESP_OK;
}

esp_err_t esp_foc_rotor_hall_rearm(esp_foc_rotor_sensor_t *s)
{
    if (s == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_foc_hall_obj_t *o = obj_from(s);
    if (!o->inited) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = esp_foc_hall_ts_rearm(&o->ts);
    if (err == ESP_OK) {
        o->have_last_ticks = false;
        esp_foc_rotor_est_reset(&o->est);
        o->valid = false;
    }
    return err;
}

esp_err_t esp_foc_rotor_hall_set_map(esp_foc_rotor_sensor_t *s,
                                     const float theta_edge_rad[6])
{
    if (s == NULL || theta_edge_rad == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_foc_hall_obj_t *o = obj_from(s);
    if (!o->acquired) {
        return ESP_ERR_INVALID_STATE;
    }
    install_map(o, theta_edge_rad);
    return ESP_OK;
}

void esp_foc_rotor_hall_get_health(const esp_foc_rotor_sensor_t *s,
                                   esp_foc_rotor_hall_health_t *out)
{
    if (out == NULL) {
        return;
    }
    if (s == NULL) {
        memset(out, 0, sizeof(*out));
        return;
    }
    const esp_foc_hall_obj_t *o = obj_from_const(s);
    *out = o->health;
    out->ts_healthy = esp_foc_hall_ts_healthy(&o->ts);
}

uint8_t esp_foc_rotor_hall_raw_code(const esp_foc_rotor_sensor_t *s)
{
    return (s != NULL) ? obj_from_const(s)->code : 0u;
}
