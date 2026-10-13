/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_macros.h"
#include "sdkconfig.h"

#include "espFoC/drivers/esp_foc_rotor_as5600.h"
#include "espFoC/osal/esp_foc_osal.h"
#include "espFoC/utils/esp_foc_angle.h"
#include "i2c_hal_bus.h"

static const char *TAG = "foc_as5600";

#define AS5600_ADDR7        0x36u
#define AS5600_REG_CONF_H   0x07u
#define AS5600_REG_ANGLE_H  0x0Eu
#define AS5600_CPR          4096u
#define AS5600_MASK         0x0FFFu

/*
 * CONF for motor control: PM=NOM, HYST=off, SF=2x (fastest slow filter).
 * A part left in LPM3 only refreshes ANGLE at 10 Hz, which aliases any shaft
 * speed above ~5 rev/s and makes a synchronous spin look like it slipped.
 * High byte = CONF[13:8] (WD|FTH|SF), low byte = CONF[7:0] (PWMF|OUTS|HYST|PM).
 */
#define AS5600_CONF_H_NOM   0x03u
#define AS5600_CONF_L_NOM   0x00u

typedef struct {
    esp_foc_rotor_sensor_t iface;
    bool acquired;
    bool inited;
    bool invert;
    q16_t inv_dt_q16;
    q16_t offset_rad;
    q16_t pos_q16;
    q16_t vel_q16;
    q16_t prev_mech_q16;
    unsigned pole_pairs;
    uint32_t dt_q32;
    uint16_t prev_raw;
    bool have_prev;
    uint32_t missed;
    uint32_t xfer_fail;
    uint32_t xfer_ok;
    uint32_t seq;
    uint32_t age_periods;
    bool ptr_armed;
    uint8_t async_reg;
    uint8_t async_rx[2];
    esp_foc_i2c_hal_bus_t bus;
} esp_foc_as5600_obj_t;

static esp_foc_as5600_obj_t s_pool[CONFIG_ESP_FOC_MAX_ROTOR_SENSORS];

static esp_foc_as5600_obj_t *obj_from(esp_foc_rotor_sensor_t *self)
{
    return __containerof(self, esp_foc_as5600_obj_t, iface);
}

static const esp_foc_as5600_obj_t *obj_from_const(const esp_foc_rotor_sensor_t *self)
{
    return __containerof(self, esp_foc_as5600_obj_t, iface);
}

static q16_t raw_to_mech_rad(uint16_t raw12)
{
    int64_t t = ((int64_t)(raw12 & AS5600_MASK) * (int64_t)Q16_TWO_PI) >> 12;
    return q16_wrap_pi((q16_t)t);
}

static q16_t apply_offset_invert(esp_foc_as5600_obj_t *o, q16_t mech)
{
    q16_t p = q16_wrap_pi(q16_sub(mech, o->offset_rad));
    if (o->invert) {
        p = q16_neg(p);
        p = q16_wrap_pi(p);
    }
    return p;
}

static void apply_raw_sample(esp_foc_as5600_obj_t *o, uint16_t raw)
{
    q16_t mech = raw_to_mech_rad(raw);
    o->pos_q16 = apply_offset_invert(o, mech);
    o->seq++;
    o->age_periods = 0;

    if (o->have_prev && raw == o->prev_raw) {
        if (o->missed < 10000u) {
            o->missed++;
        }
        if (o->missed > 16u) {
            o->vel_q16 = 0;
        }
        return;
    }

    uint32_t n = o->missed + 1u;
    o->missed = 0;

    if (n > 16u) {
        o->prev_raw = raw;
        o->prev_mech_q16 = mech;
        o->have_prev = true;
        return;
    }

    if (o->have_prev) {
        q16_t d = q16_angle_delta(o->prev_mech_q16, mech);
        if (o->invert) {
            d = q16_neg(d);
        }
        o->vel_q16 = (q16_t)((int64_t)q16_mul(d, o->inv_dt_q16) / (int64_t)n);
    }
    o->prev_raw = raw;
    o->prev_mech_q16 = mech;
    o->have_prev = true;
}

static q16_t elec_from_mech(const esp_foc_as5600_obj_t *o, q16_t th_m)
{
    if (o->pole_pairs == 0u) {
        return 0;
    }
    return q16_wrap_pi((q16_t)((int64_t)th_m * (int64_t)o->pole_pairs));
}

static q16_t omega_e_from_m(const esp_foc_as5600_obj_t *o, q16_t w_m)
{
    if (o->pole_pairs == 0u) {
        return 0;
    }
    return (q16_t)((int64_t)w_m * (int64_t)o->pole_pairs);
}

static esp_err_t configure_conf(esp_foc_as5600_obj_t *o, uint16_t *out_conf)
{
    const uint8_t wr[3] = { AS5600_REG_CONF_H, AS5600_CONF_H_NOM, AS5600_CONF_L_NOM };
    esp_err_t err = esp_foc_i2c_hal_write(&o->bus, AS5600_ADDR7, wr, sizeof(wr));
    if (err != ESP_OK) {
        return err;
    }

    uint8_t reg = AS5600_REG_CONF_H;
    uint8_t rd[2] = {0, 0};
    err = esp_foc_i2c_hal_write_read(&o->bus, AS5600_ADDR7, &reg, 1, rd, sizeof(rd));
    if (err != ESP_OK) {
        return err;
    }
    *out_conf = (uint16_t)(((uint16_t)rd[0] << 8) | rd[1]);
    return ESP_OK;
}

static esp_err_t read_raw12(esp_foc_as5600_obj_t *o, uint16_t *out)
{
    uint8_t reg = AS5600_REG_ANGLE_H;
    uint8_t buf[2] = {0, 0};
    esp_err_t err = esp_foc_i2c_hal_write_read(&o->bus, AS5600_ADDR7, &reg, 1, buf, 2);
    if (err != ESP_OK) {
        o->xfer_fail++;
        o->ptr_armed = false;
        return err;
    }
    o->xfer_ok++;
    o->ptr_armed = true;
    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    *out = (uint16_t)(raw & AS5600_MASK);
    return ESP_OK;
}

static void async_done_cb(void *arg, esp_err_t err)
{
    esp_foc_as5600_obj_t *o = (esp_foc_as5600_obj_t *)arg;
    if (err != ESP_OK) {
        o->xfer_fail++;
        if (o->missed < 10000u) {
            o->missed++;
        }
        return;
    }

    o->xfer_ok++;
    o->ptr_armed = true;
    uint16_t raw = ((uint16_t)o->async_rx[0] << 8) | o->async_rx[1];
    apply_raw_sample(o, (uint16_t)(raw & AS5600_MASK));
}

static q16_t api_get_position(esp_foc_rotor_sensor_t *self)
{
    return obj_from(self)->pos_q16;
}

static q16_t api_get_velocity(esp_foc_rotor_sensor_t *self)
{
    return obj_from(self)->vel_q16;
}

static esp_err_t api_fetch(esp_foc_rotor_sensor_t *self)
{
    esp_foc_as5600_obj_t *o = obj_from(self);
    if (!o->inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!esp_foc_in_task_context()) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * After init the ANGLE pointer stays put. Kick a current-address read,
     * block on the I2C ISR post, convert in this task. Fail keeps the last
     * angle (ZOH) and does not move the pointer.
     */
    if (!o->ptr_armed) {
        uint16_t raw = 0;
        esp_err_t err = read_raw12(o, &raw);
        if (err != ESP_OK) {
            if (o->missed < 10000u) {
                o->missed++;
            }
            return err;
        }
        apply_raw_sample(o, raw);
        return ESP_OK;
    }

    esp_err_t err = esp_foc_i2c_hal_read(&o->bus, AS5600_ADDR7, o->async_rx, 2);
    if (err != ESP_OK) {
        o->xfer_fail++;
        if (o->missed < 10000u) {
            o->missed++;
        }
        /*
         * An aborted transfer leaves the device's register pointer unknown, and
         * a current-address read from anywhere but ANGLE still returns two
         * plausible bytes. Re-address before trusting the next one.
         */
        o->ptr_armed = false;
        return err;
    }

    o->xfer_ok++;
    uint16_t raw = ((uint16_t)o->async_rx[0] << 8) | o->async_rx[1];
    apply_raw_sample(o, (uint16_t)(raw & AS5600_MASK));
    return ESP_OK;
}

static esp_err_t api_fetch_start(esp_foc_rotor_sensor_t *self)
{
    esp_foc_as5600_obj_t *o = obj_from(self);
    if (!o->inited) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * ANGLE (0x0E) suppresses pointer increment. After one setup write, a
     * current-address read is START+R, 2 bytes, NACK, STOP — no register write.
     * Reload the pointer after any failed xfer.
     */
    if (o->ptr_armed) {
        return esp_foc_i2c_hal_write_read_async(&o->bus,
                                                AS5600_ADDR7,
                                                NULL,
                                                0,
                                                o->async_rx,
                                                2,
                                                async_done_cb,
                                                o);
    }

    o->async_reg = AS5600_REG_ANGLE_H;
    return esp_foc_i2c_hal_write_read_async(&o->bus,
                                            AS5600_ADDR7,
                                            &o->async_reg,
                                            1,
                                            o->async_rx,
                                            2,
                                            async_done_cb,
                                            o);
}

static esp_err_t api_calibrate_offset(esp_foc_rotor_sensor_t *self, int samples)
{
    esp_foc_as5600_obj_t *o = obj_from(self);
    if (!o->inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (samples < 1) {
        return ESP_ERR_INVALID_ARG;
    }
    if (esp_foc_i2c_hal_busy(&o->bus)) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t first = 0;
    esp_err_t err = read_raw12(o, &first);
    if (err != ESP_OK) {
        return err;
    }

    int64_t acc = (int64_t)first;
    for (int i = 1; i < samples; i++) {
        uint16_t raw = 0;
        err = read_raw12(o, &raw);
        if (err != ESP_OK) {
            return err;
        }
        int32_t d = (int32_t)raw - (int32_t)first;
        if (d > (int32_t)(AS5600_CPR / 2u)) {
            d -= (int32_t)AS5600_CPR;
        } else if (d < -(int32_t)(AS5600_CPR / 2u)) {
            d += (int32_t)AS5600_CPR;
        }
        acc += (int64_t)first + (int64_t)d;
    }

    int32_t avg = (int32_t)(acc / (int64_t)samples);
    while (avg < 0) {
        avg += (int32_t)AS5600_CPR;
    }
    avg &= (int32_t)AS5600_MASK;

    o->offset_rad = raw_to_mech_rad((uint16_t)avg);
    o->pos_q16 = 0;
    o->vel_q16 = 0;
    o->have_prev = false;
    o->prev_raw = 0;
    o->missed = 0;
    return ESP_OK;
}

static uint32_t api_caps(const esp_foc_rotor_sensor_t *self)
{
    const esp_foc_as5600_obj_t *o = obj_from_const(self);
    uint32_t c = (uint32_t)ESP_FOC_ROTOR_CAP_MECH_ABS;
    if (o->pole_pairs != 0u) {
        c |= (uint32_t)ESP_FOC_ROTOR_CAP_ELEC_ABS;
    }
    if (o->dt_q32 != 0u) {
        c |= (uint32_t)ESP_FOC_ROTOR_CAP_PREDICT;
    }
    return c;
}

static q16_t api_get_electrical_position(esp_foc_rotor_sensor_t *self)
{
    esp_foc_as5600_obj_t *o = obj_from(self);
    return elec_from_mech(o, o->pos_q16);
}

static void api_step(esp_foc_rotor_sensor_t *self)
{
    esp_foc_as5600_obj_t *o = obj_from(self);
    if (!o->inited) {
        return;
    }
    if ((o->dt_q32 != 0u) && (o->vel_q16 != 0)) {
        q16_t adv = (q16_t)(((int64_t)o->vel_q16 * (int64_t)o->dt_q32) >> 32);
        o->pos_q16 = q16_wrap_pi(q16_add(o->pos_q16, adv));
    }
    if (o->age_periods < UINT32_MAX) {
        o->age_periods++;
    }
}

static void api_snapshot(const esp_foc_rotor_sensor_t *self, esp_foc_rotor_state_t *out)
{
    const esp_foc_as5600_obj_t *o = obj_from_const(self);
    out->theta_m = o->pos_q16;
    out->omega_m = o->vel_q16;
    out->theta_e = elec_from_mech(o, o->pos_q16);
    out->omega_e = omega_e_from_m(o, o->vel_q16);
    out->seq = o->seq;
    out->age_periods = o->age_periods;
    out->sector = ESP_FOC_ROTOR_SECTOR_UNKNOWN;
    out->valid = o->inited && o->have_prev;
    out->moving = o->vel_q16 != 0;
}

static void bind_vtable(esp_foc_as5600_obj_t *o)
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

esp_foc_rotor_sensor_t *esp_foc_rotor_as5600_acquire(unsigned index)
{
    if (index >= CONFIG_ESP_FOC_MAX_ROTOR_SENSORS) {
        return NULL;
    }
    esp_foc_as5600_obj_t *o = &s_pool[index];
    if (o->acquired) {
        return NULL;
    }
    memset(o, 0, sizeof(*o));
    o->acquired = true;
    bind_vtable(o);
    return &o->iface;
}

void esp_foc_rotor_as5600_release(esp_foc_rotor_sensor_t *s)
{
    if (s == NULL) {
        return;
    }
    esp_foc_as5600_obj_t *o = obj_from(s);
    if (o->inited) {
        (void)esp_foc_rotor_as5600_deinit(s);
    }
    o->acquired = false;
}

esp_err_t esp_foc_rotor_as5600_init(esp_foc_rotor_sensor_t *s,
                                    const esp_foc_rotor_as5600_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(s != NULL && cfg != NULL, ESP_ERR_INVALID_ARG, TAG, "null");
    esp_foc_as5600_obj_t *o = obj_from(s);
    ESP_RETURN_ON_FALSE(o->acquired && !o->inited, ESP_ERR_INVALID_STATE, TAG, "state");
    ESP_RETURN_ON_FALSE(cfg->dt_seconds > 0.0f, ESP_ERR_INVALID_ARG, TAG, "dt");

    uint32_t hz = cfg->i2c_hz != 0u ? cfg->i2c_hz : 400000u;
    esp_err_t err = esp_foc_i2c_hal_bus_init(&o->bus, cfg->i2c_port, cfg->sda, cfg->scl, hz);
    if (err != ESP_OK) {
        return err;
    }

    o->invert = cfg->invert;
    o->pole_pairs = cfg->pole_pairs;
    o->dt_q32 = (cfg->pwm_hz > 0u)
                    ? (uint32_t)(((uint64_t)1u << 32) / (uint64_t)cfg->pwm_hz)
                    : 0u;
    o->inv_dt_q16 = q16_from_float(1.0f / cfg->dt_seconds);
    o->offset_rad = 0;
    o->pos_q16 = 0;
    o->vel_q16 = 0;
    o->have_prev = false;
    o->prev_raw = 0;
    o->missed = 0;
    o->xfer_fail = 0;
    o->xfer_ok = 0;
    o->inited = true;

    uint16_t probe = 0;
    err = read_raw12(o, &probe);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "AS5600 probe read failed: %s (sda=%d scl=%d)",
                 esp_err_to_name(err), cfg->sda, cfg->scl);
        esp_foc_i2c_hal_bus_deinit(&o->bus);
        o->inited = false;
        return err;
    }

    uint16_t conf = 0;
    esp_err_t cerr = configure_conf(o, &conf);
    if (cerr != ESP_OK) {
        ESP_LOGW(TAG, "AS5600 CONF program failed: %s — sensor may stay in LPM",
                 esp_err_to_name(cerr));
    }

    /* CONF write moves the address pointer; reload ANGLE high so later
     * fetches are current-address reads. */
    err = read_raw12(o, &probe);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "AS5600 pointer re-arm failed: %s", esp_err_to_name(err));
        esp_foc_i2c_hal_bus_deinit(&o->bus);
        o->inited = false;
        return err;
    }

    ESP_LOGI(TAG,
             "AS5600 ready sda=%d scl=%d hz=%lu dt=%.6f pwm=%luHz pp=%u raw=%u "
             "conf=0x%04x pm=%u sf=%u",
             cfg->sda, cfg->scl, (unsigned long)hz, (double)cfg->dt_seconds,
             (unsigned long)cfg->pwm_hz, o->pole_pairs,
             (unsigned)probe,
             (unsigned)conf,
             (unsigned)(conf & 0x3u),
             (unsigned)((conf >> 8) & 0x3u));
    return ESP_OK;
}

esp_err_t esp_foc_rotor_as5600_deinit(esp_foc_rotor_sensor_t *s)
{
    if (s == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_foc_as5600_obj_t *o = obj_from(s);
    if (!o->inited) {
        return ESP_OK;
    }
    esp_foc_i2c_hal_bus_deinit(&o->bus);
    o->inited = false;
    return ESP_OK;
}

bool esp_foc_rotor_as5600_busy(const esp_foc_rotor_sensor_t *s)
{
    if (s == NULL) {
        return false;
    }
    return esp_foc_i2c_hal_busy(&obj_from_const(s)->bus);
}

void esp_foc_rotor_as5600_bus_recover(esp_foc_rotor_sensor_t *s)
{
    if (s == NULL) {
        return;
    }
    esp_foc_as5600_obj_t *o = obj_from(s);
    o->ptr_armed = false;
    esp_foc_i2c_hal_recover(&o->bus);
}

uint32_t esp_foc_rotor_as5600_xfer_fail_count(const esp_foc_rotor_sensor_t *s)
{
    if (s == NULL) {
        return 0;
    }
    return obj_from_const(s)->xfer_fail;
}

uint32_t esp_foc_rotor_as5600_xfer_ok_count(const esp_foc_rotor_sensor_t *s)
{
    if (s == NULL) {
        return 0;
    }
    return obj_from_const(s)->xfer_ok;
}

void esp_foc_rotor_as5600_set_invert(esp_foc_rotor_sensor_t *s, bool invert)
{
    if (s == NULL) {
        return;
    }
    obj_from(s)->invert = invert;
}

void esp_foc_rotor_as5600_set_pole_pairs(esp_foc_rotor_sensor_t *s,
                                         unsigned pole_pairs)
{
    if (s == NULL) {
        return;
    }
    obj_from(s)->pole_pairs = pole_pairs;
}
