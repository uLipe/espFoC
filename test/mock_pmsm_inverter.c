/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#include "mock_pmsm_inverter.h"

#include <math.h>
#include <string.h>

#include "espFoC/osal/esp_foc_osal.h"
#include "espFoC/utils/esp_foc_angle.h"

#define MP_TWO_PI 6.28318530718f

/* One ticker drives every started plant, so axes see the same timebase. */
#define MOCK_PMSM_MAX 2
static mock_pmsm_t *volatile s_list[MOCK_PMSM_MAX];
static volatile int s_n;
static volatile bool s_run;
static volatile bool s_alive;

static mock_pmsm_t *of(esp_foc_inverter_t *self)
{
    return (mock_pmsm_t *)self;
}

static float wrap_pi(float x)
{
    while (x > (float)M_PI) {
        x -= MP_TWO_PI;
    }
    while (x <= -(float)M_PI) {
        x += MP_TWO_PI;
    }
    return x;
}

static void mech_step(mock_pmsm_t *m, float te)
{
    const float dt = m->dt;
    if (m->locked) {
        m->w_m = 0.0f;
        return;
    }
    float w = m->w_m;
    if (fabsf(w) < 1.0e-3f) {
        if (fabsf(te) <= m->p.t_coul) {
            m->w_m = 0.0f;
            return;
        }
        w += (te - copysignf(m->p.t_coul, te)) / m->p.j * dt;
    } else {
        const float w_new = w + (te - m->p.b_visc * w - copysignf(m->p.t_coul, w)) / m->p.j * dt;
        w = (w_new * w < 0.0f) ? 0.0f : w_new;
    }
    m->w_m = w;
    m->theta_e = wrap_pi(m->theta_e + (float)m->p.pp * w * dt);
    m->theta_m = wrap_pi(m->theta_m + w * dt);
}

static void plant_step(mock_pmsm_t *m)
{
    const float s = sinf(m->theta_e);
    const float c = cosf(m->theta_e);
    const float we = (float)m->p.pp * m->w_m;

    if (m->enabled && !m->faulted) {
        const float vu = m->duty[0] * m->p.vdc;
        const float vv = m->duty[1] * m->p.vdc;
        const float vw = m->duty[2] * m->p.vdc;
        const float va = (2.0f * vu - vv - vw) / 3.0f;
        const float vb = (vv - vw) * 0.57735027f;
        const float ea = -m->p.psi * we * s;
        const float eb = m->p.psi * we * c;
        const float k = (1.0f - m->a_decay) / m->p.rs;
        m->i_a = m->a_decay * m->i_a + k * (va - ea);
        m->i_b = m->a_decay * m->i_b + k * (vb - eb);
    } else {
        m->i_a = 0.0f;
        m->i_b = 0.0f;
    }
    const float iq = -m->i_a * s + m->i_b * c;
    float te = 1.5f * (float)m->p.pp * m->p.psi * iq;
    if (m->p.t_cog != 0.0f) {
        te += m->p.t_cog * sinf((float)m->p.cog_n * m->theta_m);
    }
    mech_step(m, te);

    const float iu = m->i_a;
    const float iv = -0.5f * m->i_a + 0.8660254f * m->i_b;
    const float iw = -0.5f * m->i_a - 0.8660254f * m->i_b;
    m->i_log[0] = q16_from_float(iu);
    m->i_log[1] = q16_from_float(iv);
    m->i_log[2] = q16_from_float(iw);
}

static void tick(mock_pmsm_t *m)
{
    esp_foc_critical_enter();
    esp_foc_inverter_cb_t pwm = m->pwm_cb;
    void *pwm_arg = m->pwm_arg;
    esp_foc_critical_leave();
    if (pwm != NULL) {
        const uint64_t t0 = esp_foc_now_us();
        pwm(pwm_arg);
        const uint32_t us = (uint32_t)(esp_foc_now_us() - t0);
        m->cb_n++;
        m->cb_us_sum += us;
        if (us > m->cb_us_max) {
            m->cb_us_max = us;
        }
    }

    plant_step(m);
    m->ready = true;

    esp_foc_critical_enter();
    esp_foc_inverter_cb_t dma = m->dma_cb;
    void *dma_arg = m->dma_arg;
    esp_foc_critical_leave();
    if ((dma != NULL) && m->enabled) {
        dma(dma_arg);
    }
    m->ticks++;
}

static void ticker(void *arg)
{
    (void)arg;
    s_alive = true;
    while (s_run) {
        esp_foc_sleep_ms(1);
        for (int i = 0; i < s_n; i++) {
            mock_pmsm_t *m = s_list[i];
            /* The real bridge stops the timer when disabled, so no TEZ either. */
            if (!m->enabled) {
                continue;
            }
            const uint32_t per_ms = m->p.pwm_hz / 1000u;
            for (uint32_t k = 0; k < per_ms; k++) {
                tick(m);
            }
        }
    }
    s_alive = false;
    esp_foc_task_delete_self();
}

static void m_set_pwm_cb(esp_foc_inverter_t *self, esp_foc_inverter_cb_t cb, void *arg)
{
    of(self)->pwm_cb = cb;
    of(self)->pwm_arg = arg;
}

static void m_set_dma_cb(esp_foc_inverter_t *self, esp_foc_inverter_cb_t cb, void *arg)
{
    of(self)->dma_cb = cb;
    of(self)->dma_arg = arg;
}

static void m_set_fault_cb(esp_foc_inverter_t *self, esp_foc_fault_cb_t cb, void *arg)
{
    of(self)->fault_cb = cb;
    of(self)->fault_arg = arg;
}

static esp_err_t m_enable(esp_foc_inverter_t *self)
{
    mock_pmsm_t *m = of(self);
    if (m->faulted) {
        return ESP_ERR_INVALID_STATE;
    }
    m->wd_on = false;
    m->enabled = true;
    m->enable_count++;
    return ESP_OK;
}

static void m_disable(esp_foc_inverter_t *self)
{
    mock_pmsm_t *m = of(self);
    if (!m->enabled && !m->faulted) {
        m->idle_disables++;
    }
    m->enabled = false;
    m->disable_count++;
}

static void m_set_duties(esp_foc_inverter_t *self, q16_t du, q16_t dv, q16_t dw)
{
    mock_pmsm_t *m = of(self);
    if (m->faulted) {
        return;
    }
    m->duty[0] = q16_to_float(du);
    m->duty[1] = q16_to_float(dv);
    m->duty[2] = q16_to_float(dw);
}

static q16_t m_get_vdc(esp_foc_inverter_t *self)
{
    return q16_from_float(of(self)->p.vdc);
}

static uint32_t m_get_pwm_hz(esp_foc_inverter_t *self)
{
    return of(self)->p.pwm_hz;
}

static void m_fetch(esp_foc_inverter_t *self, q16_t *iu, q16_t *iv, q16_t *iw)
{
    mock_pmsm_t *m = of(self);
    *iu = m->i_log[0];
    *iv = m->i_log[1];
    *iw = m->i_log[2];
    m->ready = false;
}

static void m_fetch_raw(esp_foc_inverter_t *self, q16_t *iu, q16_t *iv, q16_t *iw)
{
    mock_pmsm_t *m = of(self);
    *iu = m->i_log[0];
    *iv = m->i_log[1];
    *iw = m->i_log[2];
}

static bool m_sample_ready(esp_foc_inverter_t *self)
{
    return of(self)->ready;
}

static void m_calibrate(esp_foc_inverter_t *self, int rounds)
{
    (void)rounds;
    of(self)->cal_count++;
}

static void m_set_wd(esp_foc_inverter_t *self, bool enable)
{
    of(self)->wd_on = enable;
}

static void m_soft_trip(esp_foc_inverter_t *self)
{
    mock_pmsm_trip(of(self), ESP_FOC_FAULT_SOFT_TRIP);
}

static esp_err_t m_clear_fault(esp_foc_inverter_t *self)
{
    mock_pmsm_t *m = of(self);
    if (!m->faulted) {
        return ESP_ERR_INVALID_STATE;
    }
    m->faulted = false;
    m->reason = ESP_FOC_FAULT_NONE;
    m->clear_count++;
    return ESP_OK;
}

static bool m_is_faulted(esp_foc_inverter_t *self)
{
    return of(self)->faulted;
}

static esp_foc_fault_reason_t m_reason(esp_foc_inverter_t *self)
{
    return of(self)->reason;
}

static esp_err_t m_set_map(esp_foc_inverter_t *self, const esp_foc_phase_map_t *map)
{
    mock_pmsm_t *m = of(self);
    if (m->enabled) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!esp_foc_phase_map_valid(map)) {
        return ESP_ERR_INVALID_ARG;
    }
    m->map = *map;
    return ESP_OK;
}

static void m_get_map(esp_foc_inverter_t *self, esp_foc_phase_map_t *map)
{
    *map = of(self)->map;
}

void mock_pmsm_init(mock_pmsm_t *m, const mock_pmsm_params_t *p)
{
    memset(m, 0, sizeof(*m));
    m->p = *p;
    m->dt = 1.0f / (float)p->pwm_hz;
    m->a_decay = expf(-p->rs * m->dt / p->ls);
    m->theta_e = p->theta0;
    m->theta_m = p->theta0 / (float)p->pp;
    m->duty[0] = 0.5f;
    m->duty[1] = 0.5f;
    m->duty[2] = 0.5f;
    esp_foc_phase_map_identity(&m->map);
    m->base.set_pwm_callback = m_set_pwm_cb;
    m->base.set_dma_callback = m_set_dma_cb;
    m->base.set_fault_callback = m_set_fault_cb;
    m->base.enable = m_enable;
    m->base.disable = m_disable;
    m->base.set_duties = m_set_duties;
    m->base.get_dc_link_voltage = m_get_vdc;
    m->base.get_pwm_rate_hz = m_get_pwm_hz;
    m->base.fetch_currents = m_fetch;
    m->base.fetch_currents_raw = m_fetch_raw;
    m->base.sample_ready = m_sample_ready;
    m->base.calibrate_currents = m_calibrate;
    m->base.set_sense_watchdog = m_set_wd;
    m->base.soft_trip = m_soft_trip;
    m->base.clear_fault = m_clear_fault;
    m->base.is_faulted = m_is_faulted;
    m->base.get_fault_reason = m_reason;
    m->base.set_phase_map = m_set_map;
    m->base.get_phase_map = m_get_map;
}

void mock_pmsm_start(mock_pmsm_t *m)
{
    if (s_n < MOCK_PMSM_MAX) {
        s_list[s_n] = m;
        s_n = s_n + 1;
    }
    if (s_alive) {
        return;
    }
    s_run = true;
    (void)esp_foc_task_spawn(ticker, NULL, "pmsm_tick", 4096, esp_foc_task_max_priority() - 1,
                             NULL);
    while (!s_alive) {
        esp_foc_sleep_ms(1);
    }
}

void mock_pmsm_stop(void)
{
    s_run = false;
    while (s_alive) {
        esp_foc_sleep_ms(1);
    }
    s_n = 0;
}

void mock_pmsm_trip(mock_pmsm_t *m, esp_foc_fault_reason_t reason)
{
    esp_foc_critical_enter();
    m->faulted = true;
    m->enabled = false;
    m->reason = reason;
    esp_foc_fault_cb_t cb = m->fault_cb;
    void *arg = m->fault_arg;
    esp_foc_critical_leave();
    if (cb != NULL) {
        cb(arg, reason);
    }
}

float mock_pmsm_fe_hz(const mock_pmsm_t *m)
{
    return (float)m->p.pp * m->w_m / MP_TWO_PI;
}

void mock_pmsm_timing_reset(mock_pmsm_t *m)
{
    esp_foc_critical_enter();
    m->cb_n = 0;
    m->cb_us_sum = 0;
    m->cb_us_max = 0;
    esp_foc_critical_leave();
}

static mock_pmsm_rotor_t *rot_of(const esp_foc_rotor_sensor_t *self)
{
    return (mock_pmsm_rotor_t *)self;
}

static esp_err_t r_fetch(esp_foc_rotor_sensor_t *self)
{
    mock_pmsm_rotor_t *r = rot_of(self);
    const mock_pmsm_t *m = r->plant;
    if (r->fail) {
        return ESP_FAIL;
    }
    if (r->freeze) {
        return ESP_OK;
    }
    esp_foc_critical_enter();
    const float th_e = m->theta_e;
    const float th_m = m->theta_m;
    const float w_m = m->w_m;
    esp_foc_critical_leave();
    r->theta_e = q16_from_float(wrap_pi(th_e + r->offset_e));
    r->theta_m = q16_from_float(th_m);
    r->omega_m = q16_from_float(w_m);
    const float ts = 1.0f / (float)m->p.pwm_hz;
    r->adv_m = q16_from_float(w_m * ts);
    r->adv_e = q16_from_float(w_m * (float)m->p.pp * ts);
    r->seq++;
    r->n_fetch++;
    return ESP_OK;
}

static q16_t r_get_position(esp_foc_rotor_sensor_t *self)
{
    return rot_of(self)->theta_m;
}

static q16_t r_get_velocity(esp_foc_rotor_sensor_t *self)
{
    return rot_of(self)->omega_m;
}

static q16_t r_get_elec(esp_foc_rotor_sensor_t *self)
{
    return rot_of(self)->theta_e;
}

static esp_err_t r_calibrate(esp_foc_rotor_sensor_t *self, int samples)
{
    (void)samples;
    return r_fetch(self);
}

static uint32_t r_caps(const esp_foc_rotor_sensor_t *self)
{
    (void)self;
    return ESP_FOC_ROTOR_CAP_MECH_ABS;
}

static void r_step(esp_foc_rotor_sensor_t *self)
{
    mock_pmsm_rotor_t *r = rot_of(self);
    if (r->extrapolate && (r->seq != 0u)) {
        r->theta_e = q16_wrap_pi(q16_add(r->theta_e, r->adv_e));
        r->theta_m = q16_wrap_pi(q16_add(r->theta_m, r->adv_m));
    }
}

static void r_snapshot(const esp_foc_rotor_sensor_t *self, esp_foc_rotor_state_t *out)
{
    const mock_pmsm_rotor_t *r = rot_of(self);
    memset(out, 0, sizeof(*out));
    out->theta_e = r->theta_e;
    out->theta_m = r->theta_m;
    out->omega_m = r->omega_m;
    out->omega_e = r->omega_m * r->plant->p.pp;
    out->seq = r->seq;
    out->sector = ESP_FOC_ROTOR_SECTOR_UNKNOWN;
    out->valid = (r->seq != 0u);
}

void mock_pmsm_rotor_init(mock_pmsm_rotor_t *r, mock_pmsm_t *m, float offset_e)
{
    memset(r, 0, sizeof(*r));
    r->plant = m;
    r->offset_e = offset_e;
    r->base.get_position = r_get_position;
    r->base.get_velocity = r_get_velocity;
    r->base.fetch = r_fetch;
    r->base.fetch_start = r_fetch;
    r->base.calibrate_offset = r_calibrate;
    r->base.caps = r_caps;
    r->base.step = r_step;
    r->base.snapshot = r_snapshot;
    r->base.get_electrical_position = r_get_elec;
}
