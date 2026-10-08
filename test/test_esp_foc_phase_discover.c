/*
 * Unit tests for esp_foc_phase_discover against a mock inverter (balanced R
 * star behind a hidden leg/phase wiring, two shunts with the third leg by
 * Kirchhoff in the hardware domain, same gather as the MCPWM driver) and a
 * mock rotor that relaxes toward the stator current vector.
 */
#include <math.h>
#include <string.h>

#include "unity.h"
#include "esp_err.h"
#include "espFoC/motor_control/esp_foc_phase_discover.h"
#include "espFoC/osal/esp_foc_osal.h"
#include "espFoC/utils/esp_foc_angle.h"
#include "espFoC/utils/esp_foc_clarke.h"
#include "espFoC/utils/esp_foc_park.h"
#include "espFoC/utils/esp_foc_svm.h"

#define TWO_PI_F      6.28318530718f
#define MOCK_PWM_HZ   1000u
#define MOCK_R_OHM    2.5f
#define MOCK_GAIN     60.0f   /* d theta_r / dt per A of misalignment [rad/s/A] */
#define MOCK_PP       7

typedef struct {
    esp_foc_inverter_t base;

    uint8_t phase_of_leg[3];
    int8_t shunt_sign;
    uint8_t sense_leg[2];
    float vdc;
    bool open;
    int trip_at_enable;

    esp_foc_inverter_cb_t pwm_cb;
    void *pwm_arg;
    esp_foc_inverter_cb_t dma_cb;
    void *dma_arg;
    esp_foc_fault_cb_t fault_cb;
    void *fault_arg;

    bool enabled;
    bool faulted;
    esp_foc_phase_map_t map;
    float duty_hw[3];
    q16_t i_log[3];
    volatile bool ready;
    int enable_count;
    int idle_disables;
    int trips;
    volatile uint32_t duty_writes;

    float theta_r;
    float dither_rad;
    uint32_t ticks;
} mock_inv_t;

typedef struct {
    esp_foc_rotor_sensor_t base;
    mock_inv_t *plant;
    int dir;
    float mount_rad;
    float offset_rad;
    q16_t latch;
    uint32_t caps;
    int n_fetch;
    int n_zero;
    volatile uint32_t n_step;
} mock_rotor_t;

typedef struct {
    int count[ESP_FOC_PHASE_DISCOVER_EV_DIR + 1];
    int tripped_candidates;
    esp_foc_phase_discover_t *pd;
    bool try_reenter;
    esp_err_t reenter_err;
    int app_dma_at_entry;
    esp_foc_phase_discover_event_t dir;
} ev_log_t;

static volatile bool s_tick_run;
static volatile bool s_tick_alive;
static volatile int s_app_dma_calls;

static void app_dma(void *arg)
{
    (void)arg;
    s_app_dma_calls++;
}

static float wrap_pi_f(float x)
{
    while (x > (float)M_PI) {
        x -= TWO_PI_F;
    }
    while (x <= -(float)M_PI) {
        x += TWO_PI_F;
    }
    return x;
}

static mock_inv_t *inv_of(esp_foc_inverter_t *self)
{
    return (mock_inv_t *)self;
}

/* Logical duties -> hidden wiring -> R star -> shunts -> driver gather. */
static void plant_eval(const mock_inv_t *m, const float duty_hw[3], float i_motor[3],
                       q16_t i_log[3])
{
    float v[3];
    float i_leg[3];
    for (int L = 0; L < 3; L++) {
        v[m->phase_of_leg[L]] = duty_hw[L] * m->vdc;
    }
    const float vn = (v[0] + v[1] + v[2]) / 3.0f;
    for (int k = 0; k < 3; k++) {
        i_motor[k] = m->open ? 0.0f : (v[k] - vn) / MOCK_R_OHM;
    }
    for (int L = 0; L < 3; L++) {
        i_leg[L] = i_motor[m->phase_of_leg[L]];
    }
    float hw[3];
    hw[0] = (float)m->shunt_sign * i_leg[m->sense_leg[0]];
    hw[1] = (float)m->shunt_sign * i_leg[m->sense_leg[1]];
    hw[2] = -(hw[0] + hw[1]);
    for (int L = 0; L < 3; L++) {
        float x = hw[m->map.pwm_to_hw[L]];
        if (m->map.i_sign[L] < 0) {
            x = -x;
        }
        i_log[L] = q16_from_float(x);
    }
}

static void mock_tick(mock_inv_t *m)
{
    esp_foc_critical_enter();
    esp_foc_inverter_cb_t pwm = m->pwm_cb;
    void *pwm_arg = m->pwm_arg;
    esp_foc_critical_leave();
    if (pwm != NULL) {
        pwm(pwm_arg);
    }

    float i_motor[3] = {0.0f, 0.0f, 0.0f};
    q16_t i_log[3] = {0, 0, 0};
    const bool driving = m->enabled && !m->faulted;
    if (driving) {
        plant_eval(m, m->duty_hw, i_motor, i_log);
    }
    if (driving && m->trip_at_enable > 0 && m->enable_count == m->trip_at_enable &&
        (fabsf(m->duty_hw[0] - 0.5f) > 0.01f || fabsf(m->duty_hw[1] - 0.5f) > 0.01f)) {
        m->faulted = true;
        m->enabled = false;
        m->trips++;
        if (m->fault_cb != NULL) {
            m->fault_cb(m->fault_arg, ESP_FOC_FAULT_ILIMIT);
        }
        i_log[0] = i_log[1] = i_log[2] = 0;
        i_motor[0] = i_motor[1] = i_motor[2] = 0.0f;
    }
    m->i_log[0] = i_log[0];
    m->i_log[1] = i_log[1];
    m->i_log[2] = i_log[2];
    m->ready = true;

    esp_foc_critical_enter();
    esp_foc_inverter_cb_t dma = m->dma_cb;
    void *dma_arg = m->dma_arg;
    esp_foc_critical_leave();
    if (dma != NULL) {
        dma(dma_arg);
    }

    const float a = (2.0f * i_motor[0] - i_motor[1] - i_motor[2]) / 3.0f;
    const float b = (i_motor[1] - i_motor[2]) * 0.57735027f;
    const float mag = sqrtf(a * a + b * b);
    if (mag > 1.0e-4f) {
        const float phi = atan2f(b, a);
        m->theta_r = wrap_pi_f(m->theta_r +
                               MOCK_GAIN * mag * sinf(phi - m->theta_r) / (float)MOCK_PWM_HZ);
    }
    m->ticks++;
}

static void ticker(void *arg)
{
    mock_inv_t *m = (mock_inv_t *)arg;
    s_tick_alive = true;
    while (s_tick_run) {
        esp_foc_sleep_ms(1);
        mock_tick(m);
    }
    s_tick_alive = false;
    esp_foc_task_delete_self();
}

static void ticker_start(mock_inv_t *m)
{
    s_tick_run = true;
    TEST_ASSERT_EQUAL(0, esp_foc_task_spawn(ticker, m, "pd_tick", 4096,
                                            esp_foc_task_max_priority() - 1, NULL));
    while (!s_tick_alive) {
        esp_foc_sleep_ms(1);
    }
}

static void ticker_stop(void)
{
    s_tick_run = false;
    while (s_tick_alive) {
        esp_foc_sleep_ms(1);
    }
}

static void m_set_pwm_cb(esp_foc_inverter_t *self, esp_foc_inverter_cb_t cb, void *arg)
{
    inv_of(self)->pwm_cb = cb;
    inv_of(self)->pwm_arg = arg;
}

static void m_set_dma_cb(esp_foc_inverter_t *self, esp_foc_inverter_cb_t cb, void *arg)
{
    inv_of(self)->dma_cb = cb;
    inv_of(self)->dma_arg = arg;
}

static void m_set_fault_cb(esp_foc_inverter_t *self, esp_foc_fault_cb_t cb, void *arg)
{
    inv_of(self)->fault_cb = cb;
    inv_of(self)->fault_arg = arg;
}

static esp_err_t m_enable(esp_foc_inverter_t *self)
{
    mock_inv_t *m = inv_of(self);
    if (m->faulted) {
        return ESP_ERR_INVALID_STATE;
    }
    m->enabled = true;
    m->enable_count++;
    return ESP_OK;
}

static void m_disable(esp_foc_inverter_t *self)
{
    mock_inv_t *m = inv_of(self);
    if (!m->enabled && !m->faulted) {
        m->idle_disables++;
    }
    m->enabled = false;
}

static void m_set_duties(esp_foc_inverter_t *self, q16_t du, q16_t dv, q16_t dw)
{
    mock_inv_t *m = inv_of(self);
    if (m->faulted) {
        return;
    }
    const q16_t logical[3] = {du, dv, dw};
    for (int L = 0; L < 3; L++) {
        m->duty_hw[m->map.pwm_to_hw[L]] = q16_to_float(logical[L]);
    }
    m->duty_writes++;
}

static q16_t m_get_vdc(esp_foc_inverter_t *self)
{
    return q16_from_float(inv_of(self)->vdc);
}

static uint32_t m_get_pwm_hz(esp_foc_inverter_t *self)
{
    (void)self;
    return MOCK_PWM_HZ;
}

static void m_fetch(esp_foc_inverter_t *self, q16_t *iu, q16_t *iv, q16_t *iw)
{
    mock_inv_t *m = inv_of(self);
    *iu = m->i_log[0];
    *iv = m->i_log[1];
    *iw = m->i_log[2];
    m->ready = false;
}

static void m_fetch_raw(esp_foc_inverter_t *self, q16_t *iu, q16_t *iv, q16_t *iw)
{
    mock_inv_t *m = inv_of(self);
    *iu = m->i_log[0];
    *iv = m->i_log[1];
    *iw = m->i_log[2];
}

static bool m_sample_ready(esp_foc_inverter_t *self)
{
    return inv_of(self)->ready;
}

static void m_calibrate(esp_foc_inverter_t *self, int rounds)
{
    (void)self;
    (void)rounds;
}

static void m_set_wd(esp_foc_inverter_t *self, bool enable)
{
    (void)self;
    (void)enable;
}

static void m_soft_trip(esp_foc_inverter_t *self)
{
    inv_of(self)->faulted = true;
    inv_of(self)->enabled = false;
}

static esp_err_t m_clear_fault(esp_foc_inverter_t *self)
{
    mock_inv_t *m = inv_of(self);
    if (!m->faulted) {
        return ESP_ERR_INVALID_STATE;
    }
    m->faulted = false;
    return ESP_OK;
}

static bool m_is_faulted(esp_foc_inverter_t *self)
{
    return inv_of(self)->faulted;
}

static esp_foc_fault_reason_t m_reason(esp_foc_inverter_t *self)
{
    return inv_of(self)->faulted ? ESP_FOC_FAULT_ILIMIT : ESP_FOC_FAULT_NONE;
}

static esp_err_t m_set_map(esp_foc_inverter_t *self, const esp_foc_phase_map_t *map)
{
    mock_inv_t *m = inv_of(self);
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
    *map = inv_of(self)->map;
}

static void mock_inv_init(mock_inv_t *m, const uint8_t phase_of_leg[3], int8_t sign, float vdc)
{
    memset(m, 0, sizeof(*m));
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
    for (int L = 0; L < 3; L++) {
        m->phase_of_leg[L] = phase_of_leg[L];
        m->duty_hw[L] = 0.5f;
    }
    m->shunt_sign = sign;
    m->sense_leg[0] = 0;
    m->sense_leg[1] = 1;
    m->vdc = vdc;
    m->theta_r = 1.3f;
    esp_foc_phase_map_identity(&m->map);
    /* Stands for the app's handler: the block must take it out for the run. */
    m->dma_cb = app_dma;
    s_app_dma_calls = 0;
}

static float rotor_raw(const mock_rotor_t *r)
{
    const mock_inv_t *p = r->plant;
    const float dither = p->dither_rad * sinf(TWO_PI_F * 4.0f * (float)p->ticks / (float)MOCK_PWM_HZ);
    return wrap_pi_f((float)r->dir * (p->theta_r + dither) / (float)MOCK_PP + r->mount_rad);
}

static mock_rotor_t *rot_of(esp_foc_rotor_sensor_t *self)
{
    return (mock_rotor_t *)self;
}

static q16_t r_get_pos(esp_foc_rotor_sensor_t *self)
{
    return rot_of(self)->latch;
}

static q16_t r_zero_q16(esp_foc_rotor_sensor_t *self)
{
    (void)self;
    return 0;
}

static esp_err_t r_fetch(esp_foc_rotor_sensor_t *self)
{
    mock_rotor_t *r = rot_of(self);
    r->latch = q16_from_float(wrap_pi_f(rotor_raw(r) - r->offset_rad));
    r->n_fetch++;
    return ESP_OK;
}

static esp_err_t r_fetch_start(esp_foc_rotor_sensor_t *self)
{
    (void)self;
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t r_zero(esp_foc_rotor_sensor_t *self, int samples)
{
    mock_rotor_t *r = rot_of(self);
    if (samples < 1) {
        return ESP_ERR_INVALID_ARG;
    }
    r->offset_rad = rotor_raw(r);
    r->latch = 0;
    r->n_zero++;
    return ESP_OK;
}

static uint32_t r_caps(const esp_foc_rotor_sensor_t *self)
{
    return ((const mock_rotor_t *)self)->caps;
}

static void r_step(esp_foc_rotor_sensor_t *self)
{
    rot_of(self)->n_step++;
}

static void r_snapshot(const esp_foc_rotor_sensor_t *self, esp_foc_rotor_state_t *out)
{
    memset(out, 0, sizeof(*out));
    out->theta_m = ((const mock_rotor_t *)self)->latch;
    out->sector = ESP_FOC_ROTOR_SECTOR_UNKNOWN;
    out->valid = true;
}

static void mock_rotor_init(mock_rotor_t *r, mock_inv_t *plant, int dir)
{
    memset(r, 0, sizeof(*r));
    r->base.get_position = r_get_pos;
    r->base.get_velocity = r_zero_q16;
    r->base.fetch = r_fetch;
    r->base.fetch_start = r_fetch_start;
    r->base.calibrate_offset = r_zero;
    r->base.caps = r_caps;
    r->base.step = r_step;
    r->base.snapshot = r_snapshot;
    r->base.get_electrical_position = r_zero_q16;
    r->plant = plant;
    r->dir = dir;
    r->mount_rad = 2.2f;
    r->caps = ESP_FOC_ROTOR_CAP_MECH_ABS;
}

static void on_event(void *ctx, const esp_foc_phase_discover_event_t *e)
{
    ev_log_t *log = (ev_log_t *)ctx;
    log->count[e->ev]++;
    if (e->ev == ESP_FOC_PHASE_DISCOVER_EV_CANDIDATE && e->tripped) {
        log->tripped_candidates++;
    }
    if (e->ev == ESP_FOC_PHASE_DISCOVER_EV_DIR) {
        log->dir = *e;
    }
    if (log->try_reenter) {
        esp_foc_phase_discover_result_t r;
        log->try_reenter = false;
        log->app_dma_at_entry = s_app_dma_calls;
        log->reenter_err = esp_foc_phase_discover_run(log->pd, &r);
    }
}

static void cfg_test(esp_foc_phase_discover_config_t *cfg, ev_log_t *log)
{
    esp_foc_phase_discover_default_config(cfg);
    cfg->nudge_ms = 20;
    cfg->settle_ms = 20;
    cfg->sensor.sweep_ms = 300;
    cfg->sensor.calm_ms = 100;
    cfg->sensor.timeout_ms = 1500;
    cfg->on_event = on_event;
    cfg->ctx = log;
}

static void assert_released(const mock_inv_t *m)
{
    TEST_ASSERT_FALSE(m->enabled);
    TEST_ASSERT_EQUAL(0, m->idle_disables);
    TEST_ASSERT_NULL(m->pwm_cb);
    TEST_ASSERT_NULL(m->pwm_arg);
    TEST_ASSERT_NULL(m->dma_cb);
    TEST_ASSERT_NULL(m->fault_cb);
}

/* +Vd must come back on +d and +Vq on +q through the map the block chose. */
static void assert_consistent_frame(mock_inv_t *m, const esp_foc_phase_map_t *map)
{
    const q16_t v = q16_from_float(0.1f);
    const float expect = 0.1f * m->vdc / MOCK_R_OHM;
    const esp_foc_phase_map_t saved = m->map;
    m->map = *map;
    for (int axis = 0; axis < 2; axis++) {
        q16_t a;
        q16_t b;
        q16_t du;
        q16_t dv;
        q16_t dw;
        esp_foc_inv_park(0, Q16_ONE, axis == 0 ? v : 0, axis == 1 ? v : 0, &a, &b);
        esp_foc_svm(a, b, &du, &dv, &dw);
        const q16_t logical[3] = {du, dv, dw};
        float duty_hw[3];
        for (int L = 0; L < 3; L++) {
            duty_hw[map->pwm_to_hw[L]] = q16_to_float(logical[L]);
        }
        float i_motor[3];
        q16_t i_log[3];
        plant_eval(m, duty_hw, i_motor, i_log);
        q16_t ia;
        q16_t ib;
        q16_t id;
        q16_t iq;
        esp_foc_clarke(i_log[0], i_log[1], i_log[2], &ia, &ib);
        esp_foc_park(0, Q16_ONE, ia, ib, &id, &iq);
        const float on = q16_to_float(axis == 0 ? id : iq);
        const float off = q16_to_float(axis == 0 ? iq : id);
        TEST_ASSERT_FLOAT_WITHIN(0.1f * expect, expect, on);
        TEST_ASSERT_FLOAT_WITHIN(0.05f * expect, 0.0f, off);
    }
    m->map = saved;
}

/* +1 when the chosen frame is a rotation of the motor's, -1 when mirrored. */
static int frame_handedness(const mock_inv_t *m, const esp_foc_phase_map_t *map)
{
    uint8_t s[3];
    for (int L = 0; L < 3; L++) {
        s[L] = m->phase_of_leg[map->pwm_to_hw[L]];
    }
    int inv = 0;
    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 3; j++) {
            if (s[i] > s[j]) {
                inv++;
            }
        }
    }
    return (inv & 1) ? -1 : 1;
}

static const uint8_t k_wirings[6][3] = {
    {0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0},
};

TEST_CASE("phase_discover rejects bad arguments", "[espFoC][phase_discover]")
{
    mock_inv_t m;
    mock_rotor_t r;
    esp_foc_phase_discover_t pd;
    esp_foc_phase_discover_result_t out;
    esp_foc_phase_discover_config_t cfg;
    ev_log_t log;
    memset(&log, 0, sizeof(log));
    mock_inv_init(&m, k_wirings[0], 1, 12.0f);
    mock_rotor_init(&r, &m, 1);
    cfg_test(&cfg, &log);

    memset(&pd, 0, sizeof(pd));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_phase_discover_init(NULL, &m.base, NULL, &cfg));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_phase_discover_init(&pd, NULL, NULL, &cfg));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_phase_discover_run(NULL, &out));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_phase_discover_run(&pd, &out));

    cfg.tries = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_phase_discover_init(&pd, &m.base, NULL, &cfg));
    cfg_test(&cfg, &log);
    cfg.pulse_ms = 2;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_phase_discover_init(&pd, &m.base, NULL, &cfg));
    cfg_test(&cfg, &log);
    cfg.sensor.dir_ms = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_phase_discover_init(&pd, &m.base, &r.base, &cfg));
    cfg_test(&cfg, &log);

    r.caps = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED,
                      esp_foc_phase_discover_init(&pd, &m.base, &r.base, &cfg));
    r.caps = ESP_FOC_ROTOR_CAP_MECH_ABS;

    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_phase_discover_init(&pd, &m.base, NULL, NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_phase_discover_run(&pd, NULL));
    /* Untouched inverter: init binds, it does not drive. */
    TEST_ASSERT_EQUAL(0, m.enable_count);
    TEST_ASSERT_EQUAL(0, m.duty_writes);
}

TEST_CASE("phase_discover clamps the probe to 5..25 % of Vdc", "[espFoC][phase_discover]")
{
    static const float vdc[5] = {3.0f, 6.0f, 12.0f, 24.0f, 48.0f};
    static const float pu[5] = {0.25f, 0.20f, 0.10f, 0.05f, 0.05f};
    mock_inv_t m;
    esp_foc_phase_discover_t pd;
    esp_foc_phase_discover_config_t cfg;
    esp_foc_phase_discover_default_config(&cfg);
    cfg.vd_v = 1.2f;
    for (int i = 0; i < 5; i++) {
        mock_inv_init(&m, k_wirings[0], 1, vdc[i]);
        memset(&pd, 0, sizeof(pd));
        TEST_ASSERT_EQUAL(ESP_OK, esp_foc_phase_discover_init(&pd, &m.base, NULL, &cfg));
        TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, pu[i], q16_to_float(pd.vd_pu));
    }
    mock_inv_init(&m, k_wirings[0], 1, 0.0f);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_phase_discover_init(&pd, &m.base, NULL, &cfg));
}

TEST_CASE("phase_discover finds a consistent frame for every hidden wiring",
          "[espFoC][phase_discover]")
{
    mock_inv_t m;
    esp_foc_phase_discover_t pd;
    esp_foc_phase_discover_result_t out;
    esp_foc_phase_discover_config_t cfg;
    ev_log_t log;

    for (int w = 0; w < 12; w++) {
        const int8_t sign = (w & 1) ? -1 : 1;
        memset(&log, 0, sizeof(log));
        mock_inv_init(&m, k_wirings[w / 2], sign, 12.0f);
        cfg_test(&cfg, &log);
        memset(&pd, 0, sizeof(pd));
        TEST_ASSERT_EQUAL(ESP_OK, esp_foc_phase_discover_init(&pd, &m.base, NULL, &cfg));
        ticker_start(&m);
        esp_err_t err = esp_foc_phase_discover_run(&pd, &out);
        ticker_stop();

        TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err, "wiring");
        TEST_ASSERT_TRUE(esp_foc_phase_map_valid(&out.map));
        TEST_ASSERT_EQUAL_MEMORY(&out.map, &m.map, sizeof(out.map));
        for (int L = 0; L < 3; L++) {
            TEST_ASSERT_EQUAL(sign, out.map.i_sign[L]);
        }
        assert_consistent_frame(&m, &out.map);
        TEST_ASSERT_EQUAL(1, out.attempts);
        TEST_ASSERT_TRUE(out.rank_idx >= 0 && out.rank_idx < 12);
        TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.1f * 12.0f / MOCK_R_OHM, q16_to_float(out.id_verify));
        TEST_ASSERT_FLOAT_WITHIN(0.5f, 12.0f / MOCK_R_OHM, q16_to_float(out.admittance));
        TEST_ASSERT_FALSE(out.sensor_zeroed);
        TEST_ASSERT_EQUAL(12, log.count[ESP_FOC_PHASE_DISCOVER_EV_CANDIDATE]);
        TEST_ASSERT_EQUAL(2, log.count[ESP_FOC_PHASE_DISCOVER_EV_HANDED]);
        TEST_ASSERT_EQUAL(1, log.count[ESP_FOC_PHASE_DISCOVER_EV_MAP]);
        TEST_ASSERT_EQUAL(0, log.count[ESP_FOC_PHASE_DISCOVER_EV_NUDGE]);
        assert_released(&m);
    }
}

TEST_CASE("phase_discover refuses a mirrored sense", "[espFoC][phase_discover]")
{
    mock_inv_t m;
    esp_foc_phase_discover_t pd;
    esp_foc_phase_discover_result_t out;
    esp_foc_phase_discover_config_t cfg;
    ev_log_t log;
    memset(&log, 0, sizeof(log));
    mock_inv_init(&m, k_wirings[3], 1, 12.0f);
    m.sense_leg[1] = 2;
    cfg_test(&cfg, &log);
    cfg.tries = 2;
    memset(&pd, 0, sizeof(pd));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_phase_discover_init(&pd, &m.base, NULL, &cfg));
    ticker_start(&m);
    esp_err_t err = esp_foc_phase_discover_run(&pd, &out);
    ticker_stop();

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_RESPONSE, err);
    TEST_ASSERT_EQUAL(2, out.attempts);
    TEST_ASSERT_EQUAL(1, log.count[ESP_FOC_PHASE_DISCOVER_EV_NUDGE]);
    TEST_ASSERT_EQUAL(2, log.count[ESP_FOC_PHASE_DISCOVER_EV_REFUSED]);
    TEST_ASSERT_EQUAL(0, log.count[ESP_FOC_PHASE_DISCOVER_EV_MAP]);
    assert_released(&m);
}

TEST_CASE("phase_discover fails an open winding after tries-1 nudges",
          "[espFoC][phase_discover]")
{
    mock_inv_t m;
    esp_foc_phase_discover_t pd;
    esp_foc_phase_discover_result_t out;
    esp_foc_phase_discover_config_t cfg;
    ev_log_t log;
    memset(&log, 0, sizeof(log));
    mock_inv_init(&m, k_wirings[0], 1, 12.0f);
    m.open = true;
    cfg_test(&cfg, &log);
    cfg.tries = 3;
    memset(&pd, 0, sizeof(pd));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_phase_discover_init(&pd, &m.base, NULL, &cfg));
    ticker_start(&m);
    esp_err_t err = esp_foc_phase_discover_run(&pd, &out);
    ticker_stop();

    TEST_ASSERT_EQUAL(ESP_FAIL, err);
    TEST_ASSERT_EQUAL(3, out.attempts);
    TEST_ASSERT_EQUAL(2, log.count[ESP_FOC_PHASE_DISCOVER_EV_NUDGE]);
    TEST_ASSERT_EQUAL(36, log.count[ESP_FOC_PHASE_DISCOVER_EV_CANDIDATE]);
    TEST_ASSERT_EQUAL(0, log.count[ESP_FOC_PHASE_DISCOVER_EV_RANKED]);
    assert_released(&m);
}

TEST_CASE("phase_discover clears a mid-sweep trip and carries on", "[espFoC][phase_discover]")
{
    mock_inv_t m;
    esp_foc_phase_discover_t pd;
    esp_foc_phase_discover_result_t out;
    esp_foc_phase_discover_config_t cfg;
    ev_log_t log;
    memset(&log, 0, sizeof(log));
    mock_inv_init(&m, k_wirings[4], -1, 12.0f);
    m.trip_at_enable = 3;
    cfg_test(&cfg, &log);
    memset(&pd, 0, sizeof(pd));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_phase_discover_init(&pd, &m.base, NULL, &cfg));
    ticker_start(&m);
    esp_err_t err = esp_foc_phase_discover_run(&pd, &out);
    ticker_stop();

    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL(1, m.trips);
    TEST_ASSERT_EQUAL(1, log.tripped_candidates);
    TEST_ASSERT_EQUAL(1, out.attempts);
    TEST_ASSERT_FALSE(m.faulted);
    assert_consistent_frame(&m, &out.map);
    assert_released(&m);
}

static void run_sensored(int dir, int wiring)
{
    mock_inv_t m;
    mock_rotor_t r;
    esp_foc_phase_discover_t pd;
    esp_foc_phase_discover_result_t out;
    esp_foc_phase_discover_config_t cfg;
    ev_log_t log;
    memset(&log, 0, sizeof(log));
    mock_inv_init(&m, k_wirings[wiring], 1, 12.0f);
    mock_rotor_init(&r, &m, dir);
    cfg_test(&cfg, &log);
    memset(&pd, 0, sizeof(pd));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_phase_discover_init(&pd, &m.base, &r.base, &cfg));
    ticker_start(&m);
    esp_err_t err = esp_foc_phase_discover_run(&pd, &out);
    ticker_stop();

    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_TRUE(out.sensor_zeroed);
    TEST_ASSERT_EQUAL(1, r.n_zero);
    TEST_ASSERT_TRUE(r.n_step > 0u);
    const bool expect_rev = (frame_handedness(&m, &out.map) * dir) < 0;
    TEST_ASSERT_EQUAL(expect_rev, out.sensor_reversed);
    TEST_ASSERT_EQUAL(expect_rev, log.dir.reversed);
    TEST_ASSERT_TRUE(fabsf(q16_to_float(log.dir.value)) >= cfg.sensor.dir_min_rad);
    TEST_ASSERT_EQUAL(1, log.count[ESP_FOC_PHASE_DISCOVER_EV_WELL]);
    TEST_ASSERT_EQUAL(1, log.count[ESP_FOC_PHASE_DISCOVER_EV_STILL]);
    TEST_ASSERT_EQUAL(1, log.count[ESP_FOC_PHASE_DISCOVER_EV_ZERO]);
    assert_released(&m);
}

TEST_CASE("phase_discover zeroes the sensor and reports its direction",
          "[espFoC][phase_discover]")
{
    run_sensored(1, 0);
    run_sensored(-1, 0);
    run_sensored(1, 1);
    run_sensored(-1, 5);
}

TEST_CASE("phase_discover times out on a rotor that never rests", "[espFoC][phase_discover]")
{
    mock_inv_t m;
    mock_rotor_t r;
    esp_foc_phase_discover_t pd;
    esp_foc_phase_discover_result_t out;
    esp_foc_phase_discover_config_t cfg;
    ev_log_t log;
    memset(&log, 0, sizeof(log));
    mock_inv_init(&m, k_wirings[2], 1, 12.0f);
    m.dither_rad = 1.0f;
    mock_rotor_init(&r, &m, 1);
    cfg_test(&cfg, &log);
    cfg.sensor.timeout_ms = 400;
    memset(&pd, 0, sizeof(pd));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_phase_discover_init(&pd, &m.base, &r.base, &cfg));
    ticker_start(&m);
    esp_err_t err = esp_foc_phase_discover_run(&pd, &out);
    ticker_stop();

    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, err);
    TEST_ASSERT_FALSE(out.sensor_zeroed);
    TEST_ASSERT_EQUAL(0, r.n_zero);
    TEST_ASSERT_EQUAL(1, log.count[ESP_FOC_PHASE_DISCOVER_EV_STILL]);
    assert_released(&m);
}

TEST_CASE("phase_discover refuses re-entry and goes inert after cleanup",
          "[espFoC][phase_discover]")
{
    mock_inv_t m;
    esp_foc_phase_discover_t pd;
    esp_foc_phase_discover_result_t out;
    esp_foc_phase_discover_config_t cfg;
    ev_log_t log;
    memset(&log, 0, sizeof(log));
    mock_inv_init(&m, k_wirings[1], 1, 12.0f);
    cfg_test(&cfg, &log);
    memset(&pd, 0, sizeof(pd));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_phase_discover_init(&pd, &m.base, NULL, &cfg));

    esp_foc_phase_discover_cleanup(NULL);
    esp_foc_phase_discover_cleanup(&pd);
    TEST_ASSERT_FALSE(m.enabled);
    TEST_ASSERT_EQUAL(0, m.idle_disables);
    TEST_ASSERT_EQUAL(0u, m.duty_writes);
    TEST_ASSERT_EQUAL_PTR(app_dma, m.dma_cb);

    log.pd = &pd;
    log.try_reenter = true;
    ticker_start(&m);
    esp_err_t err = esp_foc_phase_discover_run(&pd, &out);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, log.reenter_err);
    TEST_ASSERT_EQUAL(log.app_dma_at_entry, s_app_dma_calls);
    assert_released(&m);

    const uint32_t writes = m.duty_writes;
    const uint32_t ticks = m.ticks;
    esp_foc_sleep_ms(30);
    TEST_ASSERT_TRUE(m.ticks > ticks);
    TEST_ASSERT_EQUAL(writes, m.duty_writes);

    /* The caller owns the inverter again: cleanup must leave its callbacks
     * and its idle bridge alone. */
    m.dma_cb = app_dma;
    esp_foc_phase_discover_cleanup(&pd);
    esp_foc_phase_discover_cleanup(&pd);
    ticker_stop();
    TEST_ASSERT_EQUAL_PTR(app_dma, m.dma_cb);
    m.dma_cb = NULL;
    assert_released(&m);
    TEST_ASSERT_EQUAL(writes, m.duty_writes);
}
