/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#include <string.h>

#include "esp_check.h"
#include "esp_idf_version.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_macros.h"
#include "esp_rom_gpio.h"
#include "hal/gpio_ll.h"
#include "hal/pcnt_ll.h"
#include "sdkconfig.h"
#include "soc/gpio_pins.h"
#include "soc/soc_caps.h"

#if __has_include("hal/pcnt_periph.h")
#include "hal/pcnt_periph.h"
#else
#include "soc/pcnt_periph.h"
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#include "esp_clk_tree.h"
#else
#include "esp_private/esp_clk.h"
#endif

#include "espFoC/drivers/esp_foc_rotor_quadrature.h"
#include "espFoC/osal/esp_foc_osal.h"
#include "espFoC/utils/esp_foc_angle.h"
#include "esp_foc_drv_intr.h"
#include "esp_foc_hal_compat.h"

#if !SOC_PCNT_SUPPORTED
#error "espFoC quadrature rotor sensor requires a pulse counter (SOC_PCNT_SUPPORTED)"
#endif

static const char *TAG = "foc_quad";

#define QUAD_EV_LOW_LIMIT  (1u << PCNT_LL_WATCH_EVENT_LOW_LIMIT)
#define QUAD_EV_HIGH_LIMIT (1u << PCNT_LL_WATCH_EVENT_HIGH_LIMIT)

typedef struct {
    esp_foc_rotor_sensor_t iface;
    pcnt_dev_t *dev;
    uint32_t unit;
    uint32_t unit_bit;
    int32_t cpr;
    /* round(2^32 / cpr): count → fraction of a turn, wrapping in uint32 */
    int64_t k_turn;
    int64_t k_elec;
    q16_t k_vel;
    bool invert;
    unsigned pole_pairs;
    /* Raw in-turn count of the last read. One word, so θ_m and θ_e taken from
     * it in snapshot() always belong to the same sample. */
    volatile int32_t lat_cnt;
    volatile int32_t turns;
    /* Odd while the limit ISR updates turns; readers retry on change. */
    volatile uint32_t wseq;
    int64_t prev_total;
    bool have_prev;
    q16_t vel_q16;
    uint32_t seq;
    uint32_t age_periods;
    int group;
    int gpio_a;
    int gpio_b;
    intr_handle_t intr;
    bool acquired;
    bool inited;
} esp_foc_quad_obj_t;

static esp_foc_quad_obj_t s_pool[CONFIG_ESP_FOC_MAX_ROTOR_SENSORS];
static uint8_t s_group_users[ESP_FOC_PCNT_GROUPS];

static esp_foc_quad_obj_t *obj_from(esp_foc_rotor_sensor_t *self)
{
    return __containerof(self, esp_foc_quad_obj_t, iface);
}

static const esp_foc_quad_obj_t *obj_from_const(const esp_foc_rotor_sensor_t *self)
{
    return __containerof(self, esp_foc_quad_obj_t, iface);
}

static inline q16_t turn_frac_to_rad(uint32_t f)
{
    return q16_wrap_pi((q16_t)(((int64_t)(int32_t)f * (int64_t)Q16_TWO_PI) >> 32));
}

static inline int32_t signed_cnt(const esp_foc_quad_obj_t *o)
{
    int32_t c = o->lat_cnt;
    return o->invert ? -c : c;
}

static inline q16_t mech_from_cnt(const esp_foc_quad_obj_t *o, int32_t c)
{
    return turn_frac_to_rad((uint32_t)((int64_t)c * o->k_turn));
}

static inline q16_t elec_from_cnt(const esp_foc_quad_obj_t *o, int32_t c)
{
    if (o->pole_pairs == 0u) {
        return 0;
    }
    return turn_frac_to_rad((uint32_t)((int64_t)c * o->k_elec));
}

static void quad_limit_isr(void *arg)
{
    esp_foc_quad_obj_t *o = (esp_foc_quad_obj_t *)arg;
    pcnt_dev_t *dev = o->dev;
    if ((pcnt_ll_get_intr_status(dev) & o->unit_bit) == 0u) {
        return;
    }
    uint32_t ev = pcnt_ll_get_event_status(dev, o->unit);
    int32_t t = o->turns;
    if (ev & QUAD_EV_HIGH_LIMIT) {
        t++;
    } else if (ev & QUAD_EV_LOW_LIMIT) {
        t--;
    }
    o->wseq++;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    pcnt_ll_clear_intr_status(dev, o->unit_bit);
    o->turns = t;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    o->wseq++;
}

/*
 * Turns and in-turn count of one instant. A limit event between the two
 * counter reads leaves the first one ambiguous (before or after the reset), so
 * it is re-read once the pending bit is seen. A pending event whose ISR has
 * not run yet (held off by the hot path, or on its way on the other core) is
 * folded in here. The limit ISR can run on another core, hence the sequence
 * check instead of a critical section. Task context: an ISR that preempted the
 * limit ISR mid-update would spin here forever.
 */
static void read_total(esp_foc_quad_obj_t *o, int32_t *turns, int32_t *cnt)
{
    pcnt_dev_t *dev = o->dev;
    for (;;) {
        uint32_t s0 = o->wseq;
        __atomic_thread_fence(__ATOMIC_SEQ_CST);
        if (s0 & 1u) {
            continue;
        }
        int32_t t = o->turns;
        uint32_t p0 = pcnt_ll_get_intr_status(dev) & o->unit_bit;
        int32_t c = pcnt_ll_get_count(dev, o->unit);
        uint32_t p1 = pcnt_ll_get_intr_status(dev) & o->unit_bit;
        if (p1 != 0u) {
            if (p0 == 0u) {
                c = pcnt_ll_get_count(dev, o->unit);
            }
            uint32_t ev = pcnt_ll_get_event_status(dev, o->unit);
            if (ev & QUAD_EV_HIGH_LIMIT) {
                t++;
            } else if (ev & QUAD_EV_LOW_LIMIT) {
                t--;
            }
        }
        __atomic_thread_fence(__ATOMIC_SEQ_CST);
        if (o->wseq == s0) {
            *turns = t;
            *cnt = c;
            return;
        }
    }
}

static q16_t api_get_position(esp_foc_rotor_sensor_t *self)
{
    esp_foc_quad_obj_t *o = obj_from(self);
    return mech_from_cnt(o, signed_cnt(o));
}

static q16_t api_get_velocity(esp_foc_rotor_sensor_t *self)
{
    return obj_from(self)->vel_q16;
}

static q16_t api_get_electrical_position(esp_foc_rotor_sensor_t *self)
{
    esp_foc_quad_obj_t *o = obj_from(self);
    return elec_from_cnt(o, signed_cnt(o));
}

static esp_err_t api_fetch(esp_foc_rotor_sensor_t *self)
{
    esp_foc_quad_obj_t *o = obj_from(self);
    if (!o->inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!esp_foc_in_task_context()) {
        return ESP_ERR_INVALID_STATE;
    }

    int32_t t = 0;
    int32_t c = 0;
    read_total(o, &t, &c);
    int64_t total = (int64_t)t * (int64_t)o->cpr + (int64_t)c;

    q16_t vel = 0;
    if (o->have_prev) {
        int64_t d = total - o->prev_total;
        if (o->invert) {
            d = -d;
        }
        int64_t w = d * (int64_t)o->k_vel;
        if (w > (int64_t)INT32_MAX) {
            w = INT32_MAX;
        } else if (w < (int64_t)INT32_MIN) {
            w = INT32_MIN;
        }
        vel = (q16_t)w;
    }
    o->prev_total = total;
    o->have_prev = true;
    o->vel_q16 = vel;
    o->lat_cnt = c;
    o->seq++;
    o->age_periods = 0;
    return ESP_OK;
}

static esp_err_t api_fetch_start(esp_foc_rotor_sensor_t *self)
{
    return api_fetch(self);
}

/* The count is relative anyway: zero it in hardware with the rotor at rest. */
static esp_err_t api_calibrate_offset(esp_foc_rotor_sensor_t *self, int samples)
{
    esp_foc_quad_obj_t *o = obj_from(self);
    if (!o->inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (samples < 1) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_foc_critical_enter();
    o->wseq++;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    pcnt_ll_clear_count(o->dev, o->unit);
    pcnt_ll_clear_intr_status(o->dev, o->unit_bit);
    o->turns = 0;
    o->lat_cnt = 0;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    o->wseq++;
    esp_foc_critical_leave();

    o->prev_total = 0;
    o->have_prev = true;
    o->vel_q16 = 0;
    o->seq++;
    o->age_periods = 0;
    return ESP_OK;
}

static uint32_t api_caps(const esp_foc_rotor_sensor_t *self)
{
    const esp_foc_quad_obj_t *o = obj_from_const(self);
    uint32_t c = (uint32_t)ESP_FOC_ROTOR_CAP_MECH_ABS;
    if (o->pole_pairs != 0u) {
        c |= (uint32_t)ESP_FOC_ROTOR_CAP_ELEC_ABS;
    }
    return c;
}

/* In-turn angle only, so the limit reset needs no turn bookkeeping here. */
static void api_step(esp_foc_rotor_sensor_t *self)
{
    esp_foc_quad_obj_t *o = obj_from(self);
    if (!o->inited) {
        return;
    }
    o->lat_cnt = pcnt_ll_get_count(o->dev, o->unit);
    o->seq++;
    o->age_periods = 0;
}

static void api_snapshot(const esp_foc_rotor_sensor_t *self, esp_foc_rotor_state_t *out)
{
    const esp_foc_quad_obj_t *o = obj_from_const(self);
    int32_t c = signed_cnt(o);
    q16_t w = o->vel_q16;
    out->theta_m = mech_from_cnt(o, c);
    out->theta_e = elec_from_cnt(o, c);
    out->omega_m = w;
    out->omega_e = (q16_t)((int64_t)w * (int64_t)o->pole_pairs);
    out->seq = o->seq;
    out->age_periods = o->age_periods;
    out->sector = ESP_FOC_ROTOR_SECTOR_UNKNOWN;
    out->valid = o->inited;
    out->moving = w != 0;
}

static void bind_vtable(esp_foc_quad_obj_t *o)
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

static void route_input(int gpio, uint32_t sig_a, uint32_t sig_b)
{
    esp_rom_gpio_pad_select_gpio((uint32_t)gpio);
    gpio_ll_input_enable(&GPIO, (uint32_t)gpio);
    gpio_ll_pullup_en(&GPIO, (uint32_t)gpio);
    gpio_ll_pulldown_dis(&GPIO, (uint32_t)gpio);
    esp_rom_gpio_connect_in_signal((uint32_t)gpio, sig_a, false);
    esp_rom_gpio_connect_in_signal((uint32_t)gpio, sig_b, false);
}

static void unroute_inputs(const esp_foc_quad_obj_t *o)
{
    for (uint32_t ch = 0; ch < 2u; ch++) {
        esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT,
                                       ESP_FOC_PCNT_PULSE_SIG(o->group, o->unit, ch), false);
        esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT,
                                       ESP_FOC_PCNT_CTRL_SIG(o->group, o->unit, ch), false);
    }
}

/*
 * x4 decode: each channel counts both edges of one line, the other line's
 * level picks the direction. A leading B counts up.
 */
static void program_unit(esp_foc_quad_obj_t *o, uint32_t filter_cycles)
{
    pcnt_dev_t *dev = o->dev;
    uint32_t u = o->unit;

    pcnt_ll_stop_count(dev, u);
    pcnt_ll_enable_intr(dev, o->unit_bit, false);
    pcnt_ll_disable_all_events(dev, u);

    pcnt_ll_set_edge_action(dev, u, 0, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                            PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_ll_set_level_action(dev, u, 0, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                             PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_ll_set_edge_action(dev, u, 1, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                            PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_ll_set_level_action(dev, u, 1, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                             PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    if (filter_cycles != 0u) {
        pcnt_ll_set_glitch_filter_thres(dev, u, filter_cycles);
        pcnt_ll_enable_glitch_filter(dev, u, true);
    } else {
        pcnt_ll_enable_glitch_filter(dev, u, false);
    }

    pcnt_ll_set_high_limit_value(dev, u, o->cpr);
    pcnt_ll_set_low_limit_value(dev, u, -o->cpr);
    pcnt_ll_enable_high_limit_event(dev, u, true);
    pcnt_ll_enable_low_limit_event(dev, u, true);
    /* Limits are loaded into the counter logic on reset. */
    pcnt_ll_clear_count(dev, u);
    pcnt_ll_clear_intr_status(dev, o->unit_bit);
}

static esp_err_t glitch_cycles(uint32_t ns, uint32_t *out)
{
    *out = 0;
    if (ns == 0u) {
        return ESP_OK;
    }
    uint32_t hz = 0;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    esp_err_t err = esp_clk_tree_src_get_freq_hz((soc_module_clk_t)PCNT_CLK_SRC_DEFAULT,
                                                 ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &hz);
    if (err != ESP_OK) {
        return err;
    }
#else
    hz = (uint32_t)esp_clk_apb_freq();
#endif
    uint64_t cyc = ((uint64_t)ns * (uint64_t)hz + 999999999ull) / 1000000000ull;
    if (cyc > PCNT_LL_MAX_GLITCH_WIDTH) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = (uint32_t)cyc;
    return ESP_OK;
}

static bool gpio_ok(int gpio)
{
    return gpio >= 0 && gpio < SOC_GPIO_PIN_COUNT &&
           ((SOC_GPIO_VALID_GPIO_MASK >> gpio) & 1ull) != 0u;
}

static bool unit_taken(const esp_foc_quad_obj_t *self, int group, int unit)
{
    for (unsigned i = 0; i < CONFIG_ESP_FOC_MAX_ROTOR_SENSORS; i++) {
        const esp_foc_quad_obj_t *o = &s_pool[i];
        if (o != self && o->inited && o->group == group && (int)o->unit == unit) {
            return true;
        }
    }
    return false;
}

esp_foc_rotor_sensor_t *esp_foc_rotor_quadrature_acquire(unsigned index)
{
    if (index >= CONFIG_ESP_FOC_MAX_ROTOR_SENSORS) {
        return NULL;
    }
    esp_foc_quad_obj_t *o = &s_pool[index];
    if (o->acquired) {
        return NULL;
    }
    memset(o, 0, sizeof(*o));
    o->acquired = true;
    bind_vtable(o);
    return &o->iface;
}

void esp_foc_rotor_quadrature_release(esp_foc_rotor_sensor_t *s)
{
    if (s == NULL) {
        return;
    }
    esp_foc_quad_obj_t *o = obj_from(s);
    if (o->inited) {
        (void)esp_foc_rotor_quadrature_deinit(s);
    }
    o->acquired = false;
}

esp_err_t esp_foc_rotor_quadrature_init(esp_foc_rotor_sensor_t *s,
                                        const esp_foc_rotor_quadrature_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(s != NULL && cfg != NULL, ESP_ERR_INVALID_ARG, TAG, "null");
    esp_foc_quad_obj_t *o = obj_from(s);
    ESP_RETURN_ON_FALSE(o->acquired && !o->inited, ESP_ERR_INVALID_STATE, TAG, "state");
    ESP_RETURN_ON_FALSE(gpio_ok(cfg->gpio_a) && gpio_ok(cfg->gpio_b) &&
                        cfg->gpio_a != cfg->gpio_b,
                        ESP_ERR_INVALID_ARG, TAG, "gpio a=%d b=%d", cfg->gpio_a, cfg->gpio_b);
    ESP_RETURN_ON_FALSE(cfg->pcnt_group >= 0 && cfg->pcnt_group < (int)ESP_FOC_PCNT_GROUPS &&
                        cfg->pcnt_unit >= 0 && cfg->pcnt_unit < (int)ESP_FOC_PCNT_UNITS,
                        ESP_ERR_INVALID_ARG, TAG, "pcnt %d/%d", cfg->pcnt_group, cfg->pcnt_unit);
    ESP_RETURN_ON_FALSE(cfg->ppr >= 1u && cfg->ppr <= ESP_FOC_ROTOR_QUADRATURE_MAX_PPR,
                        ESP_ERR_INVALID_ARG, TAG, "ppr %lu", (unsigned long)cfg->ppr);
    ESP_RETURN_ON_FALSE(cfg->dt_seconds > 0.0f, ESP_ERR_INVALID_ARG, TAG, "dt");
    ESP_RETURN_ON_FALSE(cfg->irq_level >= 0 && cfg->irq_level <= 2,
                        ESP_ERR_INVALID_ARG, TAG, "irq_level %d", cfg->irq_level);
    ESP_RETURN_ON_FALSE(!unit_taken(o, cfg->pcnt_group, cfg->pcnt_unit),
                        ESP_ERR_INVALID_STATE, TAG, "pcnt %d/%d in use",
                        cfg->pcnt_group, cfg->pcnt_unit);

    int32_t cpr = (int32_t)(4u * cfg->ppr);
    float k_vel = 6.283185307f / ((float)cpr * cfg->dt_seconds);
    ESP_RETURN_ON_FALSE(k_vel < 32767.0f, ESP_ERR_INVALID_ARG, TAG,
                        "one count per dt exceeds the q16 speed range");

    uint32_t filter = 0;
    ESP_RETURN_ON_ERROR(glitch_cycles(cfg->glitch_ns, &filter), TAG,
                        "glitch_ns %lu", (unsigned long)cfg->glitch_ns);

    o->group = cfg->pcnt_group;
    o->unit = (uint32_t)cfg->pcnt_unit;
    o->unit_bit = PCNT_LL_UNIT_WATCH_EVENT(cfg->pcnt_unit);
    o->dev = PCNT_LL_GET_HW(cfg->pcnt_group);
    o->cpr = cpr;
    o->k_turn = (int64_t)((((uint64_t)1u << 32) + (uint64_t)cpr / 2u) / (uint64_t)cpr);
    o->k_elec = o->k_turn * (int64_t)cfg->pole_pairs;
    o->k_vel = q16_from_float(k_vel);
    o->invert = cfg->invert;
    o->pole_pairs = cfg->pole_pairs;
    o->gpio_a = cfg->gpio_a;
    o->gpio_b = cfg->gpio_b;
    o->turns = 0;
    o->lat_cnt = 0;
    o->wseq = 0;
    o->prev_total = 0;
    o->have_prev = false;
    o->vel_q16 = 0;

    if (s_group_users[o->group] == 0u) {
        pcnt_ll_enable_bus_clock(o->group, true);
        pcnt_ll_reset_register(o->group);
    }
    s_group_users[o->group]++;

    program_unit(o, filter);
    route_input(cfg->gpio_a, ESP_FOC_PCNT_PULSE_SIG(o->group, o->unit, 0),
                ESP_FOC_PCNT_CTRL_SIG(o->group, o->unit, 1));
    route_input(cfg->gpio_b, ESP_FOC_PCNT_PULSE_SIG(o->group, o->unit, 1),
                ESP_FOC_PCNT_CTRL_SIG(o->group, o->unit, 0));

    int level = (cfg->irq_level == 2) ? ESP_INTR_FLAG_LEVEL2 : ESP_INTR_FLAG_LEVEL1;
    esp_err_t err = esp_foc_drv_intr_alloc(ESP_FOC_PCNT_IRQ(o->group),
                                           ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_SHARED | level,
                                           (uint32_t)pcnt_ll_get_intr_status_reg(o->dev),
                                           o->unit_bit, quad_limit_isr, o, &o->intr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "limit irq alloc failed: %s", esp_err_to_name(err));
        unroute_inputs(o);
        pcnt_ll_disable_all_events(o->dev, o->unit);
        if (--s_group_users[o->group] == 0u) {
            pcnt_ll_enable_bus_clock(o->group, false);
        }
        return err;
    }

    pcnt_ll_enable_intr(o->dev, o->unit_bit, true);
    pcnt_ll_start_count(o->dev, o->unit);
    o->inited = true;

    ESP_LOGI(TAG, "quadrature ready a=%d b=%d pcnt=%d/%lu ppr=%lu cpr=%ld filter=%lu cyc "
             "dt=%.6f pp=%u inv=%d",
             cfg->gpio_a, cfg->gpio_b, o->group, (unsigned long)o->unit,
             (unsigned long)cfg->ppr, (long)cpr, (unsigned long)filter,
             (double)cfg->dt_seconds, o->pole_pairs, (int)o->invert);
    return ESP_OK;
}

esp_err_t esp_foc_rotor_quadrature_deinit(esp_foc_rotor_sensor_t *s)
{
    if (s == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_foc_quad_obj_t *o = obj_from(s);
    if (!o->inited) {
        return ESP_OK;
    }
    pcnt_ll_enable_intr(o->dev, o->unit_bit, false);
    pcnt_ll_stop_count(o->dev, o->unit);
    pcnt_ll_disable_all_events(o->dev, o->unit);
    pcnt_ll_clear_intr_status(o->dev, o->unit_bit);
    if (o->intr != NULL) {
        esp_intr_free(o->intr);
        o->intr = NULL;
    }
    unroute_inputs(o);
    o->inited = false;
    if (--s_group_users[o->group] == 0u) {
        pcnt_ll_enable_bus_clock(o->group, false);
    }
    return ESP_OK;
}

int64_t esp_foc_rotor_quadrature_get_count(const esp_foc_rotor_sensor_t *s)
{
    if (s == NULL) {
        return 0;
    }
    esp_foc_quad_obj_t *o = (esp_foc_quad_obj_t *)obj_from_const(s);
    if (!o->inited) {
        return 0;
    }
    int32_t t = 0;
    int32_t c = 0;
    read_total(o, &t, &c);
    int64_t total = (int64_t)t * (int64_t)o->cpr + (int64_t)c;
    return o->invert ? -total : total;
}

uint32_t esp_foc_rotor_quadrature_get_cpr(const esp_foc_rotor_sensor_t *s)
{
    return (s != NULL) ? (uint32_t)obj_from_const(s)->cpr : 0u;
}

void esp_foc_rotor_quadrature_set_invert(esp_foc_rotor_sensor_t *s, bool invert)
{
    if (s == NULL) {
        return;
    }
    esp_foc_quad_obj_t *o = obj_from(s);
    o->invert = invert;
    o->have_prev = false;
}

void esp_foc_rotor_quadrature_set_pole_pairs(esp_foc_rotor_sensor_t *s,
                                             unsigned pole_pairs)
{
    if (s == NULL) {
        return;
    }
    esp_foc_quad_obj_t *o = obj_from(s);
    o->k_elec = o->k_turn * (int64_t)pole_pairs;
    o->pole_pairs = pole_pairs;
}
