/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Edge timestamps latched in hardware: three GPIO any-edge ETM events all
 * driving one timer-group capture task.
 *
 * One capture register serves all three lines because the code is recovered by
 * reading the GPIO levels, not by asking which event fired — and hall edges sit
 * 60 electrical degrees apart, so they never coincide.
 */
#include "esp_foc_hall_soc_gate.h"

#include "hall_timestamp.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_private/periph_ctrl.h"
#include "hal/etm_ll.h"
#include "hal/gpio_etm_ll.h"
#include "hal/timer_ll.h"
#include "soc/gpio_ext_struct.h"
#include "soc/pcr_struct.h"
#include "soc/soc_caps.h"
#include "soc/soc_etm_source.h"
#include "soc/soc_etm_struct.h"
#include "soc/timer_periph.h"

static const char *TAG = "foc_hall_etm";

/* Only where the clock-source bits are shared with another peripheral does
 * setting them need the critical section. Same gate IDF's gptimer uses. */
#if SOC_PERIPH_CLK_CTRL_SHARED
#define HALL_CLK_SRC_ATOMIC() PERIPH_RCC_ATOMIC()
#else
#define HALL_CLK_SRC_ATOMIC()
#endif

/* PLL_F80M / 80 = 1 MHz, i.e. one tick per microsecond. */
#define TIMG_SRC_HZ   80000000u
#define TIMG_DIVIDER  80u
#define TIMG_TICK_HZ  (TIMG_SRC_HZ / TIMG_DIVIDER)

static timg_dev_t *timg_of(const esp_foc_hall_ts_t *ts)
{
    return (timg_dev_t *)ts->timg;
}

static void program_channels(esp_foc_hall_ts_t *ts)
{
    uint32_t task_id = TIMER_LL_ETM_TASK_TABLE(ts->cfg.timer_group, 0,
                                               GPTIMER_ETM_TASK_CAPTURE);

    for (int i = 0; i < 3; i++) {
        uint32_t gch = (uint32_t)i;
        uint32_t ech = (uint32_t)ts->cfg.etm_channel[i];

        gpio_ll_etm_event_channel_set_gpio(&GPIO_ETM, gch, (uint32_t)ts->cfg.gpio[i]);
        gpio_ll_etm_enable_event_channel(&GPIO_ETM, gch, true);

        etm_ll_disable_channel(&SOC_ETM, ech);
        etm_ll_channel_set_event(&SOC_ETM, ech, GPIO_LL_ETM_EVENT_ID_ANY_EDGE(gch));
        etm_ll_channel_set_task(&SOC_ETM, ech, task_id);
        etm_ll_enable_channel(&SOC_ETM, ech);
    }
}

static esp_err_t etm_init(esp_foc_hall_ts_t *ts)
{
    const esp_foc_hall_ts_cfg_t *cfg = &ts->cfg;

    ESP_RETURN_ON_FALSE(cfg->timer_group >= 0 && cfg->timer_group < SOC_TIMER_GROUPS,
                        ESP_ERR_INVALID_ARG, TAG, "group");
    for (int i = 0; i < 3; i++) {
        ESP_RETURN_ON_FALSE(cfg->etm_channel[i] >= 0 &&
                                (uint32_t)cfg->etm_channel[i] < SOC_ETM_CHANNELS_PER_GROUP,
                            ESP_ERR_INVALID_ARG, TAG, "chan");
        for (int j = i + 1; j < 3; j++) {
            ESP_RETURN_ON_FALSE(cfg->etm_channel[i] != cfg->etm_channel[j],
                                ESP_ERR_INVALID_ARG, TAG, "chan dup");
        }
    }

    /*
     * The init-order contract. Enabling the ETM bus clock is one idempotent
     * bit; resetting ETM wipes all 50 channels. This driver only ever does the
     * former, so inverter-then-hall is safe with no coordination: the
     * destructive reset happens before any hall channel exists to be wiped.
     *
     * "Is there an inverter?" cannot be asked without including one, which is
     * the dependency edge this whole design removes. The predicate that
     * actually matters is "did anybody bring ETM up before me", and that is
     * readable from the peripheral itself.
     *
     * Honest limit: IDF gates this clock off during startup only after a
     * power-on reset, so the reading is exact from cold and merely advisory
     * after a software or JTAG reset, where the previous run's state survives.
     */
    ts->etm_cold_start = (PCR.etm_conf.etm_clk_en == 0);
    if (ts->etm_cold_start) {
        if (cfg->require_etm_ready) {
            ESP_LOGE(TAG, "ETM is cold: create the inverter before the hall rotor "
                          "sensor, or clear require_etm_ready");
            return ESP_ERR_INVALID_STATE;
        }
        ESP_LOGW(TAG, "ETM cold start: this driver is the first ETM user");
    }
    etm_ll_enable_bus_clock(0, true);
    /* Never etm_ll_reset_register() here. See above. */

    /*
     * Timer group. Bus clock and timer clock are both gated off by IDF at
     * startup — the MWDT in this group runs off a different clock, so nothing
     * has turned these on for us.
     *
     * Going through IDF's reference count rather than writing the bit directly
     * is what keeps this from fighting a gptimer the application may open in
     * the same group: neither can switch the clock off under the other.
     *
     * timer_ll_reset_register() is never called: it resets the whole group and
     * would take the watchdog with it.
     */
    PERIPH_RCC_ACQUIRE_ATOMIC(timer_group_periph_signals.groups[cfg->timer_group].module,
                              ref_count) {
        if (ref_count == 0) {
            timer_ll_enable_bus_clock(cfg->timer_group, true);
        }
    }
    HALL_CLK_SRC_ATOMIC() {
        timer_ll_set_clock_source(cfg->timer_group, 0, GPTIMER_CLK_SRC_PLL_F80M);
        timer_ll_enable_clock(cfg->timer_group, 0, true);
    }

    timg_dev_t *hw = TIMER_LL_GET_HW(cfg->timer_group);
    ts->timg = hw;

    timer_ll_enable_counter(hw, 0, false);
    timer_ll_enable_intr(hw, TIMER_LL_EVENT_ALARM(0), false);
    timer_ll_enable_alarm(hw, 0, false);
    timer_ll_enable_auto_reload(hw, 0, false);
    timer_ll_set_count_direction(hw, 0, GPTIMER_COUNT_UP);
    timer_ll_set_clock_prescale(hw, 0, TIMG_DIVIDER);
    timer_ll_set_reload_value(hw, 0, 0);
    timer_ll_trigger_soft_reload(hw, 0);
    timer_ll_enable_counter(hw, 0, true);
    timer_ll_enable_etm(hw, true);

    ts->tick_hz = TIMG_TICK_HZ;

    program_channels(ts);

    /*
     * Baseline the latch so the first poll() does not read a leftover value as
     * an edge. Soft capture is fine here and forbidden in the hot path: it
     * busy-waits on the cross-domain update and writes the very hi/lo pair the
     * ETM task latches, so using it to read "now" would destroy a pending edge.
     */
    timer_ll_trigger_soft_capture(hw, 0);
    ts->last_cap = timer_ll_get_counter_value(hw, 0);

    ESP_LOGI(TAG, "hall ts ETM+TIMG: gpio=%d/%d/%d etm=%d/%d/%d tg=%d tick=%luHz cold=%d",
             cfg->gpio[0], cfg->gpio[1], cfg->gpio[2],
             cfg->etm_channel[0], cfg->etm_channel[1], cfg->etm_channel[2],
             cfg->timer_group, (unsigned long)ts->tick_hz, ts->etm_cold_start ? 1 : 0);
    return ESP_OK;
}

/*
 * Hot path: two register reads and not one write. The value is latched rather
 * than free-running, so reading hi and lo separately cannot tear and no
 * critical section is needed.
 */
static bool etm_poll(esp_foc_hall_ts_t *ts, uint64_t *ticks)
{
    uint64_t v = timer_ll_get_counter_value(timg_of(ts), 0);
    if (v == ts->last_cap) {
        return false;
    }
    ts->last_cap = v;
    *ticks = v;
    return true;
}

static esp_err_t etm_rearm(esp_foc_hall_ts_t *ts)
{
    etm_ll_enable_bus_clock(0, true);
    timer_ll_enable_etm(timg_of(ts), true);
    program_channels(ts);
    timer_ll_trigger_soft_capture(timg_of(ts), 0);
    ts->last_cap = timer_ll_get_counter_value(timg_of(ts), 0);
    return ESP_OK;
}

static bool etm_healthy(const esp_foc_hall_ts_t *ts)
{
    for (int i = 0; i < 3; i++) {
        if (!etm_ll_is_channel_enabled(&SOC_ETM, (uint32_t)ts->cfg.etm_channel[i])) {
            return false;
        }
        /* Also catches a channel re-pointed at somebody else's pin. */
        if (gpio_ll_etm_event_channel_get_gpio(&GPIO_ETM, (uint32_t)i) !=
            (uint32_t)ts->cfg.gpio[i]) {
            return false;
        }
    }
    return true;
}

static void etm_deinit(esp_foc_hall_ts_t *ts)
{
    for (int i = 0; i < 3; i++) {
        etm_ll_disable_channel(&SOC_ETM, (uint32_t)ts->cfg.etm_channel[i]);
        gpio_ll_etm_enable_event_channel(&GPIO_ETM, (uint32_t)i, false);
    }
    timer_ll_enable_etm(timg_of(ts), false);
    /* Counter and clocks are left running: the group is shared. */
}

const esp_foc_hall_ts_ops_t esp_foc_hall_ts_etm_timg_ops = {
    .init = etm_init,
    .poll = etm_poll,
    .rearm = etm_rearm,
    .healthy = etm_healthy,
    .deinit = etm_deinit,
};
