/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#include "esp_foc_driver_soc_gate.h"

#include <string.h>

#include "esp_check.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "hal/mcpwm_hal.h"
#include "hal/mcpwm_ll.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"
#include "soc/mcpwm_struct.h"
#include "soc/mcpwm_periph.h"
#include "soc/clk_tree_defs.h"
#include "soc/interrupts.h"
#include "soc/io_mux_reg.h"

#include "mcpwm_bridge.h"

static const char *TAG = "foc_mcpwm";

#define GROUP_CLK_HZ 80000000u
#define FAULT_SIG_ID 0

static void mcpwm_gpio_as_output(int gpio)
{
    if (gpio < 0) {
        return;
    }
    esp_rom_gpio_pad_select_gpio((uint32_t)gpio);
    gpio_ll_func_sel(&GPIO, (uint8_t)gpio, PIN_FUNC_GPIO);
    /* Keep input enabled so gpio_ll_get_level() can read back the driven level. */
    gpio_ll_input_enable(&GPIO, gpio);
    gpio_ll_output_enable(&GPIO, gpio);
    gpio_ll_pulldown_dis(&GPIO, gpio);
    gpio_ll_pullup_dis(&GPIO, gpio);
}

static void mcpwm_gpio_as_fault_in(int gpio)
{
    if (gpio < 0) {
        return;
    }
    esp_rom_gpio_pad_select_gpio((uint32_t)gpio);
    gpio_ll_func_sel(&GPIO, (uint8_t)gpio, PIN_FUNC_GPIO);
    gpio_ll_output_disable(&GPIO, gpio);
    gpio_ll_input_enable(&GPIO, gpio);
    gpio_ll_pulldown_dis(&GPIO, gpio);
    gpio_ll_pullup_dis(&GPIO, gpio);
}

static void mcpwm_route_out(int gpio, uint32_t signal_idx)
{
    if (gpio < 0) {
        return;
    }
    /* Match IDF mcpwm_gen: IOMUX GPIO + matrix out (connect enables pad OE). */
    esp_rom_gpio_pad_select_gpio((uint32_t)gpio);
    gpio_ll_func_sel(&GPIO, (uint8_t)gpio, PIN_FUNC_GPIO);
    gpio_ll_input_enable(&GPIO, gpio);
    gpio_ll_pulldown_dis(&GPIO, gpio);
    gpio_ll_pullup_dis(&GPIO, gpio);
    esp_rom_gpio_connect_out_signal((uint32_t)gpio, signal_idx, false, false);
}

static void mcpwm_setup_deadtime_ahc(mcpwm_dev_t *dev, int op, uint32_t dt_ticks)
{
    if (dt_ticks < 1u) {
        dt_ticks = 1u;
    }
    mcpwm_ll_deadtime_clock_src_t clk = MCPWM_LL_DEADTIME_CLK_SRC_TIMER;
    mcpwm_ll_operator_set_deadtime_clock_src(dev, op, clk);

    mcpwm_ll_deadtime_bypass_path(dev, op, 0, false);
    mcpwm_ll_deadtime_red_select_generator(dev, op, 0);
    mcpwm_ll_deadtime_set_rising_delay(dev, op, dt_ticks);
    mcpwm_ll_deadtime_invert_outpath(dev, op, 0, false);
    mcpwm_ll_deadtime_swap_out_path(dev, op, 0, false);

    mcpwm_ll_deadtime_bypass_path(dev, op, 1, false);
    mcpwm_ll_deadtime_fed_select_generator(dev, op, 0);
    mcpwm_ll_deadtime_set_falling_delay(dev, op, dt_ticks);
    mcpwm_ll_deadtime_invert_outpath(dev, op, 1, true);
    mcpwm_ll_deadtime_swap_out_path(dev, op, 1, false);

    mcpwm_ll_deadtime_enable_deb(dev, op, false);
    mcpwm_ll_deadtime_update_delay_at_once(dev, op);
}

static void mcpwm_setup_ost_brake(mcpwm_dev_t *dev, int op)
{
    mcpwm_ll_brake_enable_soft_ost(dev, op, true);

    mcpwm_ll_generator_set_action_on_brake_event(
        dev, op, 0, MCPWM_TIMER_DIRECTION_UP, MCPWM_OPER_BRAKE_MODE_OST, MCPWM_GEN_ACTION_LOW);
    mcpwm_ll_generator_set_action_on_brake_event(
        dev, op, 0, MCPWM_TIMER_DIRECTION_DOWN, MCPWM_OPER_BRAKE_MODE_OST, MCPWM_GEN_ACTION_LOW);
}

static void watch_step(esp_foc_mcpwm_bridge_t *b, uint32_t mask)
{
    esp_foc_mcpwm_watch_fn_t fn = b->watch_fn;
    if (fn != NULL && b->watch_skip != 0u) {
        b->watch_skip--;
        return;
    }
    if (fn == NULL || fn(b->watch_arg)) {
        b->watch_fn = NULL;
        /* Only this handler clears the enable, and watch_sample() only sets it,
         * so the two read-modify-writes cannot lose another bit. */
        mcpwm_ll_intr_enable(b->dev, mask, false);
    }
}

static void mcpwm_isr(void *arg)
{
    esp_foc_mcpwm_bridge_t *b = (esp_foc_mcpwm_bridge_t *)arg;
    uint32_t st = mcpwm_ll_intr_get_status(b->dev);
    if (b->sample_timer_id >= 0) {
        const uint32_t sample_mask = MCPWM_LL_EVENT_TIMER_EMPTY(b->sample_timer_id);
        if ((st & sample_mask) != 0u) {
            mcpwm_ll_intr_clear_status(b->dev, sample_mask);
            watch_step(b, sample_mask);
        }
    }
    uint32_t handled = st & b->irq_mask;

    if (handled == 0) {
        return;
    }

    uint32_t fault_bits = handled & MCPWM_LL_EVENT_FAULT_ENTER(FAULT_SIG_ID);
    if (fault_bits != 0u) {
        mcpwm_ll_intr_clear_status(b->dev, fault_bits);
        if (b->fault_cb != NULL) {
            b->fault_cb(b->fault_arg, fault_bits);
        }
    }

    uint32_t tez_mask = MCPWM_LL_EVENT_TIMER_EMPTY(b->timer_id);
    if ((handled & tez_mask) != 0u) {
        mcpwm_ll_intr_clear_status(b->dev, tez_mask);
        if (b->isr_cb != NULL) {
            b->isr_cb(b->isr_arg);
        }
    }
}

esp_err_t esp_foc_mcpwm_bridge_init(esp_foc_mcpwm_bridge_t *b,
                                    const esp_foc_mcpwm_bridge_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(b != NULL && cfg != NULL, ESP_ERR_INVALID_ARG, TAG, "null");
    ESP_RETURN_ON_FALSE(cfg->pwm_hz >= 5000u && cfg->pwm_hz <= 40000u, ESP_ERR_INVALID_ARG, TAG, "pwm_hz");
    memset(b, 0, sizeof(*b));

    b->group_id = cfg->group_id;
    b->timer_id = cfg->timer_id;
    b->sample_timer_id = -1;
    b->pwm_hz = cfg->pwm_hz;
    b->isr_cb = cfg->isr_cb;
    b->isr_arg = cfg->isr_arg;
    b->fault_cb = cfg->fault_cb;
    b->fault_arg = cfg->fault_arg;
    b->gpio_enable = cfg->gpio_enable;
    b->enable_active_low = cfg->enable_active_low;
    b->gpio_fault = cfg->gpio_fault;
    b->fault_active_high = cfg->fault_active_high;

    int gid = cfg->group_id;
    mcpwm_ll_enable_bus_clock(gid, true);
    mcpwm_ll_reset_register(gid);
    mcpwm_ll_group_enable_clock(gid, true);
    mcpwm_ll_group_set_clock_source(gid, SOC_MOD_CLK_PLL_F160M);
    mcpwm_ll_group_set_clock_prescale(gid, 2);

    mcpwm_hal_context_t hal = {0};
    mcpwm_hal_init_config_t hal_cfg = { .group_id = gid };
    mcpwm_hal_init(&hal, &hal_cfg);
    b->dev = MCPWM_LL_GET_HW(gid);
    ESP_RETURN_ON_FALSE(b->dev != NULL, ESP_ERR_NOT_SUPPORTED, TAG, "no mcpwm hw");

    mcpwm_hal_timer_reset(&hal, cfg->timer_id);
    mcpwm_ll_timer_set_clock_prescale(b->dev, cfg->timer_id, 1);

    /* Center-aligned: full period = 2 * peak ticks at group clock. */
    uint32_t peak = GROUP_CLK_HZ / (2u * cfg->pwm_hz);
    if (peak < 2u) {
        peak = 2u;
    }
    if (peak > (MCPWM_LL_MAX_COUNT_VALUE - 1u)) {
        peak = MCPWM_LL_MAX_COUNT_VALUE - 1u;
    }
    b->peak = peak;

    mcpwm_ll_timer_set_count_mode(b->dev, cfg->timer_id, MCPWM_TIMER_COUNT_MODE_UP_DOWN);
    mcpwm_ll_timer_set_peak(b->dev, cfg->timer_id, peak, true);
    mcpwm_ll_timer_update_period_at_once(b->dev, cfg->timer_id);

    /*
     * The ADC start has to land anywhere in the period, and the PWM timer's own
     * events only offer TEZ and TEP. A second timer, counting up over the PWM
     * period (or an integer fraction of it) and reloaded with a phase on every
     * PWM TEZ, puts its TEZ at any chosen delay and stays locked because the
     * reload repeats each period.
     */
    if (cfg->sample_timer_id >= 0) {
        const int st = cfg->sample_timer_id;
        ESP_RETURN_ON_FALSE(st < 3 && st != cfg->timer_id, ESP_ERR_INVALID_ARG, TAG,
                            "sample timer");
        mcpwm_hal_timer_reset(&hal, st);
        mcpwm_ll_timer_set_clock_prescale(b->dev, st, 1);
        mcpwm_ll_timer_set_count_mode(b->dev, st, MCPWM_TIMER_COUNT_MODE_UP);
        mcpwm_ll_timer_set_peak(b->dev, st, 2u * peak, false);
        mcpwm_ll_timer_update_period_at_once(b->dev, st);
        mcpwm_ll_timer_sync_out_on_timer_event(b->dev, cfg->timer_id, MCPWM_TIMER_EVENT_EMPTY);
        mcpwm_ll_timer_set_timer_sync_input(b->dev, st, cfg->timer_id);
        mcpwm_ll_timer_set_sync_phase_direction(b->dev, st, MCPWM_TIMER_DIRECTION_UP);
        mcpwm_ll_timer_enable_sync_input(b->dev, st, true);
        b->sample_timer_id = st;
        b->sample_period = 2u * peak;
        esp_foc_mcpwm_bridge_set_sample_delay(b, (int32_t)peak);
    }

    uint32_t dt_ticks = (uint32_t)(((uint64_t)cfg->deadtime_ns * (uint64_t)GROUP_CLK_HZ) / 1000000000ull);
    if (dt_ticks < 1u) {
        dt_ticks = 1u;
    }

    const int gpios_h[3] = { cfg->gpio_uh, cfg->gpio_vh, cfg->gpio_wh };
    const int gpios_l[3] = { cfg->gpio_ul, cfg->gpio_vl, cfg->gpio_wl };
    const uint32_t sig_a[3] = {
        mcpwm_periph_signals.groups[gid].operators[0].generators[0].pwm_sig,
        mcpwm_periph_signals.groups[gid].operators[1].generators[0].pwm_sig,
        mcpwm_periph_signals.groups[gid].operators[2].generators[0].pwm_sig,
    };
    const uint32_t sig_b[3] = {
        mcpwm_periph_signals.groups[gid].operators[0].generators[1].pwm_sig,
        mcpwm_periph_signals.groups[gid].operators[1].generators[1].pwm_sig,
        mcpwm_periph_signals.groups[gid].operators[2].generators[1].pwm_sig,
    };

    for (int op = 0; op < 3; op++) {
        mcpwm_hal_operator_reset(&hal, op);
        mcpwm_ll_operator_connect_timer(b->dev, op, cfg->timer_id);
        mcpwm_ll_operator_set_compare_value(b->dev, op, 0, 0);
        mcpwm_ll_operator_enable_update_compare_on_tez(b->dev, op, 0, true);
        mcpwm_ll_operator_update_compare_at_once(b->dev, op, 0);

        mcpwm_hal_generator_reset(&hal, op, 0);
        mcpwm_ll_generator_reset_actions(b->dev, op, 0);
        mcpwm_ll_generator_set_action_on_compare_event(
            b->dev, op, 0, MCPWM_TIMER_DIRECTION_UP, 0, MCPWM_GEN_ACTION_LOW);
        mcpwm_ll_generator_set_action_on_compare_event(
            b->dev, op, 0, MCPWM_TIMER_DIRECTION_DOWN, 0, MCPWM_GEN_ACTION_HIGH);

        mcpwm_setup_ost_brake(b->dev, op);
        mcpwm_setup_deadtime_ahc(b->dev, op, dt_ticks);
        mcpwm_route_out(gpios_h[op], sig_a[op]);
        mcpwm_route_out(gpios_l[op], sig_b[op]);
    }

    if (b->gpio_enable >= 0) {
        mcpwm_gpio_as_output(b->gpio_enable);
        gpio_ll_set_level(&GPIO, b->gpio_enable, b->enable_active_low ? 1 : 0);
        ESP_LOGI(TAG, "EN gpio=%d active_low=%d init_level=%d",
                 b->gpio_enable,
                 b->enable_active_low ? 1 : 0,
                 b->enable_active_low ? 1 : 0);
    }

    b->irq_mask = MCPWM_LL_EVENT_TIMER_EMPTY(cfg->timer_id);
    const uint32_t sample_mask =
        (b->sample_timer_id >= 0) ? MCPWM_LL_EVENT_TIMER_EMPTY(b->sample_timer_id) : 0u;
    mcpwm_ll_intr_enable(b->dev, sample_mask, false);

    if (b->gpio_fault >= 0) {
        mcpwm_gpio_as_fault_in(b->gpio_fault);
        esp_rom_gpio_connect_in_signal(
            (uint32_t)b->gpio_fault,
            mcpwm_periph_signals.groups[gid].gpio_faults[FAULT_SIG_ID].fault_sig,
            false);
        mcpwm_ll_fault_set_active_level(b->dev, FAULT_SIG_ID, b->fault_active_high);
        mcpwm_ll_fault_enable_detection(b->dev, FAULT_SIG_ID, true);
        for (int op = 0; op < 3; op++) {
            mcpwm_ll_brake_enable_oneshot_mode(b->dev, op, FAULT_SIG_ID, true);
        }
        b->irq_mask |= MCPWM_LL_EVENT_FAULT_ENTER(FAULT_SIG_ID);
    }

    mcpwm_ll_intr_clear_status(b->dev, b->irq_mask);
    mcpwm_ll_intr_enable(b->dev, b->irq_mask, true);

    esp_err_t err = esp_intr_alloc_intrstatus(
        mcpwm_periph_signals.groups[gid].irq_id,
        ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3,
        (uint32_t)mcpwm_ll_intr_get_status_reg(b->dev),
        b->irq_mask | sample_mask,
        mcpwm_isr,
        b,
        &b->intr);
    ESP_RETURN_ON_ERROR(err, TAG, "intr alloc");

    b->inited = true;
    return ESP_OK;
}

void esp_foc_mcpwm_bridge_deinit(esp_foc_mcpwm_bridge_t *b)
{
    if (b == NULL || !b->inited) {
        return;
    }
    esp_foc_mcpwm_bridge_stop(b);
    esp_foc_mcpwm_bridge_enable_output(b, false);
    if (b->intr != NULL) {
        esp_intr_free(b->intr);
        b->intr = NULL;
    }
    if (b->dev != NULL) {
        mcpwm_ll_intr_enable(b->dev, b->irq_mask, false);
        if (b->gpio_fault >= 0) {
            mcpwm_ll_fault_enable_detection(b->dev, FAULT_SIG_ID, false);
        }
    }
    b->inited = false;
}

void esp_foc_mcpwm_bridge_start(esp_foc_mcpwm_bridge_t *b)
{
    if (b == NULL || !b->inited) {
        return;
    }
    mcpwm_ll_timer_set_start_stop_command(b->dev, b->timer_id, MCPWM_TIMER_START_NO_STOP);
    if (b->sample_timer_id >= 0) {
        mcpwm_ll_timer_set_start_stop_command(b->dev, b->sample_timer_id,
                                              MCPWM_TIMER_START_NO_STOP);
    }
    b->running = true;
}

void esp_foc_mcpwm_bridge_stop(esp_foc_mcpwm_bridge_t *b)
{
    if (b == NULL || !b->inited) {
        return;
    }
    b->watch_fn = NULL;
    mcpwm_ll_timer_set_start_stop_command(b->dev, b->timer_id, MCPWM_TIMER_STOP_EMPTY);
    if (b->sample_timer_id >= 0) {
        mcpwm_ll_timer_set_start_stop_command(b->dev, b->sample_timer_id,
                                              MCPWM_TIMER_STOP_EMPTY);
    }
    b->running = false;
}

void esp_foc_mcpwm_bridge_set_duties(esp_foc_mcpwm_bridge_t *b,
                                     q16_t du, q16_t dv, q16_t dw)
{
    if (b == NULL || !b->inited) {
        return;
    }
    du = q16_clamp(du, 0, Q16_ONE);
    dv = q16_clamp(dv, 0, Q16_ONE);
    dw = q16_clamp(dw, 0, Q16_ONE);

    uint32_t peak = b->peak;
    uint32_t cu = (uint32_t)(((uint64_t)(uint32_t)du * (uint64_t)peak) >> 16);
    uint32_t cv = (uint32_t)(((uint64_t)(uint32_t)dv * (uint64_t)peak) >> 16);
    uint32_t cw = (uint32_t)(((uint64_t)(uint32_t)dw * (uint64_t)peak) >> 16);
    if (cu >= peak) {
        cu = peak - 1u;
    }
    if (cv >= peak) {
        cv = peak - 1u;
    }
    if (cw >= peak) {
        cw = peak - 1u;
    }

    mcpwm_ll_operator_set_compare_value(b->dev, 0, 0, cu);
    mcpwm_ll_operator_set_compare_value(b->dev, 1, 0, cv);
    mcpwm_ll_operator_set_compare_value(b->dev, 2, 0, cw);
}

void esp_foc_mcpwm_bridge_enable_output(esp_foc_mcpwm_bridge_t *b, bool on)
{
    if (b == NULL || b->gpio_enable < 0) {
        return;
    }
    int level;
    if (b->enable_active_low) {
        level = on ? 0 : 1;
    } else {
        level = on ? 1 : 0;
    }
    /* No ESP_LOG here — called from DMA/fault ISR on trip. */
    gpio_ll_set_level(&GPIO, b->gpio_enable, level);
}

static void timer_tez_etm(mcpwm_dev_t *dev, int timer_id, bool on)
{
    if (timer_id == 0) {
        dev->evt_en.evt_timer0_tez_en = on ? 1 : 0;
    } else if (timer_id == 1) {
        dev->evt_en.evt_timer1_tez_en = on ? 1 : 0;
    } else if (timer_id == 2) {
        dev->evt_en.evt_timer2_tez_en = on ? 1 : 0;
    }
}

void esp_foc_mcpwm_bridge_enable_tez_etm(esp_foc_mcpwm_bridge_t *b, bool on)
{
    if (b == NULL || b->dev == NULL) {
        return;
    }
    timer_tez_etm(b->dev, b->timer_id, on);
    if (b->sample_timer_id >= 0) {
        timer_tez_etm(b->dev, b->sample_timer_id, on);
    }
}

int esp_foc_mcpwm_bridge_sample_timer(const esp_foc_mcpwm_bridge_t *b)
{
    return (b->sample_timer_id >= 0) ? b->sample_timer_id : b->timer_id;
}

esp_err_t esp_foc_mcpwm_bridge_set_sample_starts(esp_foc_mcpwm_bridge_t *b, uint32_t n)
{
    ESP_RETURN_ON_FALSE(b != NULL && b->dev != NULL && b->sample_timer_id >= 0 && !b->running,
                        ESP_ERR_INVALID_STATE, TAG, "sample timer");
    ESP_RETURN_ON_FALSE(n >= 1u && n <= 2u, ESP_ERR_INVALID_ARG, TAG, "starts %lu",
                        (unsigned long)n);
    b->sample_period = 2u * b->peak / n;
    mcpwm_ll_timer_set_peak(b->dev, b->sample_timer_id, b->sample_period, false);
    mcpwm_ll_timer_update_period_at_once(b->dev, b->sample_timer_id);
    return ESP_OK;
}

void esp_foc_mcpwm_bridge_set_sample_delay(esp_foc_mcpwm_bridge_t *b, int32_t ticks)
{
    if (b == NULL || b->dev == NULL || b->sample_timer_id < 0) {
        return;
    }
    const int32_t period = (int32_t)b->sample_period;
    ticks %= period;
    if (ticks < 1) {
        ticks += period;
    }
    if (ticks >= period) {
        ticks = period - 1;
    }
    /* Reloaded to this count on PWM TEZ, it wraps through zero `ticks` later. */
    mcpwm_ll_timer_set_sync_phase_value(b->dev, b->sample_timer_id, (uint32_t)(period - ticks));
}

void esp_foc_mcpwm_bridge_enable_sample_etm(esp_foc_mcpwm_bridge_t *b, bool on)
{
    if (b == NULL || b->dev == NULL || b->sample_timer_id < 0) {
        return;
    }
    timer_tez_etm(b->dev, b->sample_timer_id, on);
}

void esp_foc_mcpwm_bridge_watch_sample(esp_foc_mcpwm_bridge_t *b, uint32_t skip,
                                       esp_foc_mcpwm_watch_fn_t fn, void *arg)
{
    if (b == NULL || b->dev == NULL || b->sample_timer_id < 0) {
        return;
    }
    b->watch_fn = NULL;
    if (fn == NULL) {
        return;
    }
    b->watch_arg = arg;
    b->watch_skip = skip;
    b->watch_fn = fn;
    const uint32_t mask = MCPWM_LL_EVENT_TIMER_EMPTY(b->sample_timer_id);
    mcpwm_ll_intr_clear_status(b->dev, mask);
    mcpwm_ll_intr_enable(b->dev, mask, true);
}

uint32_t esp_foc_mcpwm_bridge_phase_ticks(const esp_foc_mcpwm_bridge_t *b)
{
    const uint32_t c = mcpwm_ll_timer_get_count_value(b->dev, b->timer_id);
    return (mcpwm_ll_timer_get_count_direction(b->dev, b->timer_id) == MCPWM_TIMER_DIRECTION_UP)
               ? c
               : 2u * b->peak - c;
}

uint32_t esp_foc_mcpwm_bridge_tick_hz(const esp_foc_mcpwm_bridge_t *b)
{
    (void)b;
    return GROUP_CLK_HZ;
}

void esp_foc_mcpwm_bridge_trigger_soft_ost(esp_foc_mcpwm_bridge_t *b)
{
    if (b == NULL || !b->inited || b->dev == NULL) {
        return;
    }
    for (int op = 0; op < 3; op++) {
        mcpwm_ll_brake_trigger_soft_ost(b->dev, op);
    }
}

void esp_foc_mcpwm_bridge_clear_ost(esp_foc_mcpwm_bridge_t *b)
{
    if (b == NULL || !b->inited || b->dev == NULL) {
        return;
    }
    for (int op = 0; op < 3; op++) {
        mcpwm_ll_brake_clear_ost(b->dev, op);
    }
}

bool esp_foc_mcpwm_bridge_fault_gpio_active(const esp_foc_mcpwm_bridge_t *b)
{
    if (b == NULL || b->gpio_fault < 0) {
        return false;
    }
    int level = gpio_ll_get_level(&GPIO, b->gpio_fault);
    if (b->fault_active_high) {
        return level != 0;
    }
    return level == 0;
}

bool esp_foc_mcpwm_bridge_ost_active(const esp_foc_mcpwm_bridge_t *b)
{
    if (b == NULL || !b->inited || b->dev == NULL) {
        return false;
    }
    for (int op = 0; op < 3; op++) {
        if (mcpwm_ll_ost_brake_active(b->dev, op)) {
            return true;
        }
    }
    return false;
}
