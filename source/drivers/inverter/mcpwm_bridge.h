/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "soc/mcpwm_struct.h"
#include "espFoC/drivers/esp_foc_inverter.h"
#include "espFoC/utils/esp_foc_q16.h"

typedef void (*esp_foc_mcpwm_isr_fn_t)(void *arg);
/* Returns true once it is done watching. */
typedef bool (*esp_foc_mcpwm_watch_fn_t)(void *arg);
typedef void (*esp_foc_mcpwm_fault_fn_t)(void *arg, uint32_t intr_status);

typedef struct {
    int group_id;
    int timer_id;
    int gpio_uh, gpio_ul, gpio_vh, gpio_vl, gpio_wh, gpio_wl;
    int gpio_enable;
    bool enable_active_low;
    int gpio_fault;           /* < 0 unused */
    bool fault_active_high;
    uint32_t pwm_hz;
    uint32_t deadtime_ns;
    /* Second timer whose TEZ starts the ADC block; < 0 → the PWM timer's TEZ. */
    int sample_timer_id;
    esp_foc_mcpwm_isr_fn_t isr_cb;
    void *isr_arg;
    esp_foc_mcpwm_fault_fn_t fault_cb;
    void *fault_arg;
} esp_foc_mcpwm_bridge_cfg_t;

typedef struct {
    bool inited;
    bool running;
    int group_id;
    int timer_id;
    int sample_timer_id;
    uint32_t sample_period;
    esp_foc_mcpwm_watch_fn_t volatile watch_fn;
    void *watch_arg;
    volatile uint32_t watch_skip;
    uint32_t peak;
    uint32_t pwm_hz;
    int gpio_enable;
    bool enable_active_low;
    int gpio_fault;
    bool fault_active_high;
    uint32_t irq_mask;
    mcpwm_dev_t *dev;
    intr_handle_t intr;
    esp_foc_mcpwm_isr_fn_t isr_cb;
    void *isr_arg;
    esp_foc_mcpwm_fault_fn_t fault_cb;
    void *fault_arg;
} esp_foc_mcpwm_bridge_t;

esp_err_t esp_foc_mcpwm_bridge_init(esp_foc_mcpwm_bridge_t *b,
                                    const esp_foc_mcpwm_bridge_cfg_t *cfg);
void esp_foc_mcpwm_bridge_deinit(esp_foc_mcpwm_bridge_t *b);
void esp_foc_mcpwm_bridge_start(esp_foc_mcpwm_bridge_t *b);
void esp_foc_mcpwm_bridge_stop(esp_foc_mcpwm_bridge_t *b);
void esp_foc_mcpwm_bridge_set_duties(esp_foc_mcpwm_bridge_t *b,
                                     q16_t du, q16_t dv, q16_t dw);
void esp_foc_mcpwm_bridge_enable_output(esp_foc_mcpwm_bridge_t *b, bool on);
void esp_foc_mcpwm_bridge_enable_tez_etm(esp_foc_mcpwm_bridge_t *b, bool on);
/* Timer whose TEZ is the ADC start event. */
int esp_foc_mcpwm_bridge_sample_timer(const esp_foc_mcpwm_bridge_t *b);
/* ADC starts per PWM period (1 or 2), evenly spaced. Timers stopped. */
esp_err_t esp_foc_mcpwm_bridge_set_sample_starts(esp_foc_mcpwm_bridge_t *b, uint32_t n);
/* An ADC start this many group-clock ticks after the PWM TEZ, taken modulo the
 * spacing of the starts. */
void esp_foc_mcpwm_bridge_set_sample_delay(esp_foc_mcpwm_bridge_t *b, int32_t ticks);
/* Gate the sample timer's ETM event alone. */
void esp_foc_mcpwm_bridge_enable_sample_etm(esp_foc_mcpwm_bridge_t *b, bool on);
/*
 * Call `fn` from the bridge ISR at each sample-timer TEZ (each ADC start instant,
 * whether or not its ETM event is gated), after letting `skip` of them go by,
 * until it returns true; fn == NULL cancels. Task or ISR context; stop() cancels
 * too.
 */
void esp_foc_mcpwm_bridge_watch_sample(esp_foc_mcpwm_bridge_t *b, uint32_t skip,
                                       esp_foc_mcpwm_watch_fn_t fn, void *arg);
/* Group-clock ticks since the last PWM TEZ, in [0, 2*peak). */
uint32_t esp_foc_mcpwm_bridge_phase_ticks(const esp_foc_mcpwm_bridge_t *b);
uint32_t esp_foc_mcpwm_bridge_tick_hz(const esp_foc_mcpwm_bridge_t *b);
void esp_foc_mcpwm_bridge_trigger_soft_ost(esp_foc_mcpwm_bridge_t *b);
void esp_foc_mcpwm_bridge_clear_ost(esp_foc_mcpwm_bridge_t *b);
bool esp_foc_mcpwm_bridge_fault_gpio_active(const esp_foc_mcpwm_bridge_t *b);
bool esp_foc_mcpwm_bridge_ost_active(const esp_foc_mcpwm_bridge_t *b);
