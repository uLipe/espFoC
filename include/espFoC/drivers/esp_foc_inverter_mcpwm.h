/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "espFoC/drivers/esp_foc_inverter.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Where the current shunts sit. Inline phase sensing sees the winding current at
 * any instant and is sampled on the zero-vector centres (TEZ and the timer
 * peak), where the switching ripple crosses its period mean and no edge falls.
 *
 * Low-side shunts only carry current while their low FET conducts, which bounds
 * the sample window by duty and needs a per-sector choice of legs; that is not
 * implemented, and init refuses it rather than return currents that are wrong
 * at high modulation.
 */
typedef enum {
    ESP_FOC_SENSE_INLINE = 0,
    ESP_FOC_SENSE_LOW_SIDE,
} esp_foc_sense_topology_t;

/** ESP MCPWM + ADC-DMA + ETM inverter configuration (float only here). */
typedef struct {
    int gpio_uh;
    int gpio_ul;
    int gpio_vh;
    int gpio_vl;
    int gpio_wh;
    int gpio_wl;
    int gpio_enable;      /* -1 unused; >=0 active-high; <=-2 => |gpio| active-low (ON=0) */
    uint32_t pwm_hz;
    uint32_t deadtime_ns;
    float dc_link_volts;
    float shunt_ohm;
    float amp_gain;
    uint8_t shunt_count;  /* 2 or 3 */
    esp_foc_sense_topology_t sense_topology;
    int gpio_iu;
    int gpio_iv;
    int gpio_iw;          /* required if shunt_count == 3 */
    int mcpwm_group;      /* usually 0 */
    int mcpwm_timer;      /* usually 0 */
    float i_limit_amps;   /* <= 0 disables SW i-limit */
    /*
     * Sense LPF cutoff (Butterworth 2nd order, fs = pwm_hz).
     *  0  → default 8000 Hz
     * <0  → bypass (raw = canonical)
     * >0  → clamp to (0, pwm_hz/2)
     */
    float i_filt_fc_hz;
    int gpio_fault;       /* < 0 unused */
    bool fault_active_high;
    /*
     * Low-side conduction window that enable() runs before handing the bridge
     * over, in ms. 0 → default.
     *
     * A bootstrapped high-side driver has no gate supply until its cap is
     * charged, and the only path that charges it is the low-side FET pulling the
     * phase node down. Enabling straight into mid duty asks the high side to
     * switch on a cap that has been leaking since the last disable, so the first
     * vectors come out weak and lopsided. The same window is what dumps whatever
     * the windings were still holding, through the FETs instead of the body
     * diodes and the supply. ST's MC SDK calls this the boot-cap charge state
     * and configures it in the tens of ms; TI's answer is the same one line —
     * the low side has to be on to charge the bootstrap.
     */
    uint32_t boot_charge_ms;
    /* Optional seed; invalid / zero → identity at init. */
    esp_foc_phase_map_t phase_map;
} esp_foc_inverter_mcpwm_config_t;

esp_foc_inverter_t *esp_foc_inverter_mcpwm_acquire(unsigned index);
void esp_foc_inverter_mcpwm_release(esp_foc_inverter_t *inv);

esp_err_t esp_foc_inverter_mcpwm_init(esp_foc_inverter_t *inv,
                                      const esp_foc_inverter_mcpwm_config_t *cfg);
esp_err_t esp_foc_inverter_mcpwm_deinit(esp_foc_inverter_t *inv);

/**
 * Move the current sample instants from the zero-vector centres, in ns
 * (positive = later). Bench characterisation of the sampling point; takes effect from the
 * next PWM period.
 */
esp_err_t esp_foc_inverter_mcpwm_set_sample_shift_ns(esp_foc_inverter_t *inv,
                                                     int32_t shift_ns);

/**
 * Last ADC block, one entry per conversion in sampling order: amps with the
 * offset removed, before phase map and filter, and the sense slot each belongs
 * to. Only consistent from the DMA callback. Returns the number written.
 */
uint8_t esp_foc_inverter_mcpwm_peek_conversions(esp_foc_inverter_t *inv,
                                                q16_t *amps, uint8_t *slot,
                                                uint8_t max);

/** Nominal sample instant of conversion `i` of a block, in ns from the timer
 * peak, before any sample shift. */
int32_t esp_foc_inverter_mcpwm_conv_offset_ns(esp_foc_inverter_t *inv, uint8_t i);

#ifdef __cplusplus
}
#endif
