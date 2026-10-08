/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#include "esp_foc_driver_soc_gate.h"

#include <string.h>

#include "esp_check.h"
#include "esp_cpu.h"
#include "esp_log.h"
#include "esp_macros.h"
#include "esp_rom_sys.h"

#include "espFoC/drivers/esp_foc_inverter_mcpwm.h"
#include "espFoC/debug/esp_foc_trace.h"
#include "espFoC/osal/esp_foc_osal.h"
#include "espFoC/utils/esp_foc_iir.h"
#include "mcpwm_bridge.h"
#include "adc_dma_sense.h"
#include "etm_link.h"

#ifndef ESP_FOC_I_FILT_FC_DEFAULT_HZ
#define ESP_FOC_I_FILT_FC_DEFAULT_HZ 8000.0f
#endif

/* Same order ST's MC SDK asks for on its boot-cap charge state. */
#ifndef ESP_FOC_BOOT_CHARGE_MS_DEFAULT
#define ESP_FOC_BOOT_CHARGE_MS_DEFAULT 20u
#endif

/* A handful of Ls/Rs, which is sub-millisecond on any machine this drives. */
#ifndef ESP_FOC_DISCHARGE_MS
#define ESP_FOC_DISCHARGE_MS 3u
#endif

/*
 * Longest streak of bit-identical raw frames tolerated before the sense path is
 * declared dead. Measured on this bench with an app-side witness rather than
 * guessed: 156 frames worst case through a full identification (DC holds on a
 * parked rotor, the one place a constant reading is legitimate) and 71 through
 * a spin. 64 was tried first and tripped every identification. 512 frames is
 * 25.6 ms at 20 kHz, 3.3x the worst healthy streak, and far below the observed
 * failure which holds one value for the whole launch.
 */

#ifndef ESP_FOC_SENSE_STALE_FRAMES
#define ESP_FOC_SENSE_STALE_FRAMES 512u
#endif

static inline void trace_isr_duration(uint16_t type, uint32_t t0_cycles)
{
    uint32_t dt = esp_cpu_get_cycle_count() - t0_cycles;
    uint32_t tpus = esp_rom_get_cpu_ticks_per_us();
    uint32_t us = (tpus > 0u) ? (dt / tpus) : 0u;
    esp_foc_trace_push(type, (int32_t)us, (int32_t)dt);
}

static const char *TAG = "foc_inv_mcpwm";

typedef struct {
    esp_foc_inverter_t iface;
    bool acquired;
    bool inited;
    q16_t dc_link_q16;
    uint32_t pwm_hz;
    q16_t i_limit_q16;
    bool i_limit_en;
    bool i_limit_ready;
    volatile bool faulted;
    volatile bool enabled;
    esp_foc_fault_reason_t fault_reason;
    bool duty_ign_traced;
    uint32_t boot_charge_ms;
    esp_foc_phase_map_t phase_map;
    esp_foc_inverter_cb_t pwm_cb;
    void *pwm_arg;
    esp_foc_inverter_cb_t dma_cb;
    void *dma_arg;
    esp_foc_fault_cb_t fault_cb;
    void *fault_arg;
    bool i_filt_en;
    float i_filt_fc_hz;
    esp_foc_iir_t i_filt[3];
    q16_t i_raw[3];
    q16_t i_filt_out[3];
    q16_t sense_prev[3];
    uint32_t sense_stale;
    volatile bool sense_wd_en;
    volatile bool i_sample_ready;
    int32_t sample_shift_ns;
    uint32_t sense_b_start;
    esp_foc_mcpwm_bridge_t mcpwm;
    esp_foc_adc_dma_sense_t *adc;
    esp_foc_etm_link_t etm;
} esp_foc_inverter_mcpwm_obj_t;

static esp_foc_inverter_mcpwm_obj_t s_pool[CONFIG_ESP_FOC_MAX_INVERTERS];

static esp_foc_inverter_mcpwm_obj_t *obj_from(esp_foc_inverter_t *self)
{
    return __containerof(self, esp_foc_inverter_mcpwm_obj_t, iface);
}

static q16_t q16_abs_local(q16_t v)
{
    return v < 0 ? q16_neg(v) : v;
}

static void apply_phase_map(esp_foc_inverter_mcpwm_obj_t *o, const esp_foc_phase_map_t *map)
{
    o->phase_map = *map;
}

static void hw_to_logical(const esp_foc_inverter_mcpwm_obj_t *o,
                          q16_t hw_u, q16_t hw_v, q16_t hw_w,
                          q16_t *iu, q16_t *iv, q16_t *iw)
{
    const q16_t hw[3] = {hw_u, hw_v, hw_w};
    const esp_foc_phase_map_t *m = &o->phase_map;
    q16_t logi[3];
    for (int L = 0; L < 3; L++) {
        q16_t raw = hw[m->pwm_to_hw[L]];
        logi[L] = (m->i_sign[L] < 0) ? q16_neg(raw) : raw;
    }
    if (iu != NULL) {
        *iu = logi[0];
    }
    if (iv != NULL) {
        *iv = logi[1];
    }
    if (iw != NULL) {
        *iw = logi[2];
    }
}

static void trip(esp_foc_inverter_mcpwm_obj_t *o, esp_foc_fault_reason_t reason, bool hw_ost)
{
    if (o->faulted) {
        esp_foc_trace_push(ESP_FOC_TRACE_FAULT_TRIP_IGN, (int32_t)reason, (int32_t)o->fault_reason);
        return;
    }

    o->faulted = true;
    o->fault_reason = reason;
    o->duty_ign_traced = false;

    esp_foc_trace_push(ESP_FOC_TRACE_FAULT_TRIP, (int32_t)reason, 1);

    esp_foc_mcpwm_bridge_set_duties(&o->mcpwm, Q16_HALF, Q16_HALF, Q16_HALF);
    if (!hw_ost) {
        esp_foc_mcpwm_bridge_trigger_soft_ost(&o->mcpwm);
    }
    esp_foc_trace_push(ESP_FOC_TRACE_FAULT_OST, (int32_t)reason, hw_ost ? 1 : 0);

    esp_foc_mcpwm_bridge_enable_output(&o->mcpwm, false);
    esp_foc_trace_push(ESP_FOC_TRACE_FAULT_EN_OFF, (int32_t)reason, o->mcpwm.gpio_enable);

    /* Freeze TEZ/DMA so the fault ring is not overwritten at 20 kHz. */
    o->enabled = false;
    esp_foc_adc_dma_sense_disarm(o->adc);
    esp_foc_mcpwm_bridge_stop(&o->mcpwm);
    esp_foc_etm_link_enable(&o->etm, false);
    esp_foc_mcpwm_bridge_enable_tez_etm(&o->mcpwm, false);

    esp_foc_trace_push(ESP_FOC_TRACE_FAULT_CB, (int32_t)reason, 0);
    if (o->fault_cb != NULL) {
        o->fault_cb(o->fault_arg, reason);
    }
}

static void on_tez(void *arg)
{
    esp_foc_inverter_mcpwm_obj_t *o = (esp_foc_inverter_mcpwm_obj_t *)arg;
    uint32_t t0 = esp_cpu_get_cycle_count();
    esp_foc_trace_push(ESP_FOC_TRACE_TEZ_ENTER, 0, 0);
    if (o->pwm_cb != NULL) {
        o->pwm_cb(o->pwm_arg);
    }
    /* a = µs, b = CPU cycles (includes pwm_cb + TEZ_ENTER push). */
    trace_isr_duration(ESP_FOC_TRACE_TEZ_EXIT, t0);
}

static void reset_i_filters(esp_foc_inverter_mcpwm_obj_t *o)
{
    for (int i = 0; i < 3; i++) {
        esp_foc_iir_reset(&o->i_filt[i]);
        o->i_raw[i] = 0;
        o->i_filt_out[i] = 0;
    }
}

static void on_dma(void *arg)
{
    esp_foc_inverter_mcpwm_obj_t *o = (esp_foc_inverter_mcpwm_obj_t *)arg;
    q16_t hw_u = 0;
    q16_t hw_v = 0;
    q16_t hw_w = 0;
    q16_t iu = 0;
    q16_t iv = 0;
    q16_t iw = 0;

    esp_foc_adc_dma_sense_peek(o->adc, &hw_u, &hw_v, &hw_w);
    hw_to_logical(o, hw_u, hw_v, hw_w, &iu, &iv, &iw);
    o->i_raw[0] = iu;
    o->i_raw[1] = iv;
    o->i_raw[2] = iw;
    if (o->i_filt_en) {
        o->i_filt_out[0] = esp_foc_iir_update(&o->i_filt[0], iu);
        o->i_filt_out[1] = esp_foc_iir_update(&o->i_filt[1], iv);
        o->i_filt_out[2] = esp_foc_iir_update(&o->i_filt[2], iw);
    } else {
        o->i_filt_out[0] = iu;
        o->i_filt_out[1] = iv;
        o->i_filt_out[2] = iw;
    }

    if (o->enabled && o->i_limit_en && o->i_limit_ready && !o->faulted) {
        q16_t peak = q16_abs_local(o->i_filt_out[0]);
        q16_t av = q16_abs_local(o->i_filt_out[1]);
        q16_t aw = q16_abs_local(o->i_filt_out[2]);
        if (av > peak) {
            peak = av;
        }
        if (aw > peak) {
            peak = aw;
        }
        if (peak > o->i_limit_q16) {
            esp_foc_trace_push(ESP_FOC_TRACE_FAULT_ILIMIT_HIT, (int32_t)ESP_FOC_FAULT_ILIMIT, peak);
            trip(o, ESP_FOC_FAULT_ILIMIT, false);
        }
    }

    /*
     * Sits beside the current limit because it covers the limit's blind spot: a
     * frozen frame is a constant, and a constant never exceeds a threshold, so
     * the loops rail into the modulation ceiling with protection watching a
     * still picture. Compared on the pre-map, pre-filter values — as close to
     * the converter as this callback gets.
     *
     * Armed by the caller, because a constant reading is only evidence of a dead
     * sense when the current was supposed to be moving, and that is not something
     * the driver can know. It judged every enabled window once, and condemned a
     * healthy converter for answering correctly — measured 2026-09-14: the map
     * stage sat 50 ms at mid duty after every calibrate and faulted in 3 runs out
     * of 4 with frames still arriving and all three legs reading 0.0000 A, and
     * identification held one value for 1377 frames.
     *
     * Bit-exactness is only decisive because a spinning machine cannot hold one
     * value: rotation modulates the current whatever the regulators do. Standing
     * still it decides nothing, and it degrades as the converter gets quieter —
     * the worst healthy streak was 156 frames on 2026-08-28 and 125 on 08-30, on
     * the bench that now reaches 1377 with no software change between.
     */
    if (o->enabled && o->i_limit_ready && o->sense_wd_en && !o->faulted) {
        if ((hw_u == o->sense_prev[0]) && (hw_v == o->sense_prev[1]) &&
            (hw_w == o->sense_prev[2])) {
            if (o->sense_stale < UINT32_MAX) {
                o->sense_stale++;
            }
            if (o->sense_stale >= ESP_FOC_SENSE_STALE_FRAMES) {
                esp_foc_trace_push(ESP_FOC_TRACE_FAULT_ILIMIT_HIT,
                                   (int32_t)ESP_FOC_FAULT_SENSE_STALE,
                                   (int32_t)o->sense_stale);
                trip(o, ESP_FOC_FAULT_SENSE_STALE, false);
            }
        } else {
            o->sense_stale = 0u;
        }
        o->sense_prev[0] = hw_u;
        o->sense_prev[1] = hw_v;
        o->sense_prev[2] = hw_w;
    }

    o->i_sample_ready = true;
    if (o->dma_cb != NULL) {
        o->dma_cb(o->dma_arg);
    }
}

static void on_gpio_fault(void *arg, uint32_t intr_status)
{
    esp_foc_inverter_mcpwm_obj_t *o = (esp_foc_inverter_mcpwm_obj_t *)arg;
    esp_foc_trace_push(ESP_FOC_TRACE_FAULT_GPIO_IRQ, (int32_t)ESP_FOC_FAULT_GPIO, (int32_t)intr_status);
    trip(o, ESP_FOC_FAULT_GPIO, true);
}

static void api_set_pwm_callback(esp_foc_inverter_t *self,
                                 esp_foc_inverter_cb_t cb,
                                 void *arg)
{
    esp_foc_inverter_mcpwm_obj_t *o = obj_from(self);
    o->pwm_cb = cb;
    o->pwm_arg = arg;
}

static void api_set_dma_callback(esp_foc_inverter_t *self,
                                 esp_foc_inverter_cb_t cb,
                                 void *arg)
{
    esp_foc_inverter_mcpwm_obj_t *o = obj_from(self);
    o->dma_cb = cb;
    o->dma_arg = arg;
}

static void api_set_fault_callback(esp_foc_inverter_t *self,
                                   esp_foc_fault_cb_t cb,
                                   void *arg)
{
    esp_foc_inverter_mcpwm_obj_t *o = obj_from(self);
    o->fault_cb = cb;
    o->fault_arg = arg;
}

/*
 * Arm through a low-side window instead of straight into mid duty.
 *
 * Two things are wrong with enabling at 50 %. The high-side gate supply is a
 * bootstrap cap that only charges while the low-side FET holds the phase node
 * down, and it has been leaking for however long the bridge sat disabled — so
 * the first high-side pulses are weak and the vector that comes out is not the
 * one that was asked for. And anything the windings were still holding gets
 * commutated into the body diodes and pushed back at the supply.
 *
 * Zero duty puts all three low sides on, which charges the caps and gives the
 * stored energy a path through the FETs. Ls/Rs is under a millisecond; the
 * window is sized for the caps, not the winding.
 *
 * Deliberately not symmetric with trip(): that path cuts the output immediately
 * because protection cannot wait for a discharge.
 */
/*
 * Two starts per period alternate the legs by count, so which leg lands on TEZ is
 * set by the first start after the frame restarts. With the gate closed, watch
 * the start instants: right after B's, restart the frame and open the gate, a
 * full 25 µs ahead of A's whatever the shift. Opening from TEZ or the peak
 * instead races a start for some shifts, because both are 25 µs apart like the
 * starts themselves.
 */
static bool sense_open(void *arg)
{
    esp_foc_inverter_mcpwm_obj_t *o = (esp_foc_inverter_mcpwm_obj_t *)arg;
    const uint32_t period = 2u * o->mcpwm.peak;
    const uint32_t since_b = (esp_foc_mcpwm_bridge_phase_ticks(&o->mcpwm) + period -
                              o->sense_b_start) % period;
    if (since_b >= o->mcpwm.peak / 2u) {
        return false;
    }
    esp_foc_adc_dma_sense_restart(o->adc);
    esp_foc_mcpwm_bridge_enable_sample_etm(&o->mcpwm, true);
    return true;
}

static void sense_place_first_start(esp_foc_inverter_mcpwm_obj_t *o)
{
    if (esp_foc_adc_dma_sense_starts(o->adc) < 2u) {
        return;
    }
    const int64_t tick_hz = (int64_t)esp_foc_mcpwm_bridge_tick_hz(&o->mcpwm);
    const int64_t peak = (int64_t)o->mcpwm.peak;
    const int64_t period = 2 * peak;
    const int64_t lead = ((int64_t)esp_foc_adc_dma_sense_lead_ns(o->adc) * tick_hz) / 1000000000LL;
    const int64_t shift = ((int64_t)o->sample_shift_ns * tick_hz) / 1000000000LL;
    esp_foc_mcpwm_bridge_enable_sample_etm(&o->mcpwm, false);
    o->sense_b_start = (uint32_t)((((shift - lead - peak) % period) + period) % period);
    /* One start let by first, so nothing gated before is still converting. */
    esp_foc_mcpwm_bridge_watch_sample(&o->mcpwm, 1u, sense_open, o);
}

static esp_err_t api_enable(esp_foc_inverter_t *self)
{
    esp_foc_inverter_mcpwm_obj_t *o = obj_from(self);
    if (o->faulted) {
        esp_foc_trace_push(ESP_FOC_TRACE_FAULT_ENABLE_REJ, (int32_t)o->fault_reason, 0);
        return ESP_ERR_INVALID_STATE;
    }
    reset_i_filters(o);
    /* Or the first frame of this session is compared against the last frame of
     * the previous one, which is exactly the pair most likely to match. */
    o->sense_stale = 0u;
    /* A new session is not covered by the previous session's decision. */
    o->sense_wd_en = false;
    o->sense_prev[0] = INT32_MIN;
    o->sense_prev[1] = INT32_MIN;
    o->sense_prev[2] = INT32_MIN;
    esp_foc_mcpwm_bridge_clear_ost(&o->mcpwm);

    esp_foc_mcpwm_bridge_enable_output(&o->mcpwm, false);
    esp_foc_mcpwm_bridge_set_duties(&o->mcpwm, 0, 0, 0);
    esp_foc_mcpwm_bridge_start(&o->mcpwm);
    esp_foc_mcpwm_bridge_enable_output(&o->mcpwm, true);
    esp_foc_sleep_ms(o->boot_charge_ms);
    /*
     * Then settle at mid duty before the sense comes up. Callers zero the shunts
     * as their first act after enable, and an offset averaged over the tail of
     * the low-side window is a false current the d regulator will rail against —
     * measured as id = -3.7 A with vd on the ceiling, and an ILIM trip behind it.
     */
    esp_foc_mcpwm_bridge_set_duties(&o->mcpwm, Q16_HALF, Q16_HALF, Q16_HALF);
    esp_foc_sleep_ms(o->boot_charge_ms);

    esp_foc_mcpwm_bridge_enable_tez_etm(&o->mcpwm, true);
    if (esp_foc_adc_dma_sense_starts(o->adc) > 1u) {
        esp_foc_mcpwm_bridge_enable_sample_etm(&o->mcpwm, false);
    }
    esp_foc_etm_link_enable(&o->etm, true);
    (void)esp_foc_adc_dma_sense_arm(o->adc);
    sense_place_first_start(o);
    o->enabled = true;
    return ESP_OK;
}

/*
 * Hand the winding current to the low-side FETs before the output goes away.
 *
 * Cutting EN at mid duty leaves the phases floating with energy still in them,
 * and it leaves through the body diodes and into the supply — the spike an
 * operator sees on the bench, and the reason the bootstrap caps then sit with no
 * conduction path to recharge. SimpleFOC does it in this order for the same
 * reason: setPwm(0,0,0) and then disable().
 *
 * The window is sized for the inductor, not for the caps: Ls/Rs is well under a
 * millisecond, so a few of them dumps what is stored. Keeping it short is also
 * what bounds the other current in play — a shaft that has not fully stopped
 * sees three shorted legs and brakes into them, and that current builds on the
 * same time constant.
 *
 * A faulted bridge skips it. The OST has already taken the output and set_duties
 * is refused in that state, so there is nothing left to hand over.
 */
static void api_disable(esp_foc_inverter_t *self)
{
    esp_foc_inverter_mcpwm_obj_t *o = obj_from(self);
    if (!o->faulted) {
        esp_foc_mcpwm_bridge_set_duties(&o->mcpwm, 0, 0, 0);
        esp_foc_sleep_ms(ESP_FOC_DISCHARGE_MS);
    }
    o->enabled = false;
    esp_foc_adc_dma_sense_disarm(o->adc);
    esp_foc_mcpwm_bridge_stop(&o->mcpwm);
    esp_foc_mcpwm_bridge_enable_output(&o->mcpwm, false);
    esp_foc_etm_link_enable(&o->etm, false);
    esp_foc_mcpwm_bridge_enable_tez_etm(&o->mcpwm, false);
}

static void api_set_duties(esp_foc_inverter_t *self,
                           q16_t duty_u, q16_t duty_v, q16_t duty_w)
{
    esp_foc_inverter_mcpwm_obj_t *o = obj_from(self);
    if (o->faulted) {
        if (!o->duty_ign_traced) {
            o->duty_ign_traced = true;
            esp_foc_trace_push(ESP_FOC_TRACE_FAULT_DUTY_IGN, (int32_t)o->fault_reason, 0);
        }
        return;
    }
    const q16_t logical[3] = {duty_u, duty_v, duty_w};
    q16_t hw[3] = {Q16_HALF, Q16_HALF, Q16_HALF};
    for (int L = 0; L < 3; L++) {
        hw[o->phase_map.pwm_to_hw[L]] = logical[L];
    }
    esp_foc_mcpwm_bridge_set_duties(&o->mcpwm, hw[0], hw[1], hw[2]);
    esp_foc_trace_push(ESP_FOC_TRACE_DUTY, duty_u, duty_v);
}

static q16_t api_get_dc_link(esp_foc_inverter_t *self)
{
    return obj_from(self)->dc_link_q16;
}

static uint32_t api_get_pwm_rate(esp_foc_inverter_t *self)
{
    return obj_from(self)->pwm_hz;
}

static void api_fetch(esp_foc_inverter_t *self, q16_t *iu, q16_t *iv, q16_t *iw)
{
    esp_foc_inverter_mcpwm_obj_t *o = obj_from(self);
    q16_t discard_u = 0;
    q16_t discard_v = 0;
    q16_t discard_w = 0;
    /* Drain ADC ready flag; canonical values are the LPF latches. */
    esp_foc_adc_dma_sense_fetch(o->adc, &discard_u, &discard_v, &discard_w);
    o->i_sample_ready = false;
    if (iu != NULL) {
        *iu = o->i_filt_out[0];
    }
    if (iv != NULL) {
        *iv = o->i_filt_out[1];
    }
    if (iw != NULL) {
        *iw = o->i_filt_out[2];
    }
}

static void api_fetch_raw(esp_foc_inverter_t *self, q16_t *iu, q16_t *iv, q16_t *iw)
{
    esp_foc_inverter_mcpwm_obj_t *o = obj_from(self);
    if (iu != NULL) {
        *iu = o->i_raw[0];
    }
    if (iv != NULL) {
        *iv = o->i_raw[1];
    }
    if (iw != NULL) {
        *iw = o->i_raw[2];
    }
}

static bool api_sample_ready(esp_foc_inverter_t *self)
{
    return obj_from(self)->i_sample_ready;
}

static void api_calibrate(esp_foc_inverter_t *self, int rounds)
{
    esp_foc_inverter_mcpwm_obj_t *o = obj_from(self);
    o->i_limit_ready = false;
    if (rounds > 0) {
        esp_foc_adc_dma_sense_calibrate(o->adc, rounds);
    }
    reset_i_filters(o);
    o->i_limit_ready = o->i_limit_en;
}

static void api_set_sense_watchdog(esp_foc_inverter_t *self, bool enable)
{
    esp_foc_inverter_mcpwm_obj_t *o = obj_from(self);
    o->sense_stale = 0u;
    o->sense_prev[0] = INT32_MIN;
    o->sense_prev[1] = INT32_MIN;
    o->sense_prev[2] = INT32_MIN;
    o->sense_wd_en = enable;
}

static void api_soft_trip(esp_foc_inverter_t *self)
{
    esp_foc_inverter_mcpwm_obj_t *o = obj_from(self);
    esp_foc_trace_push(ESP_FOC_TRACE_FAULT_SOFT_REQ, (int32_t)ESP_FOC_FAULT_SOFT_TRIP, 0);
    trip(o, ESP_FOC_FAULT_SOFT_TRIP, false);
}

static esp_err_t api_clear_fault(esp_foc_inverter_t *self)
{
    esp_foc_inverter_mcpwm_obj_t *o = obj_from(self);
    esp_foc_fault_reason_t prev = o->fault_reason;

    esp_foc_trace_push(ESP_FOC_TRACE_FAULT_CLEAR_REQ, (int32_t)prev, 0);

    if (!o->faulted) {
        esp_foc_trace_push(ESP_FOC_TRACE_FAULT_CLEAR_REJ, (int32_t)prev, (int32_t)ESP_ERR_INVALID_STATE);
        return ESP_ERR_INVALID_STATE;
    }

    if (esp_foc_mcpwm_bridge_fault_gpio_active(&o->mcpwm)) {
        esp_foc_trace_push(ESP_FOC_TRACE_FAULT_CLEAR_REJ, (int32_t)prev, (int32_t)ESP_ERR_INVALID_STATE);
        return ESP_ERR_INVALID_STATE;
    }

    esp_foc_mcpwm_bridge_clear_ost(&o->mcpwm);
    o->faulted = false;
    o->fault_reason = ESP_FOC_FAULT_NONE;
    o->duty_ign_traced = false;
    reset_i_filters(o);
    esp_foc_trace_push(ESP_FOC_TRACE_FAULT_CLEAR_OK, (int32_t)prev, 0);
    return ESP_OK;
}

static bool api_is_faulted(esp_foc_inverter_t *self)
{
    return obj_from(self)->faulted;
}

static esp_foc_fault_reason_t api_get_fault_reason(esp_foc_inverter_t *self)
{
    return obj_from(self)->fault_reason;
}

static esp_err_t api_set_phase_map(esp_foc_inverter_t *self, const esp_foc_phase_map_t *map)
{
    esp_foc_inverter_mcpwm_obj_t *o = obj_from(self);
    if (map == NULL || !esp_foc_phase_map_valid(map)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (o->enabled) {
        return ESP_ERR_INVALID_STATE;
    }
    apply_phase_map(o, map);
    return ESP_OK;
}

static void api_get_phase_map(esp_foc_inverter_t *self, esp_foc_phase_map_t *map)
{
    if (map == NULL) {
        return;
    }
    *map = obj_from(self)->phase_map;
}

/*
 * Place the starts so the anchor sample lands on the timer peak, the zero vector
 * inside one duty period (compares reload on TEZ); with two starts per period
 * the other one lands on TEZ.
 */
static void apply_sample_delay(esp_foc_inverter_mcpwm_obj_t *o)
{
    const int64_t tick_hz = (int64_t)esp_foc_mcpwm_bridge_tick_hz(&o->mcpwm);
    const int64_t lead_ns = (int64_t)esp_foc_adc_dma_sense_lead_ns(o->adc) -
                            (int64_t)o->sample_shift_ns;
    const int64_t ticks = (int64_t)o->mcpwm.peak - (lead_ns * tick_hz) / 1000000000LL;
    esp_foc_mcpwm_bridge_set_sample_delay(&o->mcpwm, (int32_t)ticks);
}

static void bind_vtable(esp_foc_inverter_mcpwm_obj_t *o)
{
    o->iface.set_pwm_callback = api_set_pwm_callback;
    o->iface.set_dma_callback = api_set_dma_callback;
    o->iface.set_fault_callback = api_set_fault_callback;
    o->iface.enable = api_enable;
    o->iface.disable = api_disable;
    o->iface.set_duties = api_set_duties;
    o->iface.get_dc_link_voltage = api_get_dc_link;
    o->iface.get_pwm_rate_hz = api_get_pwm_rate;
    o->iface.fetch_currents = api_fetch;
    o->iface.fetch_currents_raw = api_fetch_raw;
    o->iface.sample_ready = api_sample_ready;
    o->iface.calibrate_currents = api_calibrate;
    o->iface.set_sense_watchdog = api_set_sense_watchdog;
    o->iface.soft_trip = api_soft_trip;
    o->iface.clear_fault = api_clear_fault;
    o->iface.is_faulted = api_is_faulted;
    o->iface.get_fault_reason = api_get_fault_reason;
    o->iface.set_phase_map = api_set_phase_map;
    o->iface.get_phase_map = api_get_phase_map;
}

esp_foc_inverter_t *esp_foc_inverter_mcpwm_acquire(unsigned index)
{
    if (index >= CONFIG_ESP_FOC_MAX_INVERTERS) {
        return NULL;
    }
    esp_foc_inverter_mcpwm_obj_t *o = &s_pool[index];
    if (o->acquired) {
        return NULL;
    }
    memset(o, 0, sizeof(*o));
    o->acquired = true;
    bind_vtable(o);
    return &o->iface;
}

void esp_foc_inverter_mcpwm_release(esp_foc_inverter_t *inv)
{
    if (inv == NULL) {
        return;
    }
    esp_foc_inverter_mcpwm_obj_t *o = obj_from(inv);
    if (o->inited) {
        (void)esp_foc_inverter_mcpwm_deinit(inv);
    }
    o->acquired = false;
}

esp_err_t esp_foc_inverter_mcpwm_init(esp_foc_inverter_t *inv,
                                      const esp_foc_inverter_mcpwm_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(inv != NULL && cfg != NULL, ESP_ERR_INVALID_ARG, TAG, "null");
    esp_foc_inverter_mcpwm_obj_t *o = obj_from(inv);
    ESP_RETURN_ON_FALSE(o->acquired && !o->inited, ESP_ERR_INVALID_STATE, TAG, "state");
    ESP_RETURN_ON_FALSE(cfg->shunt_count >= 2 && cfg->shunt_count <= 3, ESP_ERR_INVALID_ARG, TAG, "shunts");
    ESP_RETURN_ON_FALSE(cfg->sense_topology == ESP_FOC_SENSE_INLINE, ESP_ERR_NOT_SUPPORTED,
                        TAG, "sense topology %d", (int)cfg->sense_topology);

    (void)esp_foc_trace_init();

    float vdc = cfg->dc_link_volts;
    if (vdc <= 0.0f) {
        vdc = 12.0f;
    }
    o->dc_link_q16 = q16_from_float(vdc);
    o->pwm_hz = cfg->pwm_hz != 0 ? cfg->pwm_hz : CONFIG_ESP_FOC_PWM_RATE_HZ;
    o->i_limit_en = cfg->i_limit_amps > 0.0f;
    o->i_limit_q16 = o->i_limit_en ? q16_from_float(cfg->i_limit_amps) : 0;
    /* Armed after calibrate_currents() (rounds>0) or arm-only calibrate(0). */
    o->i_limit_ready = false;
    o->sense_wd_en = false;
    o->boot_charge_ms = cfg->boot_charge_ms != 0u ? cfg->boot_charge_ms
                                                  : ESP_FOC_BOOT_CHARGE_MS_DEFAULT;
    o->enabled = false;
    o->faulted = false;
    o->fault_reason = ESP_FOC_FAULT_NONE;
    o->duty_ign_traced = false;

    o->i_filt_en = !(cfg->i_filt_fc_hz < 0.0f);
    if (!o->i_filt_en) {
        o->i_filt_fc_hz = 0.0f;
    } else {
        float fc = cfg->i_filt_fc_hz;
        if (!(fc > 0.0f)) {
            fc = ESP_FOC_I_FILT_FC_DEFAULT_HZ;
        }
        float fc_max = 0.49f * (float)o->pwm_hz;
        if (fc >= fc_max) {
            fc = fc_max;
        }
        o->i_filt_fc_hz = fc;
        for (int i = 0; i < 3; i++) {
            esp_err_t ferr = esp_foc_iir_design_lpf(&o->i_filt[i],
                                                    (float)o->pwm_hz,
                                                    o->i_filt_fc_hz);
            if (ferr != ESP_OK) {
                return ferr;
            }
        }
    }
    reset_i_filters(o);

    if (esp_foc_phase_map_valid(&cfg->phase_map)) {
        apply_phase_map(o, &cfg->phase_map);
    } else {
        esp_foc_phase_map_t id;
        esp_foc_phase_map_identity(&id);
        apply_phase_map(o, &id);
    }

    int en_gpio = cfg->gpio_enable;
    bool en_low = false;
    if (en_gpio < -1) {
        en_low = true;
        en_gpio = -en_gpio;
    } else if (en_gpio < 0) {
        en_gpio = -1;
    }

    esp_foc_mcpwm_bridge_cfg_t mcpwm_cfg = {
        .group_id = cfg->mcpwm_group,
        .timer_id = cfg->mcpwm_timer,
        .gpio_uh = cfg->gpio_uh,
        .gpio_ul = cfg->gpio_ul,
        .gpio_vh = cfg->gpio_vh,
        .gpio_vl = cfg->gpio_vl,
        .gpio_wh = cfg->gpio_wh,
        .gpio_wl = cfg->gpio_wl,
        .gpio_enable = en_gpio,
        .enable_active_low = en_low,
        .gpio_fault = cfg->gpio_fault,
        .fault_active_high = cfg->fault_active_high,
        .pwm_hz = o->pwm_hz,
        .deadtime_ns = cfg->deadtime_ns != 0 ? cfg->deadtime_ns : 500,
        .sample_timer_id = (cfg->mcpwm_timer + 1) % 3,
        .isr_cb = on_tez,
        .isr_arg = o,
        .fault_cb = on_gpio_fault,
        .fault_arg = o,
    };
    esp_err_t err = esp_foc_mcpwm_bridge_init(&o->mcpwm, &mcpwm_cfg);
    if (err != ESP_OK) {
        return err;
    }

    esp_foc_adc_dma_cfg_t adc_cfg = {
        .shunt_count = cfg->shunt_count,
        .gpio_iu = cfg->gpio_iu,
        .gpio_iv = cfg->gpio_iv,
        .gpio_iw = cfg->gpio_iw,
        .shunt_ohm = cfg->shunt_ohm > 0.0f ? cfg->shunt_ohm : 0.01f,
        .amp_gain = cfg->amp_gain > 0.0f ? cfg->amp_gain : 20.0f,
        .pwm_hz = o->pwm_hz,
        .on_done = on_dma,
        .on_done_arg = o,
    };
    err = esp_foc_adc_dma_sense_init(&o->adc, &adc_cfg);
    if (err != ESP_OK) {
        esp_foc_mcpwm_bridge_deinit(&o->mcpwm);
        return err;
    }

    err = esp_foc_mcpwm_bridge_set_sample_starts(&o->mcpwm, esp_foc_adc_dma_sense_starts(o->adc));
    if (err != ESP_OK) {
        esp_foc_adc_dma_sense_deinit(o->adc);
        o->adc = NULL;
        esp_foc_mcpwm_bridge_deinit(&o->mcpwm);
        return err;
    }
    o->sample_shift_ns = 0;
    apply_sample_delay(o);

    esp_foc_etm_link_cfg_t etm_cfg = {
        .mcpwm_timer = esp_foc_mcpwm_bridge_sample_timer(&o->mcpwm),
        .etm_channel = 0,
        .stop_channel = 1,
        .dma_rx_channel = ESP_FOC_ADC_DMA_RX_CH,
        .stop_per_conversion = esp_foc_adc_dma_sense_starts(o->adc) > 1u,
    };
    err = esp_foc_etm_link_init(&o->etm, &etm_cfg);
    if (err != ESP_OK) {
        esp_foc_adc_dma_sense_deinit(o->adc);
        o->adc = NULL;
        esp_foc_mcpwm_bridge_deinit(&o->mcpwm);
        return err;
    }

    esp_foc_mcpwm_bridge_set_duties(&o->mcpwm, Q16_HALF, Q16_HALF, Q16_HALF);
    o->inited = true;
    ESP_LOGI(TAG,
             "inverter ready pwm=%lu Hz shunts=%u conv=%u lead=%d ns ilim=%s ifilt=%s "
             "fc=%.0f Hz fault_gpio=%d en=%d/%s",
             (unsigned long)o->pwm_hz,
             (unsigned)cfg->shunt_count,
             (unsigned)esp_foc_adc_dma_sense_conv_count(o->adc),
             (int)esp_foc_adc_dma_sense_lead_ns(o->adc),
             o->i_limit_en ? "on" : "off",
             o->i_filt_en ? "on" : "bypass",
             (double)o->i_filt_fc_hz,
             cfg->gpio_fault,
             en_gpio,
             en_low ? "active_low" : "active_high");
    return ESP_OK;
}

esp_err_t esp_foc_inverter_mcpwm_deinit(esp_foc_inverter_t *inv)
{
    if (inv == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_foc_inverter_mcpwm_obj_t *o = obj_from(inv);
    if (!o->inited) {
        return ESP_OK;
    }
    api_disable(inv);
    esp_foc_etm_link_deinit(&o->etm);
    esp_foc_adc_dma_sense_deinit(o->adc);
    o->adc = NULL;
    esp_foc_mcpwm_bridge_deinit(&o->mcpwm);
    o->faulted = false;
    o->fault_reason = ESP_FOC_FAULT_NONE;
    o->inited = false;
    return ESP_OK;
}

esp_err_t esp_foc_inverter_mcpwm_set_sample_shift_ns(esp_foc_inverter_t *inv,
                                                     int32_t shift_ns)
{
    ESP_RETURN_ON_FALSE(inv != NULL, ESP_ERR_INVALID_ARG, TAG, "null");
    esp_foc_inverter_mcpwm_obj_t *o = obj_from(inv);
    ESP_RETURN_ON_FALSE(o->inited, ESP_ERR_INVALID_STATE, TAG, "state");
    o->sample_shift_ns = shift_ns;
    apply_sample_delay(o);
    if (o->enabled) {
        sense_place_first_start(o);
    }
    return ESP_OK;
}

uint8_t esp_foc_inverter_mcpwm_peek_conversions(esp_foc_inverter_t *inv,
                                                q16_t *amps, uint8_t *slot,
                                                uint8_t max)
{
    if (inv == NULL) {
        return 0;
    }
    esp_foc_inverter_mcpwm_obj_t *o = obj_from(inv);
    if (!o->inited) {
        return 0;
    }
    return esp_foc_adc_dma_sense_peek_conversions(o->adc, amps, slot, max);
}

int32_t esp_foc_inverter_mcpwm_conv_offset_ns(esp_foc_inverter_t *inv, uint8_t i)
{
    if (inv == NULL) {
        return 0;
    }
    esp_foc_inverter_mcpwm_obj_t *o = obj_from(inv);
    if (!o->inited) {
        return 0;
    }
    return esp_foc_adc_dma_sense_conv_offset_ns(o->adc, i);
}
