/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#include "esp_foc_driver_soc_gate.h"

#include <string.h>

#include "esp_check.h"
#include "esp_clk_tree.h"
#include "esp_cpu.h"
#include "esp_intr_alloc.h"
#include "esp_private/adc_share_hw_ctrl.h"
#include "esp_private/regi2c_ctrl.h"
#include "esp_private/sar_periph_ctrl.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "espFoC/osal/esp_foc_osal.h"
#include "hal/adc_hal.h"
#include "hal/adc_hal_common.h"
#include "hal/adc_ll.h"
#include "hal/adc_types.h"
#include "hal/dma_types.h"
#include "hal/gdma_ll.h"
#include "hal/gdma_types.h"
#include "hal/gpio_ll.h"
#include "soc/adc_channel.h"
#include "soc/clk_tree_defs.h"
#include "soc/gdma_channel.h"
#include "soc/gpio_struct.h"
#include "soc/interrupts.h"
#include "soc/soc_caps.h"

#include "adc_dma_sense.h"
#include "espFoC/debug/esp_foc_trace.h"

static const char *TAG = "foc_adc";

#define ADC_DMA_GROUP   0
#define ADC_DMA_RX_CH   ESP_FOC_ADC_DMA_RX_CH
#define ADC_MAX_SHUNTS  3
#define ADC_MAX_CONV    ESP_FOC_ADC_MAX_CONV
#define ADC_BYTES_PER   4
/* Above the LSB rattle of the amplifier, well below anything that matters as
 * current: at this bench's scaling 16 counts is about 80 mA. */
#define ADC_OFFSET_DRIFT_COUNTS 16
/* Digital controller clock is PLL_F80M / (div + 1); the SAR and the interval
 * timer both run from it at half that rate. */
/*
 * Faster than IDF's default is not usable: from PLL_F80M undivided the SAR
 * misses bit decisions (codes up to ~20 LSB wide at multiples of 32) and the
 * sample window is too short to settle between channels (zero off by ~90 counts).
 */
#ifndef ESP_FOC_ADC_CLKM_DIV
#define ESP_FOC_ADC_CLKM_DIV ADC_LL_CLKM_DIV_NUM_DEFAULT
#endif
#define ADC_TIMER_HZ (80000000ull / (ESP_FOC_ADC_CLKM_DIV + 1ull) / 2ull)
/* The C6 converter's 12.5 µs floor (SOC_ADC_SAMPLE_FREQ_THRES_HIGH), in
 * interval-timer cycles. */
#define ADC_MIN_INTERVAL_CYCLES \
    ((uint32_t)((12500ull * ADC_TIMER_HZ + 999999999ull) / 1000000000ull))
/*
 * Block start to the first S/H is this plus half an interval, and each S/H is
 * followed by its conversion done within ESP_FOC_ADC_SAMPLE_TO_DONE_NS. The
 * converter has no event for the S/H, so it was placed against PWM edges on
 * silicon: over 12.8..32 µs intervals the first sample moved by 0.5
 * of the interval, +-0.5 µs around this constant, at the default clock. Another
 * ESP_FOC_ADC_CLKM_DIV needs both measured again.
 */
#ifndef ESP_FOC_ADC_SAMPLE_LATENCY_NS
#define ESP_FOC_ADC_SAMPLE_LATENCY_NS 4400u
#endif
#ifndef ESP_FOC_ADC_SAMPLE_TO_DONE_NS
#define ESP_FOC_ADC_SAMPLE_TO_DONE_NS 6800u
#endif
/* SAR acquisition window, 0..7 (3-bit field). */
#ifndef ESP_FOC_ADC_SAMPLE_CYCLE
#define ESP_FOC_ADC_SAMPLE_CYCLE ADC_LL_SAMPLE_CYCLE_DEFAULT
#endif

struct esp_foc_adc_dma_sense {
    bool inited;
    volatile bool armed;
    uint8_t shunt_count;
    uint8_t conv_count;
    uint8_t conv_slot[ADC_MAX_CONV];
    uint8_t conv_per_slot[ADC_MAX_SHUNTS];
    uint8_t starts;
    uint32_t interval_cycles;
    uint32_t lead_ns;
    int32_t conv_offset_ns[ADC_MAX_CONV];
    int channels[ADC_MAX_SHUNTS];
    q16_t scale_q16;
    int32_t offset_raw[ADC_MAX_SHUNTS];
    volatile q16_t iu, iv, iw;
    volatile bool sample_ready;
    volatile uint32_t bad_frames;
    esp_foc_adc_dma_done_fn_t on_done;
    void *on_done_arg;
    dma_descriptor_t desc;
    uint8_t dma_buf[ADC_MAX_CONV * ADC_BYTES_PER] __attribute__((aligned(4)));
    gdma_dev_t *gdma;
    intr_handle_t intr;
    adc_hal_dma_ctx_t hal_ctx;
};

static esp_foc_adc_dma_sense_t s_pool[CONFIG_ESP_FOC_MAX_INVERTERS];

static int gpio_to_adc1_channel(int gpio)
{
    switch (gpio) {
    case ADC1_CHANNEL_0_GPIO_NUM: return 0;
    case ADC1_CHANNEL_1_GPIO_NUM: return 1;
    case ADC1_CHANNEL_2_GPIO_NUM: return 2;
    case ADC1_CHANNEL_3_GPIO_NUM: return 3;
    case ADC1_CHANNEL_4_GPIO_NUM: return 4;
    case ADC1_CHANNEL_5_GPIO_NUM: return 5;
    case ADC1_CHANNEL_6_GPIO_NUM: return 6;
    default: return -1;
    }
}

static void gpio_as_analog(int gpio)
{
    if (gpio < 0) {
        return;
    }
    gpio_ll_input_disable(&GPIO, gpio);
    gpio_ll_output_disable(&GPIO, gpio);
    gpio_ll_pullup_dis(&GPIO, gpio);
    gpio_ll_pulldown_dis(&GPIO, gpio);
    esp_rom_gpio_pad_select_gpio((uint32_t)gpio);
}

/**
 * Which shunt a conversion belongs to, from the sample rather than its position.
 *
 * The pattern table is scanned in order, so slot 0 is the first shunt — the C6
 * has held that on every window measured here. It is read from the sample anyway
 * because the failure mode is invisible: exchanging two measured legs is a mirror
 * on the sense side alone, and a phase map moves drive and sense together, so no
 * permutation or sign it contains negates beta while leaving alpha alone. Every
 * symmetry, Kirchhoff and d-axis check still passes, and the only symptom is a q
 * axis wired for positive feedback. This bench spent a session on exactly that
 * mirror coming from the shunt pin assignment.
 */
static int shunt_of_channel(const esp_foc_adc_dma_sense_t *s, uint32_t ch)
{
    for (uint8_t i = 0; i < s->shunt_count; i++) {
        if ((uint32_t)s->channels[i] == ch) {
            return (int)i;
        }
    }
    return -1;
}

static void adc_parse_and_publish(esp_foc_adc_dma_sense_t *s)
{
    q16_t vals[ADC_MAX_SHUNTS] = {0};
    int32_t sum[ADC_MAX_SHUNTS] = {0};
    uint8_t n[ADC_MAX_SHUNTS] = {0};
    uint8_t seen = 0;
    const adc_digi_output_data_t *p = (const adc_digi_output_data_t *)s->dma_buf;
    for (uint8_t i = 0; i < s->conv_count; i++) {
        const int k = shunt_of_channel(s, p[i].type2.channel);
        if (k < 0) {
            continue;
        }
        sum[k] += (int32_t)p[i].type2.data;
        n[k]++;
    }
    for (uint8_t k = 0; k < s->shunt_count; k++) {
        if (n[k] != s->conv_per_slot[k]) {
            continue;
        }
        /* At most two conversions per slot, so the mean is a shift. */
        const int32_t delta = sum[k] - (int32_t)n[k] * s->offset_raw[k];
        const q16_t mean = (q16_t)((n[k] == 2u) ? (delta << 15) : (delta << 16));
        vals[k] = q16_mul(mean, s->scale_q16);
        seen++;
    }
    /*
     * A slot whose tag does not match any configured channel used to fall through
     * to the zero vals[] was initialised with, so a mis-tagged block published
     * 0 A on every leg — the quietest possible reading, indistinguishable from a
     * parked shaft, and the regulators wind into the modulation ceiling against
     * it. Drop the block instead and leave the last good one standing, which the
     * caller's stale detector can see.
     */
    if (seen < s->shunt_count) {
        s->bad_frames++;
        return;
    }
    s->iu = vals[0];
    s->iv = vals[1];
    if (s->shunt_count >= 3) {
        s->iw = vals[2];
    } else {
        s->iw = q16_neg(q16_add(vals[0], vals[1]));
    }
    s->sample_ready = true;
}

/*
 * Rearm the DMA for the next block. Deliberately does not touch the converter's
 * run state: the block is bounded by hardware (meas_num_limit) and restarted by
 * hardware (MCPWM TEZ through ADC_TASK_START0), so nothing here can race the
 * trigger.
 *
 * Two earlier arrangements are worth not repeating. This used to end with
 * adc_ll_digi_trigger_enable(), which is the converter's interval timer
 * (saradc_timer_en on the C6) and is how IDF's continuous driver paces sampling.
 * Re-enabling it every EOF meant sampling never stopped, so the TEZ start landed
 * on something already running and the ETM link did nothing at all — the cadence
 * was the free timer's, matching the PWM to within a couple of Hz because both
 * divide PLL_F80M. That is the worst case rather than a benign one: the phase is
 * set by whenever the timer came up and then holds for seconds, so each launch
 * gets one stable pathology (sampling parked on the switching edge, sampling
 * inside this very window with the GDMA channel stopped, or a block split across
 * the pattern pointer).
 *
 * Replacing it with a software trigger_disable() here was worse, because this ISR
 * races TEZ: a start that arrives before the disable is clobbered, no further EOF
 * is generated, and the sense is dead for the rest of the launch.
 */
static void adc_rearm(esp_foc_adc_dma_sense_t *s)
{
    gdma_ll_rx_stop(s->gdma, ADC_DMA_RX_CH);
    gdma_ll_rx_reset_channel(s->gdma, ADC_DMA_RX_CH);
    adc_hal_digi_reset();

    s->desc.dw0.size = s->conv_count * ADC_BYTES_PER;
    s->desc.dw0.length = 0;
    s->desc.dw0.err_eof = 0;
    s->desc.dw0.suc_eof = 0;
    s->desc.dw0.owner = 1;
    s->desc.buffer = s->dma_buf;
    s->desc.next = &s->desc;

    gdma_ll_rx_set_desc_addr(s->gdma, ADC_DMA_RX_CH, (uint32_t)&s->desc);
    gdma_ll_rx_start(s->gdma, ADC_DMA_RX_CH);
    adc_ll_digi_dma_enable();
    adc_ll_digi_clear_pattern_table(ADC_UNIT_1);
}

static void adc_gdma_isr(void *arg)
{
    esp_foc_adc_dma_sense_t *s = (esp_foc_adc_dma_sense_t *)arg;
    uint32_t st = gdma_ll_rx_get_interrupt_status(s->gdma, ADC_DMA_RX_CH, false);
    if ((st & GDMA_LL_EVENT_RX_SUC_EOF) == 0) {
        return;
    }
    uint32_t t0 = esp_cpu_get_cycle_count();
    gdma_ll_rx_clear_interrupt_status(s->gdma, ADC_DMA_RX_CH, GDMA_LL_EVENT_RX_SUC_EOF);

    /*
     * Stop the converter first, ahead of the parse and the control callback. The
     * block is complete, and everything below this line — the FOC step included —
     * can take longer than the remainder of the PWM period. Doing it at the end
     * instead put the write after the next TEZ had already delivered
     * ADC_TASK_START0, which cleared a start nothing would reissue: the chain
     * died for the rest of the launch, the last published frame froze, and the
     * align witness saw a raw channel with exactly zero variance.
     */
    /*
     * With two starts per period the next one is ~7 µs after this EOF, less than
     * the callbacks below may take, so the frame is rearmed here too, before them.
     * The disable stays even though each start is stopped in hardware after its
     * one conversion: without it the converter ran on at its interval timer
     * (39 k frames/s). A whole frame leaves the pattern pointer and the sample
     * count at zero, so the leg order holds frame to frame; it is set by the
     * first start after esp_foc_adc_dma_sense_restart().
     */
    const bool single = (s->starts == 1u);
    adc_ll_digi_trigger_disable();
    if (!single && s->armed) {
        adc_rearm(s);
    }

    adc_parse_and_publish(s);

    if (s->on_done != NULL) {
        s->on_done(s->on_done_arg);
    }
    /* Rearm after callback so trip/disarm can suppress the next frame. */
    if (s->armed && single) {
        adc_rearm(s);
    }

    /* a = µs, b = CPU cycles — full GDMA EOF ISR (parse + on_done + rearm). */
    {
        uint32_t dt = esp_cpu_get_cycle_count() - t0;
        uint32_t tpus = esp_rom_get_cpu_ticks_per_us();
        uint32_t us = (tpus > 0u) ? (dt / tpus) : 0u;
        esp_foc_trace_push(ESP_FOC_TRACE_DMA_EOF, (int32_t)us, (int32_t)dt);
    }
}

esp_err_t esp_foc_adc_dma_sense_init(esp_foc_adc_dma_sense_t **out,
                                     const esp_foc_adc_dma_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(out != NULL && cfg != NULL, ESP_ERR_INVALID_ARG, TAG, "null");
    ESP_RETURN_ON_FALSE(cfg->shunt_count >= 2 && cfg->shunt_count <= 3, ESP_ERR_INVALID_ARG, TAG, "shunts");

    esp_foc_adc_dma_sense_t *s = NULL;
    for (int i = 0; i < CONFIG_ESP_FOC_MAX_INVERTERS; i++) {
        if (!s_pool[i].inited) {
            s = &s_pool[i];
            break;
        }
    }
    ESP_RETURN_ON_FALSE(s != NULL, ESP_ERR_NO_MEM, TAG, "pool full");
    memset(s, 0, sizeof(*s));

    s->shunt_count = cfg->shunt_count;
    /*
     * Inline shunts are read only at the two zero-vector centres, TEZ and the
     * timer peak. The PWM pattern is symmetric about both, so the switching
     * ripple crosses its mean there, and no edge comes near them for any duty.
     * Anywhere else some duty puts an edge on the sample: at low modulation every
     * edge sits a quarter period from the peak, which at 20 kHz is exactly where
     * a neighbour 12.8 µs from the peak lands (measured: up to 1 A of garbage).
     * Conversions are at least 12.5 µs apart, so two legs cannot share a centre:
     * two shunts are read one per centre, A on TEZ and B on the peak, and A's
     * fundamental is half a period older than B's.
     *
     * That is two starts per period of one conversion each, not one block with a
     * half-period interval: the converter waits half an interval after a start
     * before its first sample, so such a block runs to within ~1.6 µs of the next
     * start at 20 kHz, and one start in three was lost (13.3 k EOF/s).
     *
     * Three shunts keep one A-B-C block at the minimum interval around B, which
     * still has the quarter-period problem; that layout has not been on silicon.
     */
    const uint32_t pwm_hz = cfg->pwm_hz != 0u ? cfg->pwm_hz : 20000u;
    const uint64_t period_ns = 1000000000ull / pwm_hz;
    const uint32_t interval_ns =
        (uint32_t)(ADC_MIN_INTERVAL_CYCLES * 1000000000ull / ADC_TIMER_HZ);
    s->conv_count = s->shunt_count;
    for (uint8_t i = 0; i < s->shunt_count; i++) {
        s->conv_slot[i] = i;
    }
    s->interval_cycles = ADC_MIN_INTERVAL_CYCLES;
    s->starts = (s->shunt_count == 2u) ? 2u : 1u;
    const uint32_t per_start = s->conv_count / s->starts;
    const uint32_t anchor = (s->starts == 2u) ? 0u : 1u;
    s->lead_ns = ESP_FOC_ADC_SAMPLE_LATENCY_NS + interval_ns / 2u + anchor * interval_ns;
    for (uint8_t i = 0; i < s->conv_count; i++) {
        s->conv_offset_ns[i] = (s->starts == 2u)
            ? ((int32_t)i - 1) * (int32_t)(period_ns / 2u)
            : ((int32_t)i - 1) * (int32_t)interval_ns;
    }
    const uint64_t block_ns = ESP_FOC_ADC_SAMPLE_LATENCY_NS + interval_ns / 2u +
                              (uint64_t)(per_start - 1u) * interval_ns +
                              ESP_FOC_ADC_SAMPLE_TO_DONE_NS;
    ESP_RETURN_ON_FALSE(block_ns < period_ns / s->starts, ESP_ERR_INVALID_ARG, TAG,
                        "pwm %lu Hz: a start of %lu conversions needs %llu ns, has %llu",
                        (unsigned long)pwm_hz, (unsigned long)per_start,
                        (unsigned long long)block_ns,
                        (unsigned long long)(period_ns / s->starts));
    for (uint8_t i = 0; i < s->conv_count; i++) {
        s->conv_per_slot[s->conv_slot[i]]++;
    }
    s->on_done = cfg->on_done;
    s->on_done_arg = cfg->on_done_arg;

    int gpios[3] = { cfg->gpio_iu, cfg->gpio_iv, cfg->gpio_iw };
    for (uint8_t i = 0; i < s->shunt_count; i++) {
        int ch = gpio_to_adc1_channel(gpios[i]);
        ESP_RETURN_ON_FALSE(ch >= 0, ESP_ERR_INVALID_ARG, TAG, "gpio->ch");
        s->channels[i] = ch;
        gpio_as_analog(gpios[i]);
    }

    float den = cfg->amp_gain * cfg->shunt_ohm;
    ESP_RETURN_ON_FALSE(den > 1e-9f, ESP_ERR_INVALID_ARG, TAG, "gain/shunt");
    float amps_per_count = (3.3f / 4096.0f) / den;
    s->scale_q16 = q16_from_float(amps_per_count);

    /* SAR power alone does not gate digi APB/func clocks on C6 (SOC_RCC_IS_INDEPENDENT). */
    adc_apb_periph_claim();
    ANALOG_CLOCK_ENABLE();
    sar_periph_ctrl_adc_continuous_power_acquire();

    ESP_RETURN_ON_ERROR(esp_clk_tree_enable_src(SOC_MOD_CLK_PLL_F80M, true), TAG, "pll80");

#if SOC_ADC_CALIBRATION_V1_SUPPORTED
    /* Without regi2c init-code, digi can sit armed with zero DMA frames on C6. */
    adc_hal_calibration_init(ADC_UNIT_1);
    adc_calc_hw_calibration_code(ADC_UNIT_1, ADC_ATTEN_DB_12);
    adc_set_hw_calibration_code(ADC_UNIT_1, ADC_ATTEN_DB_12);
#endif

    adc_hal_dma_config_t dma_cfg = {
        .eof_desc_num = 1,
        .eof_step = 1,
        .eof_num = s->conv_count,
    };
    s->hal_ctx.rx_desc = &s->desc;
    adc_hal_dma_ctx_config(&s->hal_ctx, &dma_cfg);
    adc_hal_digi_init(&s->hal_ctx);

    adc_ll_digi_controller_clk_div(ESP_FOC_ADC_CLKM_DIV, 1, 0);
    adc_ll_digi_clk_sel(ADC_DIGI_CLK_SRC_PLL_F80M);
    adc_ll_digi_set_clk_div(ADC_LL_DIGI_SAR_CLK_DIV_DEFAULT);
    adc_ll_set_sample_cycle(ESP_FOC_ADC_SAMPLE_CYCLE);

    adc_ll_set_power_manage(ADC_UNIT_1, ADC_LL_POWER_SW_ON);
    adc_ll_digi_reset_pattern_table();
    adc_ll_digi_set_pattern_table_len(ADC_UNIT_1, s->conv_count);
    for (uint8_t i = 0; i < s->conv_count; i++) {
        adc_digi_pattern_config_t pat = {
            .atten = ADC_ATTEN_DB_12,
            .channel = (uint8_t)s->channels[s->conv_slot[i]],
            .unit = ADC_UNIT_1,
            .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
        };
        adc_ll_digi_set_pattern_table(ADC_UNIT_1, i, pat);
    }

    adc_ll_digi_dma_set_eof_num(s->conv_count);
    ESP_LOGI(TAG, "block conv=%u starts=%u interval=%lu lead=%lu ns clkm_div=%u sample_cycle=%u",
             (unsigned)s->conv_count, (unsigned)s->starts,
             (unsigned long)(s->interval_cycles * 1000000000ull / ADC_TIMER_HZ),
             (unsigned long)s->lead_ns,
             (unsigned)ESP_FOC_ADC_CLKM_DIV, (unsigned)ESP_FOC_ADC_SAMPLE_CYCLE);
    /*
     * One block per PWM period: an MCPWM TEZ starts it through ADC_TASK_START0
     * and the DMA end-of-frame stops it through ADC_TASK_STOP0 (etm_link). That sync is the reason this driver gates on
     * SOC_ETM_SUPPORTED at all — with a free-running converter the sample instant
     * sits wherever the timer came up.
     *
     * meas_num_limit looks like the natural bound and is not usable here: it
     * latches, and ADC_TASK_START0 does not clear it, so the chain stops after
     * exactly one block (measured: tez=5166 against dma=1). The block is
     * therefore bounded by eof_num plus the ETM stop.
     *
     * The interval timer keeps a job, but a different one: it clocks samples
     * *within* a block.
     */
    adc_ll_digi_set_trigger_interval(s->interval_cycles);
    adc_ll_digi_convert_limit_enable(false);
    /* Digi timer starts in arm() — avoid samples before enable(). */
    adc_ll_digi_trigger_disable();
    adc_ll_digi_dma_enable();

    gdma_ll_enable_bus_clock(ADC_DMA_GROUP, true);
    gdma_ll_reset_register(ADC_DMA_GROUP);
    s->gdma = GDMA_LL_GET_HW(ADC_DMA_GROUP);

    gdma_ll_rx_reset_channel(s->gdma, ADC_DMA_RX_CH);
    gdma_ll_rx_connect_to_periph(s->gdma, ADC_DMA_RX_CH, GDMA_TRIG_PERIPH_ADC,
                                 SOC_GDMA_TRIG_PERIPH_ADC0);
    gdma_ll_rx_enable_owner_check(s->gdma, ADC_DMA_RX_CH, false);
    gdma_ll_rx_enable_etm_task(s->gdma, ADC_DMA_RX_CH, false);
    gdma_ll_rx_enable_interrupt(s->gdma, ADC_DMA_RX_CH, GDMA_LL_EVENT_RX_SUC_EOF, true);

    esp_err_t err = esp_intr_alloc(
        ETS_DMA_IN_CH0_INTR_SOURCE + ADC_DMA_RX_CH,
        ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3,
        adc_gdma_isr,
        s,
        &s->intr);
    ESP_RETURN_ON_ERROR(err, TAG, "gdma intr");

    s->armed = false;
    s->inited = true;
    *out = s;
    return ESP_OK;
}

void esp_foc_adc_dma_sense_deinit(esp_foc_adc_dma_sense_t *s)
{
    if (s == NULL || !s->inited) {
        return;
    }
    s->armed = false;
    if (s->intr != NULL) {
        esp_intr_free(s->intr);
        s->intr = NULL;
    }
    if (s->gdma != NULL) {
        gdma_ll_rx_stop(s->gdma, ADC_DMA_RX_CH);
        gdma_ll_rx_enable_interrupt(s->gdma, ADC_DMA_RX_CH, GDMA_LL_EVENT_RX_SUC_EOF, false);
        gdma_ll_rx_disconnect_from_periph(s->gdma, ADC_DMA_RX_CH);
    }
    adc_ll_digi_trigger_disable();
    adc_ll_digi_dma_disable();
    adc_hal_digi_deinit();
    sar_periph_ctrl_adc_continuous_power_release();
    ANALOG_CLOCK_DISABLE();
    adc_apb_periph_free();
    s->inited = false;
}

esp_err_t esp_foc_adc_dma_sense_arm(esp_foc_adc_dma_sense_t *s)
{
    ESP_RETURN_ON_FALSE(s != NULL && s->inited, ESP_ERR_INVALID_STATE, TAG, "state");
    s->armed = true;
    s->bad_frames = 0u;
    adc_rearm(s);
    /* Only place the trigger is turned on. From here the convert limit ends each
     * block and TEZ starts the next one. */
    adc_ll_digi_trigger_enable();
    return ESP_OK;
}

void esp_foc_adc_dma_sense_restart(esp_foc_adc_dma_sense_t *s)
{
    if (s == NULL || !s->inited || !s->armed) {
        return;
    }
    adc_ll_digi_trigger_disable();
    gdma_ll_rx_stop(s->gdma, ADC_DMA_RX_CH);
    /*
     * A frame cut short by a closed start gate leaves the converter's sample
     * count part-way, and neither the FSM reset nor the DMA reset clears it, so
     * the next frame would close one conversion early. The count clears when it
     * matches eof_num: walk eof_num through every value the count can hold, each
     * long enough for the converter clock to see.
     */
    for (uint32_t i = 0; i < s->conv_count; i++) {
        adc_ll_digi_dma_set_eof_num(i);
        esp_rom_delay_us(1);
    }
    adc_ll_digi_dma_set_eof_num(s->conv_count);
    adc_rearm(s);
}

void esp_foc_adc_dma_sense_disarm(esp_foc_adc_dma_sense_t *s)
{
    if (s == NULL || !s->inited) {
        return;
    }
    s->armed = false;
    adc_ll_digi_trigger_disable();
    if (s->gdma != NULL) {
        gdma_ll_rx_stop(s->gdma, ADC_DMA_RX_CH);
    }
}

void esp_foc_adc_dma_sense_kick(esp_foc_adc_dma_sense_t *s)
{
    (void)s;
}

void esp_foc_adc_dma_sense_fetch(esp_foc_adc_dma_sense_t *s,
                                 q16_t *iu, q16_t *iv, q16_t *iw)
{
    if (s == NULL) {
        return;
    }
    if (iu) {
        *iu = s->iu;
    }
    if (iv) {
        *iv = s->iv;
    }
    if (iw) {
        *iw = s->iw;
    }
    s->sample_ready = false;
}

void esp_foc_adc_dma_sense_peek(esp_foc_adc_dma_sense_t *s,
                                q16_t *iu, q16_t *iv, q16_t *iw)
{
    if (s == NULL) {
        return;
    }
    if (iu) {
        *iu = s->iu;
    }
    if (iv) {
        *iv = s->iv;
    }
    if (iw) {
        *iw = s->iw;
    }
}

bool esp_foc_adc_dma_sense_sample_ready(esp_foc_adc_dma_sense_t *s)
{
    return s != NULL && s->sample_ready;
}

void esp_foc_adc_dma_sense_calibrate(esp_foc_adc_dma_sense_t *s, int rounds)
{
    if (s == NULL || !s->inited || rounds <= 0) {
        return;
    }
    int64_t acc[ADC_MAX_SHUNTS] = {0};
    int n[ADC_MAX_SHUNTS] = {0};
    int32_t prev[ADC_MAX_SHUNTS];
    int got = 0;

    for (uint8_t i = 0; i < ADC_MAX_SHUNTS; i++) {
        prev[i] = s->offset_raw[i];
        s->offset_raw[i] = 0;
    }

    while (got < rounds) {
        if (!s->sample_ready) {
            /* Yield — busy-spin starves IDLE and trips TWDT at ~20 kHz EOF. */
            esp_foc_sleep_ms(1);
            continue;
        }
        const adc_digi_output_data_t *p = (const adc_digi_output_data_t *)s->dma_buf;
        for (uint8_t i = 0; i < s->conv_count; i++) {
            const int k = shunt_of_channel(s, p[i].type2.channel);
            if (k >= 0) {
                acc[k] += (int32_t)p[i].type2.data;
                n[k]++;
            }
        }
        s->sample_ready = false;
        got++;
    }
    /*
     * Divide by what each slot actually got.
     *
     * Dividing by the round count instead makes every unrecognised sample dilute
     * that slot's zero, and a diluted zero is not noise — it is current the
     * amplifier never saw, published for the rest of the run. Half the windows
     * missing puts the idle reading near half of mid-scale, which on this bench
     * read as 2.3 A flowing in two legs with the third at rest: a plausible
     * enough shape to be mistaken for a shorted machine.
     */
    for (uint8_t i = 0; i < s->shunt_count; i++) {
        const int want = rounds * (int)s->conv_per_slot[i];
        if (n[i] < want) {
            ESP_LOGW(TAG, "sense slot %u zeroed on %d of %d conversions",
                     (unsigned)i, n[i], want);
        }
        if (n[i] > 0) {
            s->offset_raw[i] = (int32_t)(acc[i] / n[i]);
        } else {
            ESP_LOGE(TAG, "sense slot %u never carried channel %d — its zero is "
                          "unknown and every current from it is wrong",
                     (unsigned)i, s->channels[i]);
        }
        /*
         * The zero is a property of the amplifier, so it should be the same number
         * on every arm. Saying so out loud once, and complaining when it moves, is
         * what tells a real current apart from a bad zero: the two are
         * indistinguishable downstream.
         */
        const int32_t d = s->offset_raw[i] - prev[i];
        if (prev[i] == 0) {
            ESP_LOGI(TAG, "sense slot %u (channel %d) zero=%ld counts",
                     (unsigned)i, s->channels[i], (long)s->offset_raw[i]);
        } else if (d > ADC_OFFSET_DRIFT_COUNTS || d < -ADC_OFFSET_DRIFT_COUNTS) {
            ESP_LOGW(TAG, "sense slot %u zero moved %ld counts to %ld",
                     (unsigned)i, (long)d, (long)s->offset_raw[i]);
        }
    }
}

uint8_t esp_foc_adc_dma_sense_shunt_count(const esp_foc_adc_dma_sense_t *s)
{
    return s != NULL ? s->shunt_count : 0;
}

uint8_t esp_foc_adc_dma_sense_conv_count(const esp_foc_adc_dma_sense_t *s)
{
    return s != NULL ? s->conv_count : 0;
}

uint8_t esp_foc_adc_dma_sense_starts(const esp_foc_adc_dma_sense_t *s)
{
    return s != NULL ? s->starts : 0u;
}

uint32_t esp_foc_adc_dma_sense_lead_ns(const esp_foc_adc_dma_sense_t *s)
{
    return s != NULL ? s->lead_ns : 0u;
}

int32_t esp_foc_adc_dma_sense_conv_offset_ns(const esp_foc_adc_dma_sense_t *s, uint8_t i)
{
    return (s != NULL && i < s->conv_count) ? s->conv_offset_ns[i] : 0;
}

uint8_t esp_foc_adc_dma_sense_peek_conversions(esp_foc_adc_dma_sense_t *s,
                                               q16_t *amps, uint8_t *slot,
                                               uint8_t max)
{
    if (s == NULL || !s->inited) {
        return 0;
    }
    const adc_digi_output_data_t *p = (const adc_digi_output_data_t *)s->dma_buf;
    uint8_t w = 0;
    for (uint8_t i = 0; i < s->conv_count && w < max; i++) {
        const int k = shunt_of_channel(s, p[i].type2.channel);
        if (k < 0) {
            continue;
        }
        const int32_t delta = (int32_t)p[i].type2.data - s->offset_raw[k];
        if (amps != NULL) {
            amps[w] = q16_mul((q16_t)(delta << 16), s->scale_q16);
        }
        if (slot != NULL) {
            slot[w] = (uint8_t)k;
        }
        w++;
    }
    return w;
}
