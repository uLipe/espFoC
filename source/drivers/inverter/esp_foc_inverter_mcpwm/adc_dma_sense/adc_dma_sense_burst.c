/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#include "esp_foc_driver_soc_gate.h"
#include "adc_dma_sense.h"

#if ESP_FOC_ADC_SAR_BURST

#include <string.h>

#include "esp_check.h"
#include "esp_cpu.h"
#include "esp_intr_alloc.h"
#include "esp_foc_drv_intr.h"
#include "esp_log.h"
#include "esp_private/adc_share_hw_ctrl.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/regi2c_ctrl.h"
#include "esp_private/sar_periph_ctrl.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "espFoC/debug/esp_foc_trace.h"
#include "espFoC/osal/esp_foc_osal.h"
#include "hal/adc_hal_common.h"
#include "hal/adc_periph.h"
#include "hal/adc_types.h"
#include "hal/ahb_dma_ll.h"
#include "hal/dma_types.h"
#include "hal/gdma_channel.h"
#include "hal/gdma_ll.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"
#include "soc/interrupts.h"

static const char *TAG = "foc_adc";

#define ADC_DMA_RX_CH   ESP_FOC_ADC_DMA_RX_CH
#define ADC_MAX_SHUNTS  3
#define ADC_MAX_CONV    ESP_FOC_ADC_MAX_CONV
#define ADC_BYTES_PER   4
#define ADC_OFFSET_DRIFT_COUNTS 16
/*
 * Descriptors the DMA can run ahead of the ISR. One burst fills one, the ISR
 * hands each back, so the ring only has to cover ISR latency; when it runs dry
 * the channel stalls, IN_DONE stops coming, and no further burst is closed.
 */
#define BURST_RING 8
/*
 * SAMPLE task to the first conversion of the burst, and the spacing of the rest
 * (XTAL 40 MHz, controller divider 0). Measured from the burst length against
 * the sample-task phase: 9.65..10.2 µs per boot, 0.6 µs per conversion. Not
 * placed against a PWM edge yet.
 */
#ifndef ESP_FOC_ADC_BURST_LATENCY_NS
#define ESP_FOC_ADC_BURST_LATENCY_NS 9900u
#endif
#define BURST_CONV_NS 600u
/*
 * No attenuation (SOC_ADC_ATTEN_NUM 1). Where the SAR is differential each
 * shunt is converted as its pad pair, P - N over -2.2..+2.2 V onto
 * 0..ADC_LL_ZERO_DIFF_CODE..4393 (17-bit weighted result, see
 * adc_digi_output_data_t); the board puts its amplifier reference on the N
 * pad. Single-ended the other side of the SAR sits at 0 V instead, which
 * leaves only 0..+2.2 V of a P pad.
 *
 * The full scale is measured, not the nominal 2 V: a meter-verified 1.65 V
 * converts to 1646 counts above the zero code (1.0023 mV/count).
 */
#define BURST_CODE_MAX  4393
#define BURST_FS_VOLTS  2.2f
#define BURST_FS_COUNTS (BURST_CODE_MAX - ADC_LL_ZERO_DIFF_CODE)
#if SOC_ADC_DIFF_SUPPORTED
/* Bit 0 of a pattern item selects the pad pair, whichever pad the channel
 * field names; bit 1 had no effect on silicon. Not written by the IDF LL. */
#define BURST_PATT_DIFF 1u
#else
#define BURST_PATT_DIFF 0u
#endif

struct esp_foc_adc_dma_sense {
    bool inited;
    volatile bool armed;
    uint8_t shunt_count;
    uint8_t conv_count;
    uint8_t unit;
    uint8_t conv_per_slot[ADC_MAX_SHUNTS];
    uint8_t ring_idx;
    uint32_t lead_ns;
    int32_t conv_offset_ns[ADC_MAX_CONV];
    int channels[ADC_MAX_SHUNTS];
    q16_t scale_q16;
    int32_t offset_raw[ADC_MAX_SHUNTS];
    volatile q16_t iu, iv, iw;
    volatile bool sample_ready;
    volatile uint32_t bad_frames;
    volatile uint32_t saturations;
    esp_foc_adc_dma_done_fn_t on_done;
    void *on_done_arg;
    ahb_dma_dev_t *dma;
    intr_handle_t intr;
    uint32_t dma_buf[ADC_MAX_CONV];
    dma_descriptor_t desc[BURST_RING];
    uint32_t ring_buf[BURST_RING][ADC_MAX_CONV] __attribute__((aligned(4)));
};

static esp_foc_adc_dma_sense_t s_pool[CONFIG_ESP_FOC_MAX_INVERTERS];
/*
 * The converter has one DMA output; both SARs feed it. Two sense instances
 * would interleave their bursts in one stream, which has not been framed on
 * silicon, so a second concurrent instance is refused until it is.
 */
static uint8_t s_active;
static bool s_dma_ready;

static void sar_set_trigger(uint8_t unit, adc_ll_trigger_mode_t mode, bool cont)
{
    /* No LL setter for the ETM trigger mode or continue mode. Continue goes on
     * before ETM and comes off after it, so a SAMPLE never lands half-configured. */
    if (unit == 0u) {
        if (mode != ADC_LL_TRIGGER_MODE_OFF) {
            ADC.ctrl0.sar1_continue_mode_en = cont;
        }
        ADC.ctrl0.sar1_trigger_mode = mode;
        if (mode == ADC_LL_TRIGGER_MODE_OFF) {
            ADC.ctrl0.sar1_continue_mode_en = cont;
        }
    } else {
        if (mode != ADC_LL_TRIGGER_MODE_OFF) {
            ADC.ctrl1.sar2_continue_mode_en = cont;
        }
        ADC.ctrl1.sar2_trigger_mode = mode;
        if (mode == ADC_LL_TRIGGER_MODE_OFF) {
            ADC.ctrl1.sar2_continue_mode_en = cont;
        }
    }
}

static int gpio_to_channel(uint8_t unit, int gpio)
{
    if (gpio < 0) {
        return -1;
    }
    for (int ch = 0; ch < SOC_ADC_MAX_CHANNEL_NUM; ch++) {
        if (adc_channel_io_map[unit][ch] == gpio) {
            return ch;
        }
    }
    return -1;
}

/*
 * Pads come in P/N pairs and the IDF numbers each pad as its own channel,
 * N first: channel 2k is CHk_N, 2k+1 is CHk_P. The shunt GPIO must be the P
 * pad so a rising amplifier output reads as a rising code.
 */
static bool channel_is_p_pad(int ch)
{
#if SOC_ADC_DIFF_SUPPORTED
    return (ch & 1) == 1;
#else
    (void)ch;
    return true;
#endif
}

static inline bool code_on_rail(uint32_t raw)
{
    return raw == 0u || raw >= BURST_CODE_MAX;
}

static void patt_set_model(adc_unit_t unit, uint32_t idx, uint32_t model)
{
    const uint32_t bits = (model & 3u) << (18u - (idx % 4u) * 6u);
    volatile uint32_t *tab;
    if (unit == ADC_UNIT_1) {
        switch (idx / 4u) {
        case 0: tab = &ADC.sar1_patt_tab1.val; break;
        case 1: tab = &ADC.sar1_patt_tab2.val; break;
        case 2: tab = &ADC.sar1_patt_tab3.val; break;
        default: tab = &ADC.sar1_patt_tab4.val; break;
        }
    } else {
        switch (idx / 4u) {
        case 0: tab = &ADC.sar2_patt_tab1.val; break;
        case 1: tab = &ADC.sar2_patt_tab2.val; break;
        case 2: tab = &ADC.sar2_patt_tab3.val; break;
        default: tab = &ADC.sar2_patt_tab4.val; break;
        }
    }
    *tab |= bits;
}

static void gpio_as_analog(int gpio)
{
    gpio_ll_input_disable(&GPIO, gpio);
    gpio_ll_output_disable(&GPIO, gpio);
    gpio_ll_pullup_dis(&GPIO, gpio);
    gpio_ll_pulldown_dis(&GPIO, gpio);
    esp_rom_gpio_pad_select_gpio((uint32_t)gpio);
}

/* The pair order inside a burst is fixed for a run but not at start, so a
 * conversion is placed by its channel tag, never by position. */
static int shunt_of_word(const esp_foc_adc_dma_sense_t *s, adc_digi_output_data_t w)
{
    if (w.type2.unit != s->unit) {
        return -1;
    }
    for (uint8_t i = 0; i < s->shunt_count; i++) {
        if ((uint32_t)s->channels[i] == w.type2.channel) {
            return (int)i;
        }
    }
    return -1;
}

static void adc_parse_and_publish(esp_foc_adc_dma_sense_t *s)
{
    q16_t vals[ADC_MAX_SHUNTS] = {0};
    int32_t delta[ADC_MAX_SHUNTS] = {0};
    uint8_t n[ADC_MAX_SHUNTS] = {0};
    uint8_t seen = 0;
    uint32_t rail = 0;
    for (uint8_t i = 0; i < s->conv_count; i++) {
        const adc_digi_output_data_t w = { .val = s->dma_buf[i] };
        const int k = shunt_of_word(s, w);
        if (k < 0) {
            continue;
        }
        delta[k] = (int32_t)w.type2.data - s->offset_raw[k];
        rail += code_on_rail(w.type2.data) ? 1u : 0u;
        n[k]++;
    }
    /* A clipped sample is still published (it is the nearest value the pad can
     * give) but counted, so a current past the window is not mistaken for one
     * at its edge. */
    if (rail != 0u) {
        s->saturations += rail;
    }
    for (uint8_t k = 0; k < s->shunt_count; k++) {
        if (n[k] != s->conv_per_slot[k]) {
            continue;
        }
        vals[k] = q16_mul((q16_t)(delta[k] << 16), s->scale_q16);
        seen++;
    }
    /* Keep the last good frame rather than publish zeros for a mis-tagged one;
     * the inverter's stale detector sees the repeat. */
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

static void ring_build(esp_foc_adc_dma_sense_t *s)
{
    for (int i = 0; i < BURST_RING; i++) {
        s->desc[i].dw0.size = s->conv_count * ADC_BYTES_PER;
        s->desc[i].dw0.length = 0;
        s->desc[i].dw0.err_eof = 0;
        s->desc[i].dw0.suc_eof = 0;
        s->desc[i].dw0.owner = 1;
        s->desc[i].buffer = s->ring_buf[i];
        s->desc[i].next = &s->desc[(i + 1) % BURST_RING];
    }
    s->ring_idx = 0;
}

/*
 * One IN_DONE per burst: the descriptor holding it is complete and the same
 * event has already stopped the SAR through ETM. Hand back every completed
 * descriptor (more than one only if this ISR was held off a period) and keep
 * the newest frame.
 */
static void adc_dma_isr(void *arg)
{
    esp_foc_adc_dma_sense_t *s = (esp_foc_adc_dma_sense_t *)arg;
    const uint32_t st = ahb_dma_ll_rx_get_interrupt_status(s->dma, ADC_DMA_RX_CH, false);
    ahb_dma_ll_rx_clear_interrupt_status(s->dma, ADC_DMA_RX_CH, st);
    if ((st & GDMA_LL_EVENT_RX_DONE) == 0) {
        return;
    }
    const uint32_t t0 = esp_cpu_get_cycle_count();
    const uint8_t conv = s->conv_count;
    uint8_t idx = s->ring_idx;
    uint8_t got = 0;
    while (got < BURST_RING) {
        volatile dma_descriptor_t *d = &s->desc[idx];
        if (d->dw0.owner) {
            break;
        }
        const volatile uint32_t *src = s->ring_buf[idx];
        for (uint8_t i = 0; i < conv; i++) {
            s->dma_buf[i] = src[i];
        }
        d->dw0.length = 0;
        d->dw0.suc_eof = 0;
        d->dw0.owner = 1;
        idx = (uint8_t)((idx + 1u) % BURST_RING);
        got++;
    }
    s->ring_idx = idx;
    if (got == 0u) {
        return;
    }

    adc_parse_and_publish(s);
    if (s->on_done != NULL) {
        s->on_done(s->on_done_arg);
    }

    /* a = µs, b = CPU cycles — full DMA done ISR (hand-back + parse + on_done). */
    const uint32_t dt = esp_cpu_get_cycle_count() - t0;
    const uint32_t tpus = esp_rom_get_cpu_ticks_per_us();
    esp_foc_trace_push(ESP_FOC_TRACE_DMA_EOF,
                       (int32_t)((tpus > 0u) ? (dt / tpus) : 0u), (int32_t)dt);
}

static void dma_bring_up(esp_foc_adc_dma_sense_t *s)
{
    if (!s_dma_ready) {
        PERIPH_RCC_ATOMIC() {
            gdma_ll_enable_bus_clock(GDMA_LL_LP_AHB_GROUP_START_ID, true);
            gdma_ll_reset_register(GDMA_LL_LP_AHB_GROUP_START_ID);
        }
        s_dma_ready = true;
    }
    s->dma = AHB_DMA_LL_GET_HW(GDMA_LL_LP_AHB_GROUP_START_ID);
    lp_ahb_dma_ll_set_default_memory_range(s->dma);
    ahb_dma_ll_rx_stop(s->dma, ADC_DMA_RX_CH);
    ahb_dma_ll_rx_reset_channel(s->dma, ADC_DMA_RX_CH);
    ahb_dma_ll_rx_connect_to_periph(s->dma, ADC_DMA_RX_CH, SOC_GDMA_TRIG_PERIPH_ADC0);
    ahb_dma_ll_rx_enable_etm_task(s->dma, ADC_DMA_RX_CH, false);
    /* Owner check is what keeps a burst out of a descriptor the ISR still holds. */
    ahb_dma_ll_rx_enable_owner_check(s->dma, ADC_DMA_RX_CH, true);
    ahb_dma_ll_rx_enable_interrupt(s->dma, ADC_DMA_RX_CH, AHB_DMA_LL_RX_EVENT_MASK, false);
    ahb_dma_ll_rx_clear_interrupt_status(s->dma, ADC_DMA_RX_CH, AHB_DMA_LL_RX_EVENT_MASK);
    ahb_dma_ll_rx_enable_interrupt(s->dma, ADC_DMA_RX_CH, GDMA_LL_EVENT_RX_DONE, true);
}

esp_err_t esp_foc_adc_dma_sense_init(esp_foc_adc_dma_sense_t **out,
                                     const esp_foc_adc_dma_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(out != NULL && cfg != NULL, ESP_ERR_INVALID_ARG, TAG, "null");
    /*
     * Only the 8-byte descriptor is framed on silicon: IN_DONE stops the SAR
     * right after the second conversion. A 4-byte one let one extra through, so
     * a 12-byte one is not assumed to close on three.
     */
    ESP_RETURN_ON_FALSE(cfg->shunt_count == 2, ESP_ERR_NOT_SUPPORTED, TAG,
                        "burst sense: %u shunts not validated", (unsigned)cfg->shunt_count);
    ESP_RETURN_ON_FALSE(cfg->adc_unit <= ESP_FOC_ADC_UNITS, ESP_ERR_NOT_SUPPORTED, TAG,
                        "adc unit %u", (unsigned)cfg->adc_unit);
    ESP_RETURN_ON_FALSE(s_active == 0u, ESP_ERR_NOT_SUPPORTED, TAG,
                        "one sense instance at a time on the shared ADC DMA stream");

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
    s->conv_count = cfg->shunt_count;
    s->unit = (cfg->adc_unit == 2u) ? 1u : 0u;
    for (uint8_t i = 0; i < s->conv_count; i++) {
        s->conv_per_slot[i] = 1u;
    }

    /*
     * Both legs in one burst around the timer peak, the zero-vector centre of
     * the PWM pattern: the pair is 0.6 µs apart, which keeps both inside the
     * centre for any duty a modulator leaves a zero vector at.
     */
    const uint32_t pwm_hz = cfg->pwm_hz != 0u ? cfg->pwm_hz : 20000u;
    const uint32_t half_ns = 500000000u / pwm_hz;
    s->lead_ns = ESP_FOC_ADC_BURST_LATENCY_NS + (s->conv_count - 1u) * BURST_CONV_NS / 2u;
    for (uint8_t i = 0; i < s->conv_count; i++) {
        s->conv_offset_ns[i] = (int32_t)(i * BURST_CONV_NS) -
                               (int32_t)((s->conv_count - 1u) * BURST_CONV_NS / 2u);
    }
    ESP_RETURN_ON_FALSE(s->lead_ns < half_ns, ESP_ERR_INVALID_ARG, TAG,
                        "pwm %lu Hz: burst needs %lu ns ahead of the peak, half period is %lu",
                        (unsigned long)pwm_hz, (unsigned long)s->lead_ns,
                        (unsigned long)half_ns);

    s->on_done = cfg->on_done;
    s->on_done_arg = cfg->on_done_arg;

    const int gpios[3] = { cfg->gpio_iu, cfg->gpio_iv, cfg->gpio_iw };
    for (uint8_t i = 0; i < s->shunt_count; i++) {
        const int ch = gpio_to_channel(s->unit, gpios[i]);
        ESP_RETURN_ON_FALSE(ch >= 0, ESP_ERR_INVALID_ARG, TAG, "gpio %d is not on ADC%u",
                            gpios[i], (unsigned)s->unit + 1u);
        ESP_RETURN_ON_FALSE(channel_is_p_pad(ch), ESP_ERR_INVALID_ARG, TAG,
                            "gpio %d is an N pad; give the P pad, N carries the reference",
                            gpios[i]);
        s->channels[i] = ch;
        gpio_as_analog(gpios[i]);
#if SOC_ADC_DIFF_SUPPORTED
        gpio_as_analog(adc_channel_io_map[s->unit][ch - 1]);
#endif
    }

    const float den = cfg->amp_gain * cfg->shunt_ohm;
    ESP_RETURN_ON_FALSE(den > 1e-9f, ESP_ERR_INVALID_ARG, TAG, "gain/shunt");
    s->scale_q16 = q16_from_float((BURST_FS_VOLTS / (float)BURST_FS_COUNTS) / den);

    const adc_unit_t unit = (s->unit == 0u) ? ADC_UNIT_1 : ADC_UNIT_2;
    adc_apb_periph_claim();
    ANALOG_CLOCK_ENABLE();
    sar_periph_ctrl_adc_continuous_power_acquire();
    sar_periph_ctrl_adc_reset();
    adc_hal_set_controller(unit, ADC_HAL_CONTINUOUS_READ_MODE);

    adc_ll_set_sample_cycle(ADC_LL_SAMPLE_CYCLE_DEFAULT);
    adc_ll_digi_set_clk_div(ADC_LL_DIGI_SAR_CLK_DIV_DEFAULT);
    /* XTAL undivided: 0.6 µs per conversion. RC_FAST is the only other source
     * that runs on this converter, at less than half the rate. */
    adc_ll_digi_controller_clk_div(0, 0, 0);
    adc_ll_digi_clk_sel(ADC_DIGI_CLK_SRC_XTAL);
    adc_ll_digi_set_convert_mode((unit == ADC_UNIT_1) ? ADC_LL_DIGI_CONV_ONLY_ADC1
                                                      : ADC_LL_DIGI_CONV_ONLY_ADC2);
    sar_set_trigger(s->unit, ADC_LL_TRIGGER_MODE_OFF, false);
    adc_ll_digi_trigger_disable();
    adc_ll_set_power_manage(unit, ADC_LL_POWER_SW_ON);

    adc_ll_digi_clear_pattern_table(unit);
    adc_ll_digi_set_pattern_table_len(unit, s->conv_count);
    for (uint8_t i = 0; i < s->conv_count; i++) {
        adc_digi_pattern_config_t pat = {
            .atten = ADC_ATTEN_DB_12,
            .channel = (uint8_t)s->channels[i],
            .unit = unit,
            .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
        };
        adc_ll_digi_set_pattern_table(unit, i, pat);
        patt_set_model(unit, i, BURST_PATT_DIFF);
    }
    /*
     * No converter EOF at all. Any non-zero count closes a descriptor early,
     * and IN_DONE on that early close does not stop the burst it cut, so every
     * eof_num-th period carries one extra frame (measured at 2 and 4000).
     */
    adc_ll_digi_dma_set_eof_num(0);
    adc_ll_digi_dma_enable();

    dma_bring_up(s);
    esp_err_t err = esp_foc_drv_intr_alloc(ETS_LP_AHB_PDMA_IN_CH0_INTR_SOURCE + ADC_DMA_RX_CH,
                                           ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3,
                                           0, 0, adc_dma_isr, s, &s->intr);
    ESP_RETURN_ON_ERROR(err, TAG, "lp dma intr");

    ESP_LOGI(TAG, "burst ADC%u conv=%u ch=%d,%d %s lead=%lu ns",
             (unsigned)s->unit + 1u, (unsigned)s->conv_count,
             s->channels[0], s->channels[1], BURST_PATT_DIFF ? "diff" : "single",
             (unsigned long)s->lead_ns);

    s->armed = false;
    s->inited = true;
    s_active++;
    *out = s;
    return ESP_OK;
}

void esp_foc_adc_dma_sense_deinit(esp_foc_adc_dma_sense_t *s)
{
    if (s == NULL || !s->inited) {
        return;
    }
    s->armed = false;
    sar_set_trigger(s->unit, ADC_LL_TRIGGER_MODE_OFF, false);
    if (s->intr != NULL) {
        esp_intr_free(s->intr);
        s->intr = NULL;
    }
    ahb_dma_ll_rx_stop(s->dma, ADC_DMA_RX_CH);
    ahb_dma_ll_rx_enable_interrupt(s->dma, ADC_DMA_RX_CH, AHB_DMA_LL_RX_EVENT_MASK, false);
    ahb_dma_ll_rx_disconnect_all(s->dma, ADC_DMA_RX_CH);
    adc_ll_digi_dma_disable();
    adc_ll_digi_clear_pattern_table(s->unit == 0u ? ADC_UNIT_1 : ADC_UNIT_2);
    adc_ll_digi_reset();
    sar_periph_ctrl_adc_continuous_power_release();
    ANALOG_CLOCK_DISABLE();
    adc_apb_periph_free();
    s->inited = false;
    s_active--;
}

esp_err_t esp_foc_adc_dma_sense_arm(esp_foc_adc_dma_sense_t *s)
{
    ESP_RETURN_ON_FALSE(s != NULL && s->inited, ESP_ERR_INVALID_STATE, TAG, "state");
    sar_set_trigger(s->unit, ADC_LL_TRIGGER_MODE_OFF, false);
    ahb_dma_ll_rx_stop(s->dma, ADC_DMA_RX_CH);
    ahb_dma_ll_rx_reset_channel(s->dma, ADC_DMA_RX_CH);
    ahb_dma_ll_rx_clear_interrupt_status(s->dma, ADC_DMA_RX_CH, AHB_DMA_LL_RX_EVENT_MASK);
    ring_build(s);
    s->bad_frames = 0u;
    s->saturations = 0u;
    s->armed = true;
    adc_ll_digi_reset();
    adc_ll_digi_clear_pattern_table(s->unit == 0u ? ADC_UNIT_1 : ADC_UNIT_2);
    ahb_dma_ll_rx_set_desc_addr(s->dma, ADC_DMA_RX_CH, (uint32_t)&s->desc[0]);
    ahb_dma_ll_rx_start(s->dma, ADC_DMA_RX_CH);
    adc_ll_digi_dma_enable();
    /* From here every SAMPLE task opens one burst and IN_DONE closes it. */
    sar_set_trigger(s->unit, ADC_LL_TRIGGER_MODE_ETM, true);
    return ESP_OK;
}

void esp_foc_adc_dma_sense_restart(esp_foc_adc_dma_sense_t *s)
{
    if (s == NULL || !s->inited || !s->armed) {
        return;
    }
    (void)esp_foc_adc_dma_sense_arm(s);
}

void esp_foc_adc_dma_sense_disarm(esp_foc_adc_dma_sense_t *s)
{
    if (s == NULL || !s->inited) {
        return;
    }
    s->armed = false;
    sar_set_trigger(s->unit, ADC_LL_TRIGGER_MODE_OFF, false);
    ahb_dma_ll_rx_stop(s->dma, ADC_DMA_RX_CH);
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
            esp_foc_sleep_ms(1);
            continue;
        }
        for (uint8_t i = 0; i < s->conv_count; i++) {
            const adc_digi_output_data_t w = { .val = s->dma_buf[i] };
            const int k = shunt_of_word(s, w);
            if (k >= 0) {
                acc[k] += (int32_t)w.type2.data;
                n[k]++;
            }
        }
        s->sample_ready = false;
        got++;
    }
    /* Divide by what each slot actually got: a missing sample must not dilute
     * the zero into a standing false current. */
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
    return s != NULL ? 1u : 0u;
}

uint32_t esp_foc_adc_dma_sense_lead_ns(const esp_foc_adc_dma_sense_t *s)
{
    return s != NULL ? s->lead_ns : 0u;
}

int32_t esp_foc_adc_dma_sense_conv_offset_ns(const esp_foc_adc_dma_sense_t *s, uint8_t i)
{
    return (s != NULL && i < s->conv_count) ? s->conv_offset_ns[i] : 0;
}

esp_err_t esp_foc_adc_dma_sense_saturations(const esp_foc_adc_dma_sense_t *s, uint32_t *count)
{
    ESP_RETURN_ON_FALSE(s != NULL && s->inited && count != NULL, ESP_ERR_INVALID_ARG, TAG, "arg");
    *count = s->saturations;
    return ESP_OK;
}

uint8_t esp_foc_adc_dma_sense_peek_conversions(esp_foc_adc_dma_sense_t *s,
                                               q16_t *amps, uint8_t *slot,
                                               uint8_t max)
{
    if (s == NULL || !s->inited) {
        return 0;
    }
    uint8_t w = 0;
    for (uint8_t i = 0; i < s->conv_count && w < max; i++) {
        const adc_digi_output_data_t word = { .val = s->dma_buf[i] };
        const int k = shunt_of_word(s, word);
        if (k < 0) {
            continue;
        }
        const int32_t delta = (int32_t)word.type2.data - s->offset_raw[k];
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

#endif /* ESP_FOC_ADC_SAR_BURST */
