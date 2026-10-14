/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#include <stdbool.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"

#include "espFoC/drivers/esp_foc_vref_generator.h"
#include "espFoC/osal/esp_foc_osal.h"

/* The LP DAC has registers but no cap or LL in the IDF this builds against;
 * its presence is the register header. */
#if __has_include("soc/dac_struct.h") && __has_include("soc/lp_peri_clkrst_struct.h")
#define ESP_FOC_VREF_LP_DAC 1
#else
#define ESP_FOC_VREF_LP_DAC 0
#endif

#if ESP_FOC_VREF_LP_DAC
#include "esp_private/periph_ctrl.h"
#include "esp_rom_gpio.h"
#include "hal/gpio_ll.h"
#include "hal/rtc_io_ll.h"
#include "soc/dac_struct.h"
#include "soc/gpio_struct.h"
#include "soc/lp_peri_clkrst_struct.h"
#endif

static const char *TAG = "foc_vref";

#define VREF_SETTLE_MS_DEFAULT 2u

#if ESP_FOC_VREF_LP_DAC

extern dac_dev_t LP_DAC;

#define VREF_CHANNELS   2
/* Low-speed pad mode: 12-bit code over 0..VDDPST. */
#define VREF_CODE_MAX   4095u
/* Sintx timer divider pushing the DC code to the pad. */
#define VREF_SINTX_DIV  100u

static const int s_pad_gpio[VREF_CHANNELS] = { 4, 5 };

struct esp_foc_vref_generator {
    bool inited;
    uint8_t channel;
    int gpio;
    uint32_t settle_ms;
};

static esp_foc_vref_generator_t s_pool[VREF_CHANNELS];
static uint8_t s_active;

static void dac_clock_on(void)
{
    /* The LP_DAC block ignores writes until its functional clock runs. */
    PERIPH_RCC_ATOMIC() {
        LP_PERI_CLKRST.dac_ctrl.lp_dac_clk_sel = 0;
        LP_PERI_CLKRST.dac_ctrl.lp_dac_clk_en = 1;
        LP_PERI_CLKRST.dac_ctrl.lp_dac_rst_en = 1;
        LP_PERI_CLKRST.dac_ctrl.lp_dac_rst_en = 0;
    }
    LP_DAC.date.clk_en = 1;
}

static void dac_clock_off(void)
{
    LP_DAC.date.clk_en = 0;
    PERIPH_RCC_ATOMIC() {
        LP_PERI_CLKRST.dac_ctrl.lp_dac_clk_en = 0;
    }
}

static void pad_as_analog(int gpio)
{
    gpio_ll_input_disable(&GPIO, gpio);
    gpio_ll_output_disable(&GPIO, gpio);
    gpio_ll_pullup_dis(&GPIO, gpio);
    gpio_ll_pulldown_dis(&GPIO, gpio);
    esp_rom_gpio_pad_select_gpio((uint32_t)gpio);
    rtcio_ll_function_select(gpio, RTCIO_LL_FUNC_DIGITAL);
}

static void dac_set_code(uint8_t ch, uint32_t code)
{
    if (ch == 0u) {
        LP_DAC.sintx_data.dc_1 = code;
    } else {
        LP_DAC.sintx_data.dc_2 = code;
    }
}

/*
 * Low-speed mode with the output buffer at full swing: the pad feeds a SAR
 * input whose sample capacitor would otherwise pull it at every conversion.
 * The code reaches the pad through the Sintx path in DC mode (tone off).
 */
static void dac_pad_on(uint8_t ch)
{
    if (ch == 0u) {
        LP_DAC.pad_cfg.hs_mode_pad_0 = 0;
        LP_DAC.pad_cfg.dsamp_ls_pad_0 = 0;
        LP_DAC.pad_cfg.en_adc_mux_pad_0 = 0;
        LP_DAC.pad_cfg.dbuf_ls_pad_0 = 1;
        LP_DAC.pad_cfg.dbuf_range_ls_pad_0 = 1;
        LP_DAC.data_output_cfg.pad_0_data_sel = 0;
        LP_DAC.sintx_cfg.scale_1 = 0;
        LP_DAC.sintx_cfg.inv_1 = 0;
        LP_DAC.pad_cfg.xpd_pad_0 = 1;
    } else {
        LP_DAC.pad_cfg.hs_mode_pad_1 = 0;
        LP_DAC.pad_cfg.dsamp_ls_pad_1 = 0;
        LP_DAC.pad_cfg.en_adc_mux_pad_1 = 0;
        LP_DAC.pad_cfg.dbuf_ls_pad_1 = 1;
        LP_DAC.pad_cfg.dbuf_range_ls_pad_1 = 1;
        LP_DAC.data_output_cfg.pad_1_data_sel = 0;
        LP_DAC.sintx_cfg.scale_2 = 0;
        LP_DAC.sintx_cfg.inv_2 = 0;
        LP_DAC.pad_cfg.xpd_pad_1 = 1;
    }
    LP_DAC.sintx_cfg.sw_tone = 0;
    LP_DAC.sintx_timer_cfg.sintx_timer_target = VREF_SINTX_DIV;
    LP_DAC.sintx_timer_cfg.sintx_timer_en = 1;
}

static void dac_pad_off(uint8_t ch)
{
    if (ch == 0u) {
        LP_DAC.pad_cfg.xpd_pad_0 = 0;
        LP_DAC.pad_cfg.dbuf_ls_pad_0 = 0;
        LP_DAC.pad_cfg.dbuf_range_ls_pad_0 = 0;
    } else {
        LP_DAC.pad_cfg.xpd_pad_1 = 0;
        LP_DAC.pad_cfg.dbuf_ls_pad_1 = 0;
        LP_DAC.pad_cfg.dbuf_range_ls_pad_1 = 0;
    }
}

esp_err_t esp_foc_vref_generator_init(esp_foc_vref_generator_t **out,
                                      const esp_foc_vref_generator_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(out != NULL && cfg != NULL, ESP_ERR_INVALID_ARG, TAG, "null");
    ESP_RETURN_ON_FALSE(cfg->channel < VREF_CHANNELS, ESP_ERR_INVALID_ARG, TAG,
                        "dac channel %u", (unsigned)cfg->channel);
    ESP_RETURN_ON_FALSE(cfg->gpio == s_pad_gpio[cfg->channel], ESP_ERR_INVALID_ARG, TAG,
                        "dac channel %u is on gpio %d, not %d", (unsigned)cfg->channel,
                        s_pad_gpio[cfg->channel], cfg->gpio);
    esp_foc_vref_generator_t *g = &s_pool[cfg->channel];
    ESP_RETURN_ON_FALSE(!g->inited, ESP_ERR_INVALID_STATE, TAG,
                        "dac channel %u in use", (unsigned)cfg->channel);

    memset(g, 0, sizeof(*g));
    g->channel = cfg->channel;
    g->gpio = cfg->gpio;
    g->settle_ms = cfg->settle_ms != 0u ? cfg->settle_ms : VREF_SETTLE_MS_DEFAULT;

    if (s_active == 0u) {
        dac_clock_on();
    }
    s_active++;
    pad_as_analog(g->gpio);
    dac_set_code(g->channel, 0u);
    dac_pad_on(g->channel);
    g->inited = true;
    ESP_LOGI(TAG, "dac ch%u on gpio %d", (unsigned)g->channel, g->gpio);
    *out = g;
    return ESP_OK;
}

esp_err_t esp_foc_vref_generator_set_voltage(esp_foc_vref_generator_t *g, float volts)
{
    ESP_RETURN_ON_FALSE(g != NULL && g->inited, ESP_ERR_INVALID_ARG, TAG, "arg");
    if (volts < 0.0f) {
        volts = 0.0f;
    } else if (volts > ESP_FOC_VREF_FULL_SCALE_V) {
        volts = ESP_FOC_VREF_FULL_SCALE_V;
    }
    const uint32_t code =
        (uint32_t)(volts * ((float)VREF_CODE_MAX / ESP_FOC_VREF_FULL_SCALE_V) + 0.5f);
    dac_set_code(g->channel, code);
    esp_foc_sleep_ms(g->settle_ms);
    return ESP_OK;
}

esp_err_t esp_foc_vref_generator_deinit(esp_foc_vref_generator_t *g)
{
    ESP_RETURN_ON_FALSE(g != NULL && g->inited, ESP_ERR_INVALID_ARG, TAG, "arg");
    dac_pad_off(g->channel);
    g->inited = false;
    if (--s_active == 0u) {
        LP_DAC.sintx_timer_cfg.sintx_timer_en = 0;
        dac_clock_off();
    }
    return ESP_OK;
}

#else /* !ESP_FOC_VREF_LP_DAC */

esp_err_t esp_foc_vref_generator_init(esp_foc_vref_generator_t **out,
                                      const esp_foc_vref_generator_config_t *cfg)
{
    (void)out;
    (void)cfg;
    ESP_LOGE(TAG, "no on-chip DAC on this target");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t esp_foc_vref_generator_set_voltage(esp_foc_vref_generator_t *g, float volts)
{
    (void)g;
    (void)volts;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t esp_foc_vref_generator_deinit(esp_foc_vref_generator_t *g)
{
    (void)g;
    return ESP_ERR_NOT_SUPPORTED;
}

#endif /* ESP_FOC_VREF_LP_DAC */
