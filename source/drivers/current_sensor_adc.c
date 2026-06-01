/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * Unified ADC current-sense driver: adc_hal digi + target DMA fragment (no esp_adc).
 */

#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_intr_alloc.h"
#include "esp_clk_tree.h"
#include "esp_private/regi2c_ctrl.h"
#include "esp_private/sar_periph_ctrl.h"
#include "esp_private/adc_share_hw_ctrl.h"
#include "esp_private/gpio.h"
#include "soc/adc_periph.h"
#include "soc/soc_caps.h"
#include "soc/clk_tree_defs.h"
#include "hal/adc_hal.h"
#include "hal/adc_ll.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/biquad_q16.h"
#include "espFoC/utils/foc_math_q16.h"
#include "espFoC/driver_q16_local.h"
#include "espFoC/current_sensor_adc.h"
#include "espFoC/esp_foc_adc_cali_lut.h"
#include "isensor_adc_internal.h"

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
#include "esp_cache.h"
#endif

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define ADC_GET_CHANNEL(p_data)     ((p_data)->type1.channel)
#define ADC_GET_DATA(p_data)        ((p_data)->type1.data)
#else
#define ADC_GET_CHANNEL(p_data)     ((p_data)->type2.channel)
#define ADC_GET_DATA(p_data)        ((p_data)->type2.data)
#endif

#define ISENSOR_ADC_DMA_DESC_ALIGN  4
#define ISENSOR_ADC_FRAME_BYTES     (ESP_FOC_ISENSOR_ADC_NUM_CHANNELS * SOC_ADC_DIGI_RESULT_BYTES)
#define ISENSOR_ADC_CONVERT_LIMIT   2

static const char *TAG = "ESP_FOC_ISENSOR";

typedef struct {
    adc_hal_dma_ctx_t hal;
    adc_hal_digi_ctrlr_cfg_t hal_cfg;
    adc_digi_pattern_config_t patterns[ESP_FOC_ISENSOR_ADC_NUM_CHANNELS];
    uint8_t *rx_buf;
    uint32_t rx_desc_size;
    isensor_adc_dma_ctx_t dma_ctx;
    adc_channel_t channels[ESP_FOC_ISENSOR_ADC_NUM_CHANNELS];
    adc_unit_t unit;
    adc_atten_t atten;
    float adc_to_current_scale;
    q16_t adc_to_current_scale_q16;
    int16_t cali_lut[ESP_FOC_ISENSOR_ADC_NUM_CHANNELS][ESP_FOC_ISENSOR_ADC_LUT_SIZE];
    int32_t latest_raw[ESP_FOC_ISENSOR_ADC_NUM_CHANNELS];
    int32_t filtered_count[ESP_FOC_ISENSOR_ADC_NUM_CHANNELS];
    esp_foc_biquad_q16_t bq[ESP_FOC_ISENSOR_ADC_NUM_CHANNELS];
    float offsets[ESP_FOC_ISENSOR_ADC_NUM_CHANNELS];
    int32_t offset_counts[ESP_FOC_ISENSOR_ADC_NUM_CHANNELS];
    volatile q16_t *publish_alpha;
    volatile q16_t *publish_beta;
    volatile q16_t *publish_iu;
    volatile q16_t *publish_iv;
    esp_foc_isensor_t interface;
    esp_foc_isensor_adc_trigger_t trigger;
    esp_foc_isensor_adc_state_t state;
    isensor_callback_t callback;
    void *user_data;
    bool started;
} isensor_adc_t;

DRAM_ATTR static isensor_adc_t s_isensor;
static bool s_adc_initialized;

static const float adc_to_volts = 3.1f / 4096.0f;

static adc_bitwidth_t isensor_adc_digi_bitwidth(void)
{
#if CONFIG_IDF_TARGET_ESP32
    return ADC_BITWIDTH_12;
#else
    return SOC_ADC_DIGI_MAX_BITWIDTH;
#endif
}

static adc_atten_t isensor_adc_default_atten(void)
{
#if CONFIG_IDF_TARGET_ESP32
    return ADC_ATTEN_DB_11;
#else
    return ADC_ATTEN_DB_12;
#endif
}

static void isensor_adc_digi_apply_convert_limit(void)
{
    adc_ll_digi_set_convert_limit_num(ISENSOR_ADC_CONVERT_LIMIT);
    adc_ll_digi_convert_limit_enable(true);
}

static esp_err_t isensor_adc_gpio_init(adc_unit_t unit, uint32_t chan_mask)
{
    while (chan_mask) {
        int ch = __builtin_ctz(chan_mask);
        int8_t io = adc_channel_io_map[unit][ch];
        if (io < 0) {
            return ESP_ERR_INVALID_ARG;
        }
        gpio_config_as_analog(io);
        chan_mask &= ~(1U << ch);
    }
    return ESP_OK;
}

static inline void isensor_adc_publish_alpha_beta(isensor_adc_t *obj)
{
    const int32_t adc_rng = 2048;
    int32_t d0 = obj->filtered_count[0] - obj->offset_counts[0];
    int32_t d1 = obj->filtered_count[1] - obj->offset_counts[1];
    d0 = esp_foc_clamp_int32(d0, -adc_rng, adc_rng);
    d1 = esp_foc_clamp_int32(d1, -adc_rng, adc_rng);
    q16_t iu = q16_mul(esp_foc_q16_from_adc_diff_clamped(d0, adc_rng),
                       obj->adc_to_current_scale_q16);
    q16_t iv = q16_mul(esp_foc_q16_from_adc_diff_clamped(d1, adc_rng),
                       obj->adc_to_current_scale_q16);
    q16_t iw = q16_sub(0, q16_add(iu, iv));
    q16_t alpha, beta;
    q16_clarke(iu, iv, iw, &alpha, &beta);
    *obj->publish_alpha = alpha;
    *obj->publish_beta = beta;
    if (obj->publish_iu != NULL) {
        *obj->publish_iu = iu;
    }
    if (obj->publish_iv != NULL) {
        *obj->publish_iv = iv;
    }
}

static void IRAM_ATTR isensor_adc_process_frame(isensor_adc_t *isensor, const uint8_t *frame, uint32_t size)
{
    if (frame == NULL || size < ISENSOR_ADC_FRAME_BYTES) {
        return;
    }

    adc_digi_output_data_t *p = (adc_digi_output_data_t *)frame;
    for (int i = 0; i < ESP_FOC_ISENSOR_ADC_NUM_CHANNELS; i++) {
        int32_t raw = (int32_t)ADC_GET_DATA(p);
        raw = esp_foc_adc_cali_lut_apply(isensor->cali_lut[i], raw);
        isensor->latest_raw[i] = raw;
        q16_t y = esp_foc_biquad_q16_update(&isensor->bq[i], (q16_t)(raw << 16));
        isensor->filtered_count[i] = y >> 16;
        p++;
    }

    if (isensor->publish_alpha != NULL && isensor->publish_beta != NULL) {
        isensor_adc_publish_alpha_beta(isensor);
    }

    if (isensor->callback != NULL) {
        isensor->callback(isensor->user_data);
    }
}

static void IRAM_ATTR isensor_adc_dma_done(void *arg)
{
    isensor_adc_t *isensor = (isensor_adc_t *)arg;
    adc_hal_dma_desc_status_t status;
    uint8_t *finished_buffer = NULL;
    uint32_t finished_size = 0;

    while (1) {
        status = adc_hal_get_reading_result(&isensor->hal, isensor->dma_ctx.eof_desc_addr,
                                            &finished_buffer, &finished_size);
        if (status != ADC_HAL_DMA_DESC_VALID) {
            break;
        }
#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
        esp_cache_msync(finished_buffer, finished_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
#endif
        isensor_adc_process_frame(isensor, finished_buffer, finished_size);
    }

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
    esp_cache_msync(isensor->hal.rx_desc, isensor->rx_desc_size,
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_INVALIDATE);
#endif

    adc_hal_digi_enable(false);
    adc_hal_digi_connect(false);
    isensor->state = ESP_FOC_ISENSOR_ADC_STATE_IDLE;
}

static esp_err_t isensor_adc_hw_start(isensor_adc_t *isensor)
{
    ANALOG_CLOCK_ENABLE();

    ADC_BUS_CLK_ATOMIC() {
        adc_ll_reset_register();
    }

    sar_periph_ctrl_adc_continuous_power_acquire();
    adc_lock_acquire(isensor->unit);

#if SOC_ADC_CALIBRATION_V1_SUPPORTED
    adc_hal_calibration_init(isensor->unit);
    adc_set_hw_calibration_code(isensor->unit, isensor->atten);
#endif

    adc_hal_set_controller(isensor->unit, ADC_HAL_CONTINUOUS_READ_MODE);

#if !CONFIG_IDF_TARGET_ESP32
    ESP_ERROR_CHECK(esp_clk_tree_enable_src((soc_module_clk_t)isensor->hal_cfg.clk_src, true));
#endif

    adc_hal_digi_init(&isensor->hal);
    adc_hal_digi_controller_config(&isensor->hal, &isensor->hal_cfg);
    isensor_adc_digi_apply_convert_limit();
    adc_hal_digi_enable(false);
    adc_hal_digi_connect(false);

    isensor_adc_dma_stop(&isensor->dma_ctx);
    adc_hal_digi_reset();
    adc_hal_digi_dma_link(&isensor->hal, isensor->rx_buf);

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
    esp_cache_msync(isensor->hal.rx_desc, isensor->rx_desc_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
#endif

    isensor_adc_dma_start(&isensor->dma_ctx, isensor->hal.rx_desc);
    adc_hal_digi_connect(true);
    adc_hal_digi_enable(true);
    isensor->state = ESP_FOC_ISENSOR_ADC_STATE_BUSY;
    isensor->started = true;
    return ESP_OK;
}

static esp_err_t isensor_adc_setup_hal(isensor_adc_t *isensor)
{
    uint32_t clk_hz = 0;
    esp_clk_tree_src_get_freq_hz(ADC_DIGI_CLK_SRC_DEFAULT, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &clk_hz);

    isensor->hal_cfg.adc_pattern = isensor->patterns;
    isensor->hal_cfg.adc_pattern_len = ESP_FOC_ISENSOR_ADC_NUM_CHANNELS;
    isensor->hal_cfg.sample_freq_hz = ESP_FOC_ISENSOR_ADC_PATTERN_HZ;
    isensor->hal_cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    isensor->hal_cfg.clk_src = ADC_DIGI_CLK_SRC_DEFAULT;
    isensor->hal_cfg.clk_src_freq_hz = clk_hz;

    uint32_t chan_mask = 0;
    for (int i = 0; i < ESP_FOC_ISENSOR_ADC_NUM_CHANNELS; i++) {
        isensor->patterns[i].atten = isensor->atten;
        isensor->patterns[i].channel = isensor->channels[i] & 0x7;
        isensor->patterns[i].unit = isensor->unit;
        isensor->patterns[i].bit_width = isensor_adc_digi_bitwidth();
        chan_mask |= BIT(isensor->patterns[i].channel);
        ESP_LOGI(TAG, "pattern[%d] unit=%d ch=%d atten=%d", i,
                 (int)isensor->patterns[i].unit, (int)isensor->patterns[i].channel,
                 (int)isensor->patterns[i].atten);
    }

    adc_hal_dma_config_t dma_cfg = {
        .eof_desc_num = 1,
        .eof_step = 1,
        .eof_num = ESP_FOC_ISENSOR_ADC_NUM_CHANNELS,
    };
    adc_hal_dma_ctx_config(&isensor->hal, &dma_cfg);

    isensor->rx_buf = heap_caps_calloc(1, ISENSOR_ADC_FRAME_BYTES,
                                       MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (isensor->rx_buf == NULL) {
        return ESP_ERR_NO_MEM;
    }

    isensor->hal.rx_desc = heap_caps_aligned_calloc(ISENSOR_ADC_DMA_DESC_ALIGN, 1,
                                                    sizeof(dma_descriptor_t),
                                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (isensor->hal.rx_desc == NULL) {
        return ESP_ERR_NO_MEM;
    }

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
    uint32_t line = 4;
    isensor->rx_desc_size = (sizeof(dma_descriptor_t) + line - 1) & ~(line - 1);
#else
    isensor->rx_desc_size = sizeof(dma_descriptor_t);
#endif

    ESP_RETURN_ON_ERROR(isensor_adc_gpio_init(isensor->unit, chan_mask), TAG, "gpio init failed");
    ESP_RETURN_ON_ERROR(isensor_adc_dma_init(&isensor->dma_ctx, isensor_adc_dma_done, isensor),
                        TAG, "dma init failed");
    return ESP_OK;
}

static void fetch_isensors(esp_foc_isensor_t *self, isensor_values_t *values)
{
    isensor_adc_t *obj = __containerof(self, isensor_adc_t, interface);
    const int32_t adc_rng = 2048;

    int32_t d0 = obj->filtered_count[0] - obj->offset_counts[0];
    int32_t d1 = obj->filtered_count[1] - obj->offset_counts[1];
    d0 = esp_foc_clamp_int32(d0, -adc_rng, adc_rng);
    d1 = esp_foc_clamp_int32(d1, -adc_rng, adc_rng);

    q16_t iu0 = q16_mul(esp_foc_q16_from_adc_diff_clamped(d0, adc_rng), obj->adc_to_current_scale_q16);
    q16_t iv0 = q16_mul(esp_foc_q16_from_adc_diff_clamped(d1, adc_rng), obj->adc_to_current_scale_q16);
    q16_t zero = 0;
    q16_t iw0 = q16_sub(zero, q16_add(iu0, iv0));

    values->iu_axis_0 = iu0;
    values->iv_axis_0 = iv0;
    values->iw_axis_0 = iw0;
    values->iu_axis_1 = 0;
    values->iv_axis_1 = 0;
    values->iw_axis_1 = 0;
}

static void sample_isensors(esp_foc_isensor_t *self)
{
    isensor_adc_t *obj = __containerof(self, isensor_adc_t, interface);

    if (obj->trigger != ESP_FOC_ISENSOR_ADC_TRIG_SOFTWARE) {
        return;
    }
    if (obj->state == ESP_FOC_ISENSOR_ADC_STATE_BUSY) {
        return;
    }

    if (!obj->started) {
        isensor_adc_hw_start(obj);
        return;
    }

    isensor_adc_dma_reset(&obj->dma_ctx);
    adc_hal_digi_reset();
    adc_hal_digi_dma_link(&obj->hal, obj->rx_buf);
#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
    esp_cache_msync(obj->hal.rx_desc, obj->rx_desc_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
#endif
    isensor_adc_dma_start(&obj->dma_ctx, obj->hal.rx_desc);
    adc_hal_digi_connect(true);
    adc_hal_digi_enable(true);
    obj->state = ESP_FOC_ISENSOR_ADC_STATE_BUSY;
}

static void calibrate_isensors(esp_foc_isensor_t *self, int calibration_rounds)
{
    isensor_adc_t *obj = __containerof(self, isensor_adc_t, interface);

    if (obj->trigger != ESP_FOC_ISENSOR_ADC_TRIG_SOFTWARE) {
        return;
    }

    isensor_values_t val;
    esp_foc_sleep_ms(100);

    for (int oi = 0; oi < ESP_FOC_ISENSOR_ADC_NUM_CHANNELS; oi++) {
        obj->offsets[oi] = 0.0f;
        obj->offset_counts[oi] = 0;
    }

    for (int i = 0; i < calibration_rounds; i++) {
        self->sample_isensors(self);
        esp_foc_sleep_ms(10);
        for (int ch = 0; ch < ESP_FOC_ISENSOR_ADC_NUM_CHANNELS; ch++) {
            obj->offsets[ch] += (float)obj->latest_raw[ch];
        }
    }

    for (int ch = 0; ch < ESP_FOC_ISENSOR_ADC_NUM_CHANNELS; ch++) {
        obj->offsets[ch] /= calibration_rounds;
        obj->offset_counts[ch] = (int32_t)lroundf(obj->offsets[ch]);
        esp_foc_biquad_q16_reset(&obj->bq[ch]);
        obj->filtered_count[ch] = obj->latest_raw[ch];
    }

    ESP_LOGI(TAG, "ADC calibrated, offsets: %f, %f", obj->offsets[0], obj->offsets[1]);
    esp_foc_sleep_ms(100);

    self->sample_isensors(self);
    esp_foc_sleep_ms(10);
    self->fetch_isensors(self, &val);
    ESP_LOGI(TAG, "No-current test: iu=%f iv=%f iw=%f",
             q16_to_float(val.iu_axis_0), q16_to_float(val.iv_axis_0), q16_to_float(val.iw_axis_0));
}

static void set_callback(esp_foc_isensor_t *self, isensor_callback_t cb, void *arg)
{
    isensor_adc_t *obj = __containerof(self, isensor_adc_t, interface);
    esp_foc_critical_enter();
    obj->callback = cb;
    obj->user_data = arg;
    esp_foc_critical_leave();
}

static void set_filter_cutoff(esp_foc_isensor_t *self, float fc_hz, float fs_hz)
{
    isensor_adc_t *obj = __containerof(self, isensor_adc_t, interface);
    esp_foc_critical_enter();
    for (int i = 0; i < ESP_FOC_ISENSOR_ADC_NUM_CHANNELS; ++i) {
        esp_foc_biquad_butterworth_lpf_design_q16(&obj->bq[i], fc_hz, fs_hz);
    }
    esp_foc_critical_leave();
}

static void set_publish_targets(esp_foc_isensor_t *self,
                                q16_t *i_alpha_target,
                                q16_t *i_beta_target,
                                q16_t *i_u_target,
                                q16_t *i_v_target)
{
    isensor_adc_t *obj = __containerof(self, isensor_adc_t, interface);
    esp_foc_critical_enter();
    obj->publish_alpha = (volatile q16_t *)i_alpha_target;
    obj->publish_beta = (volatile q16_t *)i_beta_target;
    obj->publish_iu = (volatile q16_t *)i_u_target;
    obj->publish_iv = (volatile q16_t *)i_v_target;
    esp_foc_critical_leave();
}

esp_foc_err_t esp_foc_isensor_adc_set_trigger(esp_foc_isensor_t *isensor,
                                              esp_foc_isensor_adc_trigger_t mode)
{
    if (isensor == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if (mode == ESP_FOC_ISENSOR_ADC_TRIG_ETM) {
        return ESP_FOC_ERR_NOT_SUPPORTED;
    }
    isensor_adc_t *obj = __containerof(isensor, isensor_adc_t, interface);
    obj->trigger = mode;
    return ESP_FOC_OK;
}

esp_foc_isensor_t *isensor_adc_new(esp_foc_isensor_adc_config_t *config)
{
    if (config == NULL) {
        return NULL;
    }
    if (s_adc_initialized) {
        return &s_isensor.interface;
    }

    if (config->unit != ADC_UNIT_1) {
        ESP_LOGE(TAG, "only ADC1 supported in v1");
        return NULL;
    }
    if (config->amp_gain <= 0.0f || config->shunt_resistance <= 0.0f) {
        ESP_LOGE(TAG, "invalid amp_gain or shunt_resistance");
        return NULL;
    }

    memset(&s_isensor, 0, sizeof(s_isensor));
    s_isensor.unit = config->unit;
    s_isensor.atten = isensor_adc_default_atten();
    s_isensor.channels[0] = config->channels[0];
    s_isensor.channels[1] = config->channels[1];
    s_isensor.trigger = ESP_FOC_ISENSOR_ADC_TRIG_SOFTWARE;
    s_isensor.state = ESP_FOC_ISENSOR_ADC_STATE_IDLE;

    s_isensor.adc_to_current_scale = adc_to_volts * (1.0f / (config->amp_gain * config->shunt_resistance));
    s_isensor.adc_to_current_scale_q16 = q16_from_float(2048.0f * s_isensor.adc_to_current_scale);

    for (int ch = 0; ch < ESP_FOC_ISENSOR_ADC_NUM_CHANNELS; ch++) {
        esp_foc_adc_cali_lut_build(s_isensor.unit, s_isensor.channels[ch], s_isensor.atten,
                                   s_isensor.cali_lut[ch], ESP_FOC_ISENSOR_ADC_LUT_SIZE);
        esp_foc_biquad_q16_set_bypass(&s_isensor.bq[ch]);
    }

    s_isensor.interface.fetch_isensors = fetch_isensors;
    s_isensor.interface.sample_isensors = sample_isensors;
    s_isensor.interface.calibrate_isensors = calibrate_isensors;
    s_isensor.interface.set_isensor_callback = set_callback;
    s_isensor.interface.set_filter_cutoff = set_filter_cutoff;
    s_isensor.interface.set_publish_targets = set_publish_targets;

    adc_apb_periph_claim();

    esp_err_t err = isensor_adc_setup_hal(&s_isensor);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC HAL setup failed: %d", err);
        return NULL;
    }

    isensor_adc_hw_start(&s_isensor);
    esp_foc_sleep_ms(10);
    s_adc_initialized = true;
    return &s_isensor.interface;
}
