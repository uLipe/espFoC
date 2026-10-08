/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "espFoC/utils/esp_foc_q16.h"

/* GDMA RX channel the converter streams into; the ETM stop link keys on its EOF. */
#define ESP_FOC_ADC_DMA_RX_CH 2
/* Conversions per block, worst case (see the pattern choice in init). */
#define ESP_FOC_ADC_MAX_CONV 3

typedef void (*esp_foc_adc_dma_done_fn_t)(void *arg);

typedef struct {
    uint8_t shunt_count;
    int gpio_iu;
    int gpio_iv;
    int gpio_iw;
    float shunt_ohm;
    float amp_gain;
    /** Target PWM / DMA EOF rate; 0 → 20 kHz digi interval (legacy bring-up). */
    uint32_t pwm_hz;
    esp_foc_adc_dma_done_fn_t on_done;
    void *on_done_arg;
} esp_foc_adc_dma_cfg_t;

typedef struct esp_foc_adc_dma_sense esp_foc_adc_dma_sense_t;

esp_err_t esp_foc_adc_dma_sense_init(esp_foc_adc_dma_sense_t **out,
                                     const esp_foc_adc_dma_cfg_t *cfg);
void esp_foc_adc_dma_sense_deinit(esp_foc_adc_dma_sense_t *s);
esp_err_t esp_foc_adc_dma_sense_arm(esp_foc_adc_dma_sense_t *s);
void esp_foc_adc_dma_sense_disarm(esp_foc_adc_dma_sense_t *s);
/* Drop any partial frame so the next conversion is the first of one. Only with
 * no start pending or in flight. */
void esp_foc_adc_dma_sense_restart(esp_foc_adc_dma_sense_t *s);
/* TEZ-synced one-shot digi burst (convert_limit). No-op if SW continuous trigger. */
void esp_foc_adc_dma_sense_kick(esp_foc_adc_dma_sense_t *s);
void esp_foc_adc_dma_sense_fetch(esp_foc_adc_dma_sense_t *s,
                                 q16_t *iu, q16_t *iv, q16_t *iw);
/** Read last published sample without clearing sample_ready (ISR / i-limit). */
void esp_foc_adc_dma_sense_peek(esp_foc_adc_dma_sense_t *s,
                                q16_t *iu, q16_t *iv, q16_t *iw);
bool esp_foc_adc_dma_sense_sample_ready(esp_foc_adc_dma_sense_t *s);
void esp_foc_adc_dma_sense_calibrate(esp_foc_adc_dma_sense_t *s, int rounds);
uint8_t esp_foc_adc_dma_sense_shunt_count(const esp_foc_adc_dma_sense_t *s);
/* Conversions per block and their spacing; the block spans (n-1)*interval. */
uint8_t esp_foc_adc_dma_sense_conv_count(const esp_foc_adc_dma_sense_t *s);
/** Converter starts per PWM period, evenly spaced; each stops in hardware. */
uint8_t esp_foc_adc_dma_sense_starts(const esp_foc_adc_dma_sense_t *s);
/** A start to the sample that has to land on a zero-vector centre, in ns. */
uint32_t esp_foc_adc_dma_sense_lead_ns(const esp_foc_adc_dma_sense_t *s);
/** Nominal sample instant of conversion `i` of a frame, in ns from the timer peak. */
int32_t esp_foc_adc_dma_sense_conv_offset_ns(const esp_foc_adc_dma_sense_t *s, uint8_t i);
/*
 * Last block, one entry per conversion in sampling order: amps (offset removed,
 * hardware slot, no phase map) and the slot it belongs to. Consistent only from
 * the DMA done callback. Returns the number written.
 */
uint8_t esp_foc_adc_dma_sense_peek_conversions(esp_foc_adc_dma_sense_t *s,
                                               q16_t *amps, uint8_t *slot,
                                               uint8_t max);
