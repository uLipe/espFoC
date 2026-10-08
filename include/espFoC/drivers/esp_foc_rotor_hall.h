/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "espFoC/drivers/esp_foc_rotor_sensor.h"
#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Where an edge's timestamp comes from. ETM+TIMG latches the instant in
 * hardware; GPIO_IRQ reads a microsecond clock at ISR entry and therefore
 * carries the ISR's own jitter. Both feed the same estimator.
 */
typedef enum {
    ESP_FOC_HALL_TS_ETM_TIMG = 0,
    ESP_FOC_HALL_TS_GPIO_IRQ = 1,
} esp_foc_hall_ts_kind_t;

typedef struct {
    /* Hall lines A, B, C. Any GPIO; avoid strapping pins. */
    int gpio[3];
    unsigned pole_pairs;
    /* Rate at which esp_foc_rotor_sensor_step() will be called [Hz]. */
    uint32_t pwm_hz;

    esp_foc_hall_ts_kind_t ts_kind;
    /* ETM_TIMG: three free ETM channel indices, supplied by the application
     * exactly as etm_link's single channel is. No allocator: 50 channels
     * exist and four are spoken for. */
    int etm_channel[3];
    int timer_group;
    /*
     * ETM_TIMG. Enforces the init-order contract described below. Leave true
     * in any application that also creates an inverter; clear it only where
     * the hall is legitimately the first ETM user.
     */
    bool require_etm_ready;
    /* GPIO_IRQ: interrupt level, must stay below the PWM ISR's. 0 → 1. */
    int irq_level;

    /*
     * Which way round the hall sequence runs. +1 or 0 for the natural order,
     * -1 to reverse it. A discovered map subsumes this; it exists for bring-up
     * before one has been discovered.
     */
    int dir_sign;

    /*
     * Commissioning table: electrical angle [rad] of the boundary that opens
     * each sector, indexed by the sector's position in the hall sequence.
     * Absorbs both the wiring permutation and the mounting offset, which is
     * why those are not separate fields.
     *
     * With have_map false the driver falls back to the nominal k·60° ladder,
     * so θe is right up to an arbitrary constant — enough to prove decode,
     * direction, timestamping and extrapolation on a bench, and reported
     * honestly through ESP_FOC_ROTOR_CAP_NEEDS_MAP.
     */
    float theta_edge_rad[6];
    bool have_map;

    /* Estimator tuning. 0 selects the defaults. */
    float lambda_theta;
    float lambda_omega;
    float standstill_ms;
} esp_foc_rotor_hall_config_t;

typedef struct {
    uint32_t edges;
    /* 000 or 111: an open line or a dead sensor. */
    uint32_t illegal_code;
    /* More than one bit moved at once — one or more edges were missed. */
    uint32_t multi_bit;
    /*
     * The code changed but the capture register did not. Signature of an ETM
     * channel that was cleared out from under us, and equally of a wrong
     * channel index or a swapped pin, which is what bring-up actually hits.
     */
    uint32_t capture_lost;
    /* A capture fired with the code unchanged: a glitch that came back. */
    uint32_t spurious;
    /* Estimator verdicts, surfaced here so one call reports the whole path. */
    uint32_t bounce;
    uint32_t stale;
    uint32_t clamped;
    /* Longest gap between edges seen, in timestamp ticks. */
    uint64_t worst_dticks;
    bool etm_cold_start;
    bool ts_healthy;
} esp_foc_rotor_hall_health_t;

/**
 * Hall rotor sensor: electrical angle from three 60°-quantized lines, made
 * continuous by esp_foc_rotor_est.
 *
 * Reports ELEC_ABS | PREDICT (| NEEDS_MAP until a map is installed). It does
 * not report MECH_ABS: θm is published as θe/pp and is only true modulo
 * 2π/pp, because hall is absolute in electrical and not in mechanical.
 *
 * fetch(), fetch_start() and calibrate_offset() answer ESP_ERR_NOT_SUPPORTED.
 * The driver is event-driven, so there is no blocking sample to take, and the
 * offset's job belongs to map discovery rather than to a zeroing routine.
 *
 * ## Init order, when the ETM strategy is used
 *
 * **Create the inverter before this sensor.** The reason is an asymmetry in
 * the two ETM bring-up calls: enabling the bus clock is one idempotent bit,
 * while resetting the peripheral clears all 50 channels. The inverter's ADC
 * trigger performs that reset; this driver only ever enables the clock and
 * never resets. In that order the destructive reset lands before any hall
 * channel exists, and no coordination between the two drivers is needed.
 *
 * The reverse order is the only broken one. init() detects it by reading
 * whether anything has enabled the ETM clock yet — a question about the
 * peripheral, not about the inverter, so the two drivers stay independent —
 * and with require_etm_ready set it refuses with ESP_ERR_INVALID_STATE
 * instead of silently producing a wrong angle.
 *
 * An inverter created *later*, while this sensor is already running, is caught
 * at runtime instead: step() sees the code change while the capture register
 * does not, and counts capture_lost. esp_foc_rotor_hall_rearm() reprograms the
 * channels, which also gives an application that genuinely wants the reverse
 * order a legitimate path — create both, then re-arm.
 */
esp_foc_rotor_sensor_t *esp_foc_rotor_hall_acquire(unsigned index);
void esp_foc_rotor_hall_release(esp_foc_rotor_sensor_t *s);

esp_err_t esp_foc_rotor_hall_init(esp_foc_rotor_sensor_t *s,
                                  const esp_foc_rotor_hall_config_t *cfg);
esp_err_t esp_foc_rotor_hall_deinit(esp_foc_rotor_sensor_t *s);

/** Reprogram the timestamp hardware after something else cleared it. */
esp_err_t esp_foc_rotor_hall_rearm(esp_foc_rotor_sensor_t *s);

/** Install a discovered map; clears NEEDS_MAP. Angles in electrical radians. */
esp_err_t esp_foc_rotor_hall_set_map(esp_foc_rotor_sensor_t *s,
                                     const float theta_edge_rad[6]);

void esp_foc_rotor_hall_get_health(const esp_foc_rotor_sensor_t *s,
                                   esp_foc_rotor_hall_health_t *out);

/** Raw 3-bit line code, for bring-up logs. */
uint8_t esp_foc_rotor_hall_raw_code(const esp_foc_rotor_sensor_t *s);

#ifdef __cplusplus
}
#endif
