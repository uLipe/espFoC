/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#include "sdkconfig.h"
#include "espFoC/esp_foc_in_the_loop.h"

#include <stddef.h>
#include <string.h>

#include "espFoC/esp_foc.h"
#include "espFoC/utils/biquad_q16.h"
#include "espFoC/utils/foc_math_q16.h"
#include "esp_attr.h"
#include "esp_log.h"

#include "esp_foc_itl_plant.h"
#include "esp_foc_itl_tick.h"

static const char *TAG = "foc_itl";

#define FOC_ITL_PLANT_RUNNER_PRIORITY  2
#define FOC_ITL_PLANT_JOIN_MS          50

static void plant_runner_join(esp_foc_event_handle_t ev)
{
    if (ev == NULL) {
        return;
    }
    for (int ms = 0; ms < FOC_ITL_PLANT_JOIN_MS && esp_foc_runner_is_alive(ev); ++ms) {
        esp_foc_runner_wake(ev);
        esp_foc_sleep_ms(1);
    }
}

typedef struct {
    esp_foc_itl_plant_t plant;
    int64_t enc_accum;

    volatile q16_t duty_u;
    volatile q16_t duty_v;
    volatile q16_t duty_w;

    esp_foc_biquad_q16_t bq_u;
    esp_foc_biquad_q16_t bq_v;

    volatile q16_t *publish_alpha;
    volatile q16_t *publish_beta;
    volatile q16_t *publish_iu;
    volatile q16_t *publish_iv;

    volatile esp_foc_inverter_callback_t inverter_cb;
    volatile void *inverter_cb_arg;

    uint32_t pwm_hz;
    q16_t vdc_q16;

    esp_foc_event_handle_t plant_ev;
    bool alive;

    esp_foc_inverter_t inverter;
    esp_foc_isensor_t isensor;
    esp_foc_rotor_sensor_t rotor;
} esp_foc_itl_ctx_t;

static esp_foc_itl_ctx_t s_ctx;
static bool s_created;

static uint32_t fitl_pwm_hz_from_config(uint32_t cfg_hz)
{
    if (cfg_hz != 0u) {
        return cfg_hz;
    }
#if defined(CONFIG_FOC_ITL_PWM_10KHZ)
    return 10000u;
#else
    return (uint32_t)CONFIG_ESP_FOC_PWM_RATE_HZ;
#endif
}

static void snapshot_duties(esp_foc_itl_ctx_t *ctx, q16_t *du, q16_t *dv, q16_t *dw)
{
    esp_foc_critical_enter();
    *du = ctx->duty_u;
    *dv = ctx->duty_v;
    *dw = ctx->duty_w;
    esp_foc_critical_leave();
}

static void publish_currents(esp_foc_itl_ctx_t *ctx, q16_t iu, q16_t iv)
{
    q16_t iw = q16_sub(q16_sub(0, iu), iv);
    q16_t alpha;
    q16_t beta;
    q16_clarke(iu, iv, iw, &alpha, &beta);

    if (ctx->publish_alpha != NULL) {
        *ctx->publish_alpha = alpha;
    }
    if (ctx->publish_beta != NULL) {
        *ctx->publish_beta = beta;
    }
    if (ctx->publish_iu != NULL) {
        *ctx->publish_iu = iu;
    }
    if (ctx->publish_iv != NULL) {
        *ctx->publish_iv = iv;
    }
}

static void plant_task_fn(void *arg)
{
    esp_foc_itl_ctx_t *ctx = (esp_foc_itl_ctx_t *)arg;

    while (ctx->alive) {
        esp_foc_wait_notifier();

        q16_t du;
        q16_t dv;
        q16_t dw;
        snapshot_duties(ctx, &du, &dv, &dw);

        q16_t vu;
        q16_t vv;
        q16_t vw;
        esp_foc_itl_plant_duties_to_phase_volts(du, dv, dw, &ctx->plant, &vu, &vv, &vw);
        esp_foc_itl_plant_step(&ctx->plant, vu, vv, vw);

        const q16_t iu = esp_foc_biquad_q16_update(&ctx->bq_u, ctx->plant.iu_q16);
        const q16_t iv = esp_foc_biquad_q16_update(&ctx->bq_v, ctx->plant.iv_q16);
        publish_currents(ctx, iu, iv);

        const int32_t ticks = esp_foc_itl_plant_encoder_ticks(&ctx->plant);
        ctx->enc_accum = (int64_t)ticks;
    }

    esp_foc_runner_delete_self();
}

static void IRAM_ATTR tick_cb(void *arg)
{
    esp_foc_itl_ctx_t *ctx = (esp_foc_itl_ctx_t *)arg;
    esp_foc_inverter_callback_t cb = ctx->inverter_cb;
    void *cb_arg = (void *)ctx->inverter_cb_arg;

    if (cb != NULL) {
        cb(cb_arg);
    }

    if (ctx->plant_ev != NULL) {
        esp_foc_send_notification_from_isr(ctx->plant_ev);
    }
}

static void itl_set_inverter_callback(esp_foc_inverter_t *self,
                                      esp_foc_inverter_callback_t callback,
                                      void *argument)
{
    esp_foc_itl_ctx_t *ctx = (esp_foc_itl_ctx_t *)((char *)self - offsetof(esp_foc_itl_ctx_t, inverter));
    esp_foc_critical_enter();
    ctx->inverter_cb = callback;
    ctx->inverter_cb_arg = argument;
    esp_foc_critical_leave();
}

static q16_t itl_get_dc_link_voltage(esp_foc_inverter_t *self)
{
    esp_foc_itl_ctx_t *ctx = (esp_foc_itl_ctx_t *)((char *)self - offsetof(esp_foc_itl_ctx_t, inverter));
    return ctx->vdc_q16;
}

static void itl_set_duties(esp_foc_inverter_t *self, q16_t duty_a, q16_t duty_b, q16_t duty_c)
{
    esp_foc_itl_ctx_t *ctx = (esp_foc_itl_ctx_t *)((char *)self - offsetof(esp_foc_itl_ctx_t, inverter));
    esp_foc_critical_enter();
    ctx->duty_u = duty_a;
    ctx->duty_v = duty_b;
    ctx->duty_w = duty_c;
    esp_foc_critical_leave();
}

static uint32_t itl_get_inverter_pwm_rate(esp_foc_inverter_t *self)
{
    esp_foc_itl_ctx_t *ctx = (esp_foc_itl_ctx_t *)((char *)self - offsetof(esp_foc_itl_ctx_t, inverter));
    return ctx->pwm_hz;
}

static void itl_enable(esp_foc_inverter_t *self)
{
    (void)self;
}

static void itl_disable(esp_foc_inverter_t *self)
{
    (void)self;
}

static void itl_fetch_isensors(esp_foc_isensor_t *self, isensor_values_t *values)
{
    (void)self;
    (void)values;
}

static void itl_sample_isensors(esp_foc_isensor_t *self)
{
    (void)self;
}

static void itl_calibrate_isensors(esp_foc_isensor_t *self, int calibration_rounds)
{
    (void)self;
    (void)calibration_rounds;
}

static void itl_set_isensor_callback(esp_foc_isensor_t *self, isensor_callback_t cb, void *param)
{
    (void)self;
    (void)cb;
    (void)param;
}

static void itl_set_filter_cutoff(esp_foc_isensor_t *self, float fc_hz, float fs_hz)
{
    esp_foc_itl_ctx_t *ctx = (esp_foc_itl_ctx_t *)((char *)self - offsetof(esp_foc_itl_ctx_t, isensor));
    esp_foc_critical_enter();
    esp_foc_biquad_butterworth_lpf_design_q16(&ctx->bq_u, fc_hz, fs_hz);
    esp_foc_biquad_butterworth_lpf_design_q16(&ctx->bq_v, fc_hz, fs_hz);
    esp_foc_critical_leave();
}

static void itl_set_publish_targets(esp_foc_isensor_t *self,
                                    q16_t *i_alpha_target,
                                    q16_t *i_beta_target,
                                    q16_t *i_u_target,
                                    q16_t *i_v_target)
{
    esp_foc_itl_ctx_t *ctx = (esp_foc_itl_ctx_t *)((char *)self - offsetof(esp_foc_itl_ctx_t, isensor));
    esp_foc_critical_enter();
    ctx->publish_alpha = (volatile q16_t *)i_alpha_target;
    ctx->publish_beta = (volatile q16_t *)i_beta_target;
    ctx->publish_iu = (volatile q16_t *)i_u_target;
    ctx->publish_iv = (volatile q16_t *)i_v_target;
    esp_foc_critical_leave();
}

static void itl_rotor_set_to_zero(esp_foc_rotor_sensor_t *self)
{
    esp_foc_itl_ctx_t *ctx = (esp_foc_itl_ctx_t *)((char *)self - offsetof(esp_foc_itl_ctx_t, rotor));
    esp_foc_critical_enter();
    esp_foc_itl_plant_reset_parked(&ctx->plant);
    ctx->enc_accum = 0;
    esp_foc_critical_leave();
}

static uint32_t itl_rotor_get_cpr(esp_foc_rotor_sensor_t *self)
{
    (void)self;
    return ESP_FOC_ITL_CPR;
}

static q16_t itl_rotor_read_counts(esp_foc_rotor_sensor_t *self)
{
    esp_foc_itl_ctx_t *ctx = (esp_foc_itl_ctx_t *)((char *)self - offsetof(esp_foc_itl_ctx_t, rotor));
    const int32_t ticks = esp_foc_itl_plant_encoder_ticks(&ctx->plant);
    return q16_from_int(ticks);
}

static int64_t itl_rotor_read_accumulated(esp_foc_rotor_sensor_t *self)
{
    esp_foc_itl_ctx_t *ctx = (esp_foc_itl_ctx_t *)((char *)self - offsetof(esp_foc_itl_ctx_t, rotor));
    return ctx->enc_accum;
}

static void itl_rotor_set_simulation_count(esp_foc_rotor_sensor_t *self, q16_t increment)
{
    (void)self;
    (void)increment;
}

static void itl_rotor_set_zero_offset_raw_12b(esp_foc_rotor_sensor_t *self, uint16_t raw_angle)
{
    (void)self;
    (void)raw_angle;
}

static uint16_t itl_rotor_get_zero_offset_12b(esp_foc_rotor_sensor_t *self)
{
    (void)self;
    return 0;
}

static void wire_vtables(esp_foc_itl_ctx_t *ctx)
{
    ctx->inverter.set_inverter_callback = itl_set_inverter_callback;
    ctx->inverter.get_dc_link_voltage = itl_get_dc_link_voltage;
    ctx->inverter.set_duties = itl_set_duties;
    ctx->inverter.get_inverter_pwm_rate = itl_get_inverter_pwm_rate;
    ctx->inverter.enable = itl_enable;
    ctx->inverter.disable = itl_disable;

    ctx->isensor.fetch_isensors = itl_fetch_isensors;
    ctx->isensor.sample_isensors = itl_sample_isensors;
    ctx->isensor.calibrate_isensors = itl_calibrate_isensors;
    ctx->isensor.set_isensor_callback = itl_set_isensor_callback;
    ctx->isensor.set_filter_cutoff = itl_set_filter_cutoff;
    ctx->isensor.set_publish_targets = itl_set_publish_targets;

    ctx->rotor.set_to_zero = itl_rotor_set_to_zero;
    ctx->rotor.get_counts_per_revolution = itl_rotor_get_cpr;
    ctx->rotor.read_counts = itl_rotor_read_counts;
    ctx->rotor.read_accumulated_counts_i64 = itl_rotor_read_accumulated;
    ctx->rotor.set_simulation_count = itl_rotor_set_simulation_count;
    ctx->rotor.set_zero_offset_raw_12b = itl_rotor_set_zero_offset_raw_12b;
    ctx->rotor.get_zero_offset_12b = itl_rotor_get_zero_offset_12b;
}

static esp_foc_itl_plant_params_t params_from_config(const esp_foc_in_the_loop_config_t *cfg)
{
    esp_foc_itl_plant_params_t p = {
        .r_ohm = cfg->r_ohm,
        .l_henry = cfg->l_henry,
        .j_kgm2 = cfg->j_kgm2,
        .b_nms = cfg->b_nms,
        .kt_nm_per_a = cfg->kt_nm_per_a,
        .pole_pairs = cfg->pole_pairs,
        .vdc_volts = q16_to_float(cfg->vdc_q16),
        .i_max_a = cfg->i_max_a,
        .locked_rotor = cfg->locked_rotor,
        .delta_connection = cfg->connection_delta,
    };

    if (p.r_ohm <= 0.0f) {
        p.r_ohm = (float)CONFIG_FOC_ITL_DEFAULT_R_MILLIOHM / 1000.0f;
    }
    if (p.l_henry <= 0.0f) {
        p.l_henry = (float)CONFIG_FOC_ITL_DEFAULT_L_UH / 1000000.0f;
    }
    if (p.j_kgm2 <= 0.0f) {
        p.j_kgm2 = (float)CONFIG_FOC_ITL_DEFAULT_J_X1E7 / 10000000.0f;
    }
    if (p.b_nms <= 0.0f) {
        p.b_nms = (float)CONFIG_FOC_ITL_DEFAULT_B_X1E7 / 10000000.0f;
    }
    if (p.kt_nm_per_a <= 0.0f) {
        p.kt_nm_per_a = (float)CONFIG_FOC_ITL_DEFAULT_KT_X1E7 / 10000000.0f;
    }
    if (p.pole_pairs <= 0) {
        p.pole_pairs = CONFIG_FOC_ITL_DEFAULT_POLE_PAIRS;
    }
    if (p.vdc_volts <= 0.0f) {
        p.vdc_volts = (float)CONFIG_ESP_FOC_DC_LINK_NOMINAL_MV / 1000.0f;
    }
    if (p.i_max_a <= 0.0f) {
        p.i_max_a = (float)CONFIG_FOC_ITL_DEFAULT_I_MAX_MA / 1000.0f;
    }
    if (!cfg->connection_delta) {
#ifdef CONFIG_FOC_ITL_MOTOR_DELTA
        p.delta_connection = (bool)CONFIG_FOC_ITL_MOTOR_DELTA;
#else
        p.delta_connection = false;
#endif
    }
    return p;
}

esp_foc_err_t esp_foc_in_the_loop_create(const esp_foc_in_the_loop_config_t *cfg,
                                     esp_foc_in_the_loop_handles_t *out)
{
    if (cfg == NULL || out == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if (s_created) {
        ESP_LOGE(TAG, "singleton already created");
        return ESP_FOC_ERR_INVALID_ARG;
    }

    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.pwm_hz = fitl_pwm_hz_from_config(cfg->pwm_hz);
    s_ctx.vdc_q16 = cfg->vdc_q16;
    if (s_ctx.vdc_q16 <= 0) {
        s_ctx.vdc_q16 = q16_from_float((float)CONFIG_ESP_FOC_DC_LINK_NOMINAL_MV / 1000.0f);
    }

    const esp_foc_itl_plant_params_t plant_params = params_from_config(cfg);
    esp_foc_itl_plant_init(&s_ctx.plant, &plant_params);
    esp_foc_itl_plant_set_dt(&s_ctx.plant, s_ctx.pwm_hz);
    wire_vtables(&s_ctx);

    const float fc = (float)CONFIG_ESP_FOC_CURRENT_FILTER_CUTOFF_HZ;
    const float fs = (float)s_ctx.pwm_hz;
    esp_foc_biquad_butterworth_lpf_design_q16(&s_ctx.bq_u, fc, fs);
    esp_foc_biquad_butterworth_lpf_design_q16(&s_ctx.bq_v, fc, fs);

    s_ctx.alive = true;
    void *plant_handle = NULL;
    if (esp_foc_create_runner(plant_task_fn, &s_ctx, FOC_ITL_PLANT_RUNNER_PRIORITY,
                              &plant_handle) != 0) {
        ESP_LOGE(TAG, "plant task create failed");
        memset(&s_ctx, 0, sizeof(s_ctx));
        return ESP_FOC_ERR_UNKNOWN;
    }
    s_ctx.plant_ev = (esp_foc_event_handle_t)plant_handle;

    esp_foc_err_t err = esp_foc_itl_tick_start(tick_cb, &s_ctx, s_ctx.pwm_hz);
    if (err != ESP_FOC_OK) {
        s_ctx.alive = false;
        esp_foc_runner_wake(s_ctx.plant_ev);
        plant_runner_join(s_ctx.plant_ev);
        memset(&s_ctx, 0, sizeof(s_ctx));
        return err;
    }

    out->inverter = &s_ctx.inverter;
    out->isensor = &s_ctx.isensor;
    out->rotor = &s_ctx.rotor;
    s_created = true;

    ESP_LOGI(TAG, "FITL ready @ %lu Hz, pp=%d, locked=%d",
             (unsigned long)s_ctx.pwm_hz, plant_params.pole_pairs,
             (int)plant_params.locked_rotor);
    return ESP_FOC_OK;
}

void esp_foc_in_the_loop_destroy(void)
{
    if (!s_created) {
        return;
    }

    esp_foc_itl_tick_stop();
    s_ctx.alive = false;
    if (s_ctx.plant_ev != NULL) {
        esp_foc_runner_wake(s_ctx.plant_ev);
        plant_runner_join(s_ctx.plant_ev);
    }

    memset(&s_ctx, 0, sizeof(s_ctx));
    s_created = false;
}
