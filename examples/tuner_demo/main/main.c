/*
 * MIT License
 *
 * espFoC tuner demo: default bypass gains, tuner protocol round-trip.
 * Inline stub drivers — runs in QEMU without hardware.
 *
 *   cd examples/tuner_demo
 *   idf.py set-target esp32
 *   idf.py qemu
 */

#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "espFoC/esp_foc.h"
#include "espFoC/gui_link/esp_foc_tuner.h"

static const char *TAG = "tuner-demo";

typedef struct {
    esp_foc_inverter_t base;
    float dc_link_volts;
    uint32_t pwm_hz;
} stub_inverter_t;

static q16_t stub_get_dc_link(esp_foc_inverter_t *self)
{
    return q16_from_float(((stub_inverter_t *)self)->dc_link_volts);
}
static uint32_t stub_get_pwm(esp_foc_inverter_t *self)
{
    return ((stub_inverter_t *)self)->pwm_hz;
}
static void stub_set_duties(esp_foc_inverter_t *s, q16_t a, q16_t b, q16_t c)
{
    (void)s; (void)a; (void)b; (void)c;
}
static void stub_set_cb(esp_foc_inverter_t *s, esp_foc_inverter_callback_t c, void *a)
{
    (void)s; (void)c; (void)a;
}
static void stub_enable(esp_foc_inverter_t *s)  { (void)s; }
static void stub_disable(esp_foc_inverter_t *s) { (void)s; }

static stub_inverter_t s_inv = {
    .base = {
        .set_inverter_callback = stub_set_cb,
        .get_dc_link_voltage   = stub_get_dc_link,
        .set_duties            = stub_set_duties,
        .get_inverter_pwm_rate = stub_get_pwm,
        .enable                = stub_enable,
        .disable               = stub_disable,
    },
    .dc_link_volts = 48.0f,
    .pwm_hz = 20000,
};

static void stub_set_zero(esp_foc_rotor_sensor_t *s) { (void)s; }
static uint32_t stub_cpr(esp_foc_rotor_sensor_t *s) { (void)s; return 4096; }
static q16_t stub_read_counts(esp_foc_rotor_sensor_t *s) { (void)s; return 0; }
static int64_t stub_read_accum(esp_foc_rotor_sensor_t *s) { (void)s; return 0; }
static void stub_set_sim(esp_foc_rotor_sensor_t *s, q16_t inc) { (void)s; (void)inc; }

static esp_foc_rotor_sensor_t s_rotor = {
    .set_to_zero                  = stub_set_zero,
    .get_counts_per_revolution    = stub_cpr,
    .read_counts                  = stub_read_counts,
    .read_accumulated_counts_i64  = stub_read_accum,
    .set_simulation_count         = stub_set_sim,
};

static void log_gains(const char *label, esp_foc_axis_t *axis)
{
    q16_t kp, ki, kd, kff, lim;
    esp_foc_axis_get_current_loop_gains_q16(axis, &kp, &ki, &kd, &kff, &lim);
    ESP_LOGI(TAG, "%-22s Kp=%.4f Ki=%.4f Kd=%.4f Kff=%.4f ILim=%.3f",
             label,
             (double)q16_to_float(kp),
             (double)q16_to_float(ki),
             (double)q16_to_float(kd),
             (double)q16_to_float(kff),
             (double)q16_to_float(lim));
}

static void serialize_q16_le(uint8_t *dst, q16_t v)
{
    uint32_t u = (uint32_t)v;
    dst[0] = (uint8_t)u;
    dst[1] = (uint8_t)(u >> 8);
    dst[2] = (uint8_t)(u >> 16);
    dst[3] = (uint8_t)(u >> 24);
}

static q16_t deserialize_q16_le(const uint8_t *src)
{
    uint32_t u = (uint32_t)src[0] |
                 ((uint32_t)src[1] << 8) |
                 ((uint32_t)src[2] << 16) |
                 ((uint32_t)src[3] << 24);
    return (q16_t)u;
}

static void demo_protocol(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== Tuner protocol round-trip ===");

    uint8_t resp[4];
    size_t resp_len = sizeof(resp);
    ESP_ERROR_CHECK(esp_foc_tuner_handle_request(0,
        ESP_FOC_TUNER_OP_READ, ESP_FOC_TUNER_PARAM_KFF_Q16,
        NULL, 0, resp, &resp_len));
    ESP_LOGI(TAG, "READ Kff -> %.4f",
             (double)q16_to_float(deserialize_q16_le(resp)));

    uint8_t payload[4];
    serialize_q16_le(payload, q16_from_float(0.95f));
    size_t no_resp = 0;
    ESP_ERROR_CHECK(esp_foc_tuner_handle_request(0,
        ESP_FOC_TUNER_OP_WRITE, ESP_FOC_TUNER_WRITE_KFF_Q16,
        payload, sizeof(payload), NULL, &no_resp));

    resp_len = sizeof(resp);
    ESP_ERROR_CHECK(esp_foc_tuner_handle_request(0,
        ESP_FOC_TUNER_OP_READ, ESP_FOC_TUNER_PARAM_KFF_Q16,
        NULL, 0, resp, &resp_len));
    ESP_LOGI(TAG, "READ Kff -> %.4f (after write)",
             (double)q16_to_float(deserialize_q16_le(resp)));

    no_resp = 0;
    ESP_ERROR_CHECK(esp_foc_tuner_handle_request(0,
        ESP_FOC_TUNER_OP_EXEC, ESP_FOC_TUNER_CMD_PING,
        NULL, 0, NULL, &no_resp));
    ESP_LOGI(TAG, "EXEC PING OK");
}

static void demo_task(void *arg)
{
    esp_foc_axis_t *axis = (esp_foc_axis_t *)arg;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== Initial gains (bypass default) ===");
    log_gains("boot", axis);

    demo_protocol();

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== Demo complete ===");
    vTaskDelete(NULL);
}

void app_main(void)
{
    static esp_foc_axis_t axis;
    esp_foc_motor_control_settings_t settings = {
        .natural_direction = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW,
        .motor_pole_pairs = 7,
        .motor_unit       = 0,
    };

    memset(&axis, 0, sizeof(axis));

    ESP_LOGI(TAG, "espFoC tuner demo starting (PWM=%lu Hz)",
             (unsigned long)s_inv.pwm_hz);

    esp_foc_err_t err = esp_foc_initialize_axis(
        &axis, &s_inv.base, &s_rotor, NULL, settings);
    if (err != ESP_FOC_OK) {
        ESP_LOGE(TAG, "axis init failed: %d", err);
        return;
    }
    ESP_ERROR_CHECK(esp_foc_tuner_attach_axis(0, &axis));

    xTaskCreate(demo_task, "demo", 4096, &axis, 4, NULL);
}
