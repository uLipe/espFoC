/*
 * MIT License
 *
 * espFoC tuner demo:
 *   - Walks through the build-time autotuned gains, then the runtime
 *     retune API and the tuner request/response protocol.
 *   - Uses inline stub drivers so it runs unchanged in QEMU (no hardware).
 *
 * Build/run in QEMU:
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
#include "espFoC/esp_foc_axis_tuning.h"
#include "espFoC/esp_foc_tuner.h"

static const char *TAG = "tuner-demo";

/* --- Inline stub drivers ----------------------------------------------- */

typedef struct {
    esp_foc_inverter_t base;
    float dc_link_pu;
    uint32_t pwm_hz;
} stub_inverter_t;

static q16_t stub_get_dc_link(esp_foc_inverter_t *self)
{
    return q16_from_float(((stub_inverter_t *)self)->dc_link_pu);
}
static uint32_t stub_get_pwm(esp_foc_inverter_t *self)
{
    return ((stub_inverter_t *)self)->pwm_hz;
}
static void stub_set_voltages(esp_foc_inverter_t *s, q16_t a, q16_t b, q16_t c)
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
        .set_voltages          = stub_set_voltages,
        .get_inverter_pwm_rate = stub_get_pwm,
        .enable                = stub_enable,
        .disable               = stub_disable,
    },
    .dc_link_pu = 1.0f,
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

/* --- Demo helpers ------------------------------------------------------ */

static void log_gains(const char *label, esp_foc_axis_t *axis)
{
    q16_t kp, ki, lim;
    esp_foc_axis_get_current_pi_gains_q16(axis, &kp, &ki, &lim);
    ESP_LOGI(TAG, "%-22s Kp=%9.4f V/A   Ki=%9.2f V/(A*s)   ILim=%6.2f V",
             label,
             (double)q16_to_float(kp),
             (double)q16_to_float(ki),
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
        ESP_FOC_TUNER_OP_READ, ESP_FOC_TUNER_PARAM_KP_Q16,
        NULL, 0, resp, &resp_len));
    ESP_LOGI(TAG, "READ Kp -> %.4f V/A",
             (double)q16_to_float(deserialize_q16_le(resp)));

    uint8_t payload[4];
    serialize_q16_le(payload, q16_from_float(2.345f));
    size_t no_resp = 0;
    ESP_ERROR_CHECK(esp_foc_tuner_handle_request(0,
        ESP_FOC_TUNER_OP_WRITE, ESP_FOC_TUNER_WRITE_KP_Q16,
        payload, sizeof(payload), NULL, &no_resp));
    ESP_LOGI(TAG, "WRITE Kp = 2.345 (manual override)");

    resp_len = sizeof(resp);
    ESP_ERROR_CHECK(esp_foc_tuner_handle_request(0,
        ESP_FOC_TUNER_OP_READ, ESP_FOC_TUNER_PARAM_KP_Q16,
        NULL, 0, resp, &resp_len));
    ESP_LOGI(TAG, "READ Kp -> %.4f V/A (after write)",
             (double)q16_to_float(deserialize_q16_le(resp)));

    /* EXEC RECOMPUTE_GAINS with iPower gimbal motor params */
    uint8_t exec_payload[12];
    serialize_q16_le(exec_payload + 0, q16_from_float(1.08f));
    serialize_q16_le(exec_payload + 4, q16_from_float(0.0018f));
    serialize_q16_le(exec_payload + 8, q16_from_float(150.0f));
    no_resp = 0;
    ESP_ERROR_CHECK(esp_foc_tuner_handle_request(0,
        ESP_FOC_TUNER_OP_EXEC, ESP_FOC_TUNER_CMD_RECOMPUTE_GAINS,
        exec_payload, sizeof(exec_payload), NULL, &no_resp));
    ESP_LOGI(TAG, "EXEC RECOMPUTE_GAINS R=1.08 L=1.8mH bw=150Hz");

    resp_len = sizeof(resp);
    ESP_ERROR_CHECK(esp_foc_tuner_handle_request(0,
        ESP_FOC_TUNER_OP_READ, ESP_FOC_TUNER_PARAM_KP_Q16,
        NULL, 0, resp, &resp_len));
    ESP_LOGI(TAG, "READ Kp -> %.4f V/A (after MPZ recompute)",
             (double)q16_to_float(deserialize_q16_le(resp)));

    resp_len = sizeof(resp);
    ESP_ERROR_CHECK(esp_foc_tuner_handle_request(0,
        ESP_FOC_TUNER_OP_READ, ESP_FOC_TUNER_PARAM_KI_Q16,
        NULL, 0, resp, &resp_len));
    ESP_LOGI(TAG, "READ Ki -> %.2f V/(A*s)",
             (double)q16_to_float(deserialize_q16_le(resp)));
}

static void try_retune(esp_foc_axis_t *axis, const char *label,
                       float r, float l, float bw)
{
    ESP_LOGI(TAG, "retune %s: R=%.3f L=%.4fmH bw=%.0fHz", label, (double)r,
             (double)(l * 1000.0f), (double)bw);
    esp_foc_err_t err = esp_foc_axis_retune_current_pi_q16(axis,
        q16_from_float(r), q16_from_float(l), q16_from_float(bw));
    if (err != ESP_FOC_OK) {
        ESP_LOGW(TAG, "retune REJECTED (err=%d) -- bw probably above Nyquist "
                       "for the current loop period", err);
        return;
    }
    log_gains(label, axis);
}

static void demo_task(void *arg)
{
    esp_foc_axis_t *axis = (esp_foc_axis_t *)arg;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== Initial gains (build-time autogen, default.json) ===");
    log_gains("autogen default", axis);

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== Runtime retune via MPZ helper ===");
    /* Loop period is PWM/decimation = 20kHz/20 = 1ms, so Nyquist = 500 Hz.
     * Each retune below picks a bandwidth that fits the discrete model. */
    try_retune(axis, "iPower (R=1.08,L=1.8mH)",  1.08f, 0.0018f, 150.0f);
    try_retune(axis, "low-R servo  (R=0.25,L=0.8mH)",  0.25f, 0.0008f, 200.0f);
    try_retune(axis, "high-L motor (R=0.80,L=5.0mH)",  0.80f, 0.0050f,  50.0f);

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Asking for a bandwidth above Nyquist (bw=600Hz, Ts=1ms):");
    try_retune(axis, "above-nyquist demo", 0.5f, 0.001f, 600.0f);

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

    ESP_LOGI(TAG, "espFoC tuner demo starting (PWM=%lu Hz, decimation=20)",
             (unsigned long)s_inv.pwm_hz);

    esp_foc_err_t err = esp_foc_initialize_axis(
        &axis, &s_inv.base, &s_rotor, NULL, settings);
    if (err != ESP_FOC_OK) {
        ESP_LOGE(TAG, "axis init failed: %d", err);
        return;
    }
    ESP_ERROR_CHECK(esp_foc_tuner_attach_axis(0, &axis));

    /* Drive the demo from a task because esp_foc_initialize_axis sleeps. */
    xTaskCreate(demo_task, "demo", 4096, &axis, 4, NULL);
}
