/*
 * SPDX-FileCopyrightText: 2026 Felipe Neves
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Sensored motor control driven from a desktop GUI (host/espfoc_gui.py).
 *
 * The board does nothing on its own. The host walks it through
 * commissioning, one step per button:
 *
 *   1. MAP    phase discovery: phase order, current signs, encoder zero and
 *             direction.
 *   2. IDENT  motor identification (R, L, flux, K, J, encoder offset and
 *             delay), then the sensored controller is built from it and
 *             started, holding position.
 *   3. COG    cogging learn: slow sweeps both ways, the cogging feedforward
 *             table is switched on, the current position becomes 0.
 *
 * After that the host picks torque, speed or position mode and streams a
 * reference. The board answers with telemetry at 10 Hz.
 *
 * Everything travels over the console UART as small binary frames (see the
 * README for the layout). ESP_LOG output is wrapped into LOG frames, so the
 * GUI log shows what the library reports while a step runs.
 *
 * STOP while the controller runs is a dry cut (bridge off, controller idle).
 * Phase discovery and identification cannot be cancelled half way, so STOP
 * during them reboots the chip; the bridge stays disabled across the reset.
 */

#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "espFoC/drivers/esp_foc_inverter_mcpwm.h"
#ifdef CONFIG_EXAMPLE_ROTOR_SENSOR_QUADRATURE
#include "espFoC/drivers/esp_foc_rotor_quadrature.h"
#else
#include "espFoC/drivers/esp_foc_rotor_as5600.h"
#endif
#include "espFoC/motor_control/esp_foc_motor_id.h"
#include "espFoC/motor_control/esp_foc_phase_discover.h"
#include "espFoC/motor_control/esp_foc_sensored.h"
#include "espFoC/osal/esp_foc_osal.h"
#include "espFoC/utils/esp_foc_clarke.h"
#include "espFoC/utils/esp_foc_park.h"
#include "espFoC/utils/esp_foc_svm.h"
#include "espFoC/utils/esp_foc_trig.h"
#include "commissioning_store.h"

#define MOTOR_AXIS 0u

#define PWM_RATE_HZ ((uint32_t)CONFIG_ESP_FOC_PWM_RATE_HZ)
#define ENCODER_RATE_HZ ((uint32_t)CONFIG_ESP_FOC_SD_FETCH_HZ)
#define POLE_PAIRS CONFIG_EXAMPLE_MOTOR_POLE_PAIRS
#define DC_LINK_VOLTS ((float)CONFIG_EXAMPLE_DC_LINK_MILLIVOLT * 1.0e-3f)

#if CONFIG_EXAMPLE_BRIDGE_ENABLE_ACTIVE_LOW
#define BRIDGE_ENABLE_ACTIVE_LOW true
#else
#define BRIDGE_ENABLE_ACTIVE_LOW false
#endif

#define GUI_MAX_RPM ((float)CONFIG_EXAMPLE_GUI_MAX_RPM)
#define GUI_MAX_IQ_A ((float)CONFIG_EXAMPLE_GUI_MAX_IQ_MA * 1.0e-3f)
#define GUI_MAX_DEG ((float)CONFIG_EXAMPLE_GUI_MAX_DEG)
#define GUI_ACCEL_RPM_S ((float)CONFIG_EXAMPLE_GUI_ACCEL_RPM_S)

#define TWO_PI (2.0f * (float)M_PI)
#define DEG_PER_RAD (180.0f / (float)M_PI)
#define RPM_PER_RADS_MECH (60.0f / TWO_PI)

/* Up to six passes of two-turn sweeps each way at 0.25 rev/s, plus rests. */
#define COGGING_LEARN_TIMEOUT_MS 150000u

#define CONSOLE_UART CONFIG_ESP_CONSOLE_UART_NUM
/* Frames are queued here, so an event callback on the controller's task
 * never waits on the wire. */
#define UART_TX_BUFFER 4096
#define UART_RX_BUFFER 512

#define TELEMETRY_PERIOD_MS 100u
#define WORKER_STACK 8192u
#define WORKER_PRIORITY 4
#define TELEMETRY_PRIORITY 3

/* ---------------------------------------------------------------------------
 * Wire protocol
 *
 *   0xA5 | type | len | payload[len] | crc8(type, len, payload)
 *
 * crc8 is polynomial 0x07, initial value 0. Multi-byte fields are little
 * endian, floats are IEEE-754 single precision.
 * ------------------------------------------------------------------------- */
#define FRAME_SYNC 0xA5u
#define FRAME_MAX_PAYLOAD 64u

/* Host -> board */
#define CMD_PING 0x01u  /* no payload; the board answers INFO */
#define CMD_MAP 0x02u   /* start phase discovery */
#define CMD_IDENT 0x03u /* start identification, then start the controller */
#define CMD_COG 0x04u   /* start cogging learn */
#define CMD_MODE 0x05u  /* u8 mode: 0 torque, 1 speed, 2 position */
#define CMD_REF 0x06u   /* f32 reference: A, rpm or deg by the current mode */
#define CMD_STOP 0x07u  /* no payload */

/* Board -> host */
#define MSG_INFO 0x81u /* u8 pole pairs, f32 max rpm, f32 max iq A, f32 max deg */
#define MSG_TEL 0x82u  /* see send_telemetry() */
#define MSG_PROG 0x83u /* u8 step, u8 percent (0xFF: not known) */
#define MSG_LOG 0x84u  /* text, not terminated */
#define MSG_ACK 0x85u  /* u8 command, i32 esp_err_t */

/* Commissioning steps; also the bit positions of the "done" mask. */
typedef enum {
    STEP_MAP = 0,
    STEP_IDENT,
    STEP_COG,
    STEP_COUNT,
} step_t;

/* What the worker task is doing; reported in every TEL frame. */
typedef enum {
    BUSY_NONE = 0,
    BUSY_MAP,
    BUSY_IDENT,
    BUSY_COG,
    BUSY_START,
} busy_t;

#define TEL_FLAG_RUNNING (1u << 0)
#define TEL_FLAG_ABORT (1u << 1)
#define TEL_FLAG_FAULT (1u << 2)
#define TEL_FLAG_INPOS (1u << 3)
#define TEL_FLAG_COGGING (1u << 4)

static const char *TAG = "gui";

static esp_foc_inverter_t *inverter;
static esp_foc_rotor_sensor_t *encoder;

/* What commissioning measured, one member per step: enough to rebuild the
 * controller without running a step again. The encoder zero and the
 * cogging table only hold on the encoder mounting the steps ran on. */
typedef struct {
    struct {
        esp_foc_phase_discover_result_t result;
        q16_t encoder_zero_rad;
    } map;
    esp_foc_motor_id_result_t plant;
    struct {
        esp_foc_sensored_cogging_info_t info;
        q16_t table[CONFIG_ESP_FOC_SD_COG_BINS];
    } cog;
} commissioning_t;

static commissioning_t record;
static esp_foc_sensored_config_t controller_config;

/* Written by the RX loop and the worker; read by telemetry. Byte-sized, so
 * a read never tears. */
static volatile uint8_t steps_done;
static volatile uint8_t busy;
static volatile bool controller_ready;
static volatile uint8_t active_mode = ESP_FOC_SD_MODE_POSITION;
/* Whole turns between the shaft and the origin when position mode was last
 * entered, taken off every angle the GUI sees or sends. */
static volatile int32_t turns_folded;
/* Quadrature count is 0 after every boot. Park and the cogging table from
 * NVS belong to the previous count, so the first start has to seat them. */
static bool frame_seated;

/* ------------------------------------------------------------------------- */

static unsigned pole_pairs(void)
{
    return (controller_config.pole_pairs != 0u) ? controller_config.pole_pairs
                                                : (unsigned)POLE_PAIRS;
}

static float rpm_to_hz(float rpm)
{
    return rpm / 60.0f * (float)pole_pairs();
}

static float rads_to_rpm(float we_rads)
{
    return we_rads * RPM_PER_RADS_MECH / (float)pole_pairs();
}

static uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* One uart_write_bytes call per frame: the driver serialises writers, so
 * frames from different tasks never interleave. */
static void send_frame(uint8_t type, const void *payload, size_t len)
{
    uint8_t frame[FRAME_MAX_PAYLOAD + 4];

    if (len > FRAME_MAX_PAYLOAD) {
        len = FRAME_MAX_PAYLOAD;
    }
    frame[0] = FRAME_SYNC;
    frame[1] = type;
    frame[2] = (uint8_t)len;
    if (len > 0u) {
        memcpy(&frame[3], payload, len);
    }
    frame[3 + len] = crc8(&frame[1], len + 2u);
    uart_write_bytes(CONSOLE_UART, frame, len + 4u);
}

static void put_f32(uint8_t *dst, float value)
{
    memcpy(dst, &value, sizeof(value));
}

static void send_ack(uint8_t command, esp_err_t err)
{
    uint8_t payload[5];
    int32_t code = (int32_t)err;

    payload[0] = command;
    memcpy(&payload[1], &code, sizeof(code));
    send_frame(MSG_ACK, payload, sizeof(payload));
}

static void send_progress(step_t step, uint8_t percent)
{
    const uint8_t payload[2] = {(uint8_t)step, percent};
    send_frame(MSG_PROG, payload, sizeof(payload));
}

static void send_info(void)
{
    uint8_t payload[13];

    payload[0] = (uint8_t)pole_pairs();
    put_f32(&payload[1], GUI_MAX_RPM);
    put_f32(&payload[5], GUI_MAX_IQ_A);
    put_f32(&payload[9], GUI_MAX_DEG);
    send_frame(MSG_INFO, payload, sizeof(payload));
}

static void send_text(const char *text)
{
    size_t len = strlen(text);

    while (len > 0u) {
        const size_t chunk = (len > FRAME_MAX_PAYLOAD) ? FRAME_MAX_PAYLOAD : len;
        send_frame(MSG_LOG, text, chunk);
        text += chunk;
        len -= chunk;
    }
}

static void log_line(const char *format, ...)
{
    char text[128];
    va_list args;

    va_start(args, format);
    vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    send_text(text);
}

/* Every ESP_LOGx after the hook is installed lands here instead of on the
 * wire as plain text. Trailing newlines are dropped: one frame, one line. */
static int log_to_frame(const char *format, va_list args)
{
    char text[160];
    int len = vsnprintf(text, sizeof(text), format, args);

    if (len <= 0) {
        return len;
    }
    size_t n = strnlen(text, sizeof(text));
    while ((n > 0u) && ((text[n - 1u] == '\n') || (text[n - 1u] == '\r'))) {
        text[--n] = '\0';
    }
    if (n > 0u) {
        send_text(text);
    }
    return len;
}

/* ---------------------------------------------------------------------------
 * Telemetry
 *
 * TEL payload, 35 bytes:
 *   u8  steps done mask (bit per step_t)
 *   u8  busy (busy_t)
 *   u8  controller state (esp_foc_sensored_state_t, 0 before the controller
 *       exists)
 *   u8  mode (esp_foc_sensored_mode_t)
 *   u8  flags (TEL_FLAG_*)
 *   f32 speed reference [rpm], f32 speed [rpm]
 *   f32 position reference [deg], f32 position [deg]
 *   f32 iq reference [A], f32 iq [A]
 *   f32 DC link [V]
 * ------------------------------------------------------------------------- */
static void send_telemetry(void)
{
    uint8_t payload[35];
    esp_foc_sensored_status_t status;
    uint8_t flags = 0;

    memset(&status, 0, sizeof(status));
    if (controller_ready) {
        esp_foc_sensored_get_status(MOTOR_AXIS, &status);
        flags |= (status.state == ESP_FOC_SD_STATE_RUNNING) ? TEL_FLAG_RUNNING : 0u;
        flags |= (status.abort != ESP_FOC_SD_ABORT_NONE) ? TEL_FLAG_ABORT : 0u;
        flags |= (status.fault != ESP_FOC_FAULT_NONE) ? TEL_FLAG_FAULT : 0u;
        flags |= status.inpos ? TEL_FLAG_INPOS : 0u;
        flags |= status.cogging_on ? TEL_FLAG_COGGING : 0u;
        /* A loop that is not running keeps its last reference; report the
         * measurement instead so the scope does not show a stale target. */
        if (status.mode != ESP_FOC_SD_MODE_POSITION) {
            status.theta_ref_rad = status.theta_m_rad;
        }
        if (status.mode == ESP_FOC_SD_MODE_TORQUE) {
            status.w_ref_rads = status.we_rads;
        }
        const float fold_rad = (float)turns_folded * 360.0f / DEG_PER_RAD;
        status.theta_m_rad -= fold_rad;
        status.theta_ref_rad -= fold_rad;
    } else {
        /* No controller yet: show the shaft angle the encoder last latched,
         * so the position scope already moves during commissioning. */
        status.theta_m_rad = q16_to_float(esp_foc_rotor_sensor_get_position(encoder));
        status.theta_ref_rad = status.theta_m_rad;
        status.vdc_v = DC_LINK_VOLTS;
    }

    payload[0] = steps_done;
    payload[1] = busy;
    payload[2] = controller_ready ? (uint8_t)status.state : 0u;
    payload[3] = active_mode;
    payload[4] = flags;
    put_f32(&payload[5], rads_to_rpm(status.w_ref_rads));
    put_f32(&payload[9], rads_to_rpm(status.we_rads));
    put_f32(&payload[13], status.theta_ref_rad * DEG_PER_RAD);
    put_f32(&payload[17], status.theta_m_rad * DEG_PER_RAD);
    put_f32(&payload[21], status.iq_ref_a);
    put_f32(&payload[25], status.iq_a);
    put_f32(&payload[29], status.vdc_v);
    payload[33] = 0;
    payload[34] = 0;
    send_frame(MSG_TEL, payload, sizeof(payload));
}

static void telemetry_task(void *arg)
{
    (void)arg;
    for (;;) {
        send_telemetry();
        esp_foc_sleep_ms(TELEMETRY_PERIOD_MS);
    }
}

/* ---------------------------------------------------------------------------
 * Board bring-up (same wiring as the other sensored examples)
 * ------------------------------------------------------------------------- */
static esp_err_t setup_inverter(void)
{
    inverter = esp_foc_inverter_mcpwm_acquire(0);
    if (inverter == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* The driver encodes an active-low enable as the negated GPIO number. */
    int enable_gpio = CONFIG_EXAMPLE_BRIDGE_ENABLE_GPIO;
    if ((enable_gpio >= 0) && BRIDGE_ENABLE_ACTIVE_LOW) {
        enable_gpio = -enable_gpio;
    }

    const esp_foc_inverter_mcpwm_config_t inverter_config = {
        .gpio_uh = CONFIG_EXAMPLE_PWM_U_HIGH_GPIO,
        .gpio_ul = CONFIG_EXAMPLE_PWM_U_LOW_GPIO,
        .gpio_vh = CONFIG_EXAMPLE_PWM_V_HIGH_GPIO,
        .gpio_vl = CONFIG_EXAMPLE_PWM_V_LOW_GPIO,
        .gpio_wh = CONFIG_EXAMPLE_PWM_W_HIGH_GPIO,
        .gpio_wl = CONFIG_EXAMPLE_PWM_W_LOW_GPIO,
        .gpio_enable = enable_gpio,
        .pwm_hz = PWM_RATE_HZ,
        .deadtime_ns = CONFIG_EXAMPLE_PWM_DEAD_TIME_NS,
        .dc_link_volts = DC_LINK_VOLTS,
        .shunt_ohm = (float)CONFIG_EXAMPLE_SHUNT_MILLIOHM * 1.0e-3f,
        .amp_gain = (float)CONFIG_EXAMPLE_CURRENT_AMP_GAIN,
        .shunt_count = (CONFIG_EXAMPLE_CURRENT_W_GPIO < 0) ? 2u : 3u,
        .sense_topology = ESP_FOC_SENSE_INLINE,
        .gpio_iu = CONFIG_EXAMPLE_CURRENT_U_GPIO,
        .gpio_iv = CONFIG_EXAMPLE_CURRENT_V_GPIO,
        .gpio_iw = CONFIG_EXAMPLE_CURRENT_W_GPIO,
        .i_limit_amps = (float)CONFIG_EXAMPLE_BRIDGE_TRIP_MA * 1.0e-3f,
        .gpio_fault = -1,
    };
    return esp_foc_inverter_mcpwm_init(inverter, &inverter_config);
}

static void encoder_set_invert(bool invert)
{
#ifdef CONFIG_EXAMPLE_ROTOR_SENSOR_QUADRATURE
    esp_foc_rotor_quadrature_set_invert(encoder, invert);
#else
    esp_foc_rotor_as5600_set_invert(encoder, invert);
#endif
}

static esp_err_t setup_encoder(void)
{
#ifdef CONFIG_EXAMPLE_ROTOR_SENSOR_QUADRATURE
    encoder = esp_foc_rotor_quadrature_acquire(0);
    if (encoder == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const esp_foc_rotor_quadrature_config_t encoder_config = {
        .gpio_a = CONFIG_EXAMPLE_QUADRATURE_GPIO_A,
        .gpio_b = CONFIG_EXAMPLE_QUADRATURE_GPIO_B,
        .pcnt_group = 0,
        .pcnt_unit = 0,
        .ppr = CONFIG_EXAMPLE_QUADRATURE_PPR,
        .glitch_ns = 100u,
        .dt_seconds = 1.0f / (float)ENCODER_RATE_HZ,
        /* Phase discovery measures the counting direction; start unflipped. */
        .invert = false,
        .pole_pairs = POLE_PAIRS,
        .irq_level = 1,
    };
    return esp_foc_rotor_quadrature_init(encoder, &encoder_config);
#else
    encoder = esp_foc_rotor_as5600_acquire(0);
    if (encoder == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const esp_foc_rotor_as5600_config_t encoder_config = {
        .sda = CONFIG_EXAMPLE_ENCODER_SDA_GPIO,
        .scl = CONFIG_EXAMPLE_ENCODER_SCL_GPIO,
        .i2c_port = 0,
        .i2c_hz = CONFIG_EXAMPLE_ENCODER_I2C_HZ,
        .dt_seconds = 1.0f / (float)ENCODER_RATE_HZ,
        /* Phase discovery measures the counting direction; start unflipped. */
        .invert = false,
        .pole_pairs = POLE_PAIRS,
        .pwm_hz = PWM_RATE_HZ,
    };
    return esp_foc_rotor_as5600_init(encoder, &encoder_config);
#endif
}

/* ---------------------------------------------------------------------------
 * Commissioning results in NVS (CONFIG_EXAMPLE_GUI_PERSIST)
 *
 * One blob per step. Running a step discards it and every step after it,
 * since each is measured on top of the ones before. Flash writes stall the
 * encoder fetch task, so every save and erase happens with the bridge off.
 * ------------------------------------------------------------------------- */
/* Steps running on restored results; reported again on PING. */
static volatile uint8_t steps_restored;

#if CONFIG_EXAMPLE_GUI_PERSIST
static const char *const step_keys[STEP_COUNT] = {"map", "ident", "cog"};
static const char *const step_names[STEP_COUNT] = {"phase map", "identification",
                                                   "cogging table"};

static void *step_data(step_t step, size_t *size)
{
    switch (step) {
    case STEP_MAP:
        *size = sizeof(record.map);
        return &record.map;
    case STEP_IDENT:
        *size = sizeof(record.plant);
        return &record.plant;
    default:
        *size = sizeof(record.cog);
        return &record.cog;
    }
}

/* Everything a stored result depends on besides the motor itself. */
static uint32_t board_hash(void)
{
    const int32_t board[] = {
        CONFIG_EXAMPLE_DC_LINK_MILLIVOLT, CONFIG_ESP_FOC_PWM_RATE_HZ,
        CONFIG_EXAMPLE_PWM_DEAD_TIME_NS, CONFIG_EXAMPLE_SHUNT_MILLIOHM,
        CONFIG_EXAMPLE_CURRENT_AMP_GAIN, CONFIG_EXAMPLE_PWM_U_HIGH_GPIO,
        CONFIG_EXAMPLE_PWM_U_LOW_GPIO, CONFIG_EXAMPLE_PWM_V_HIGH_GPIO,
        CONFIG_EXAMPLE_PWM_V_LOW_GPIO, CONFIG_EXAMPLE_PWM_W_HIGH_GPIO,
        CONFIG_EXAMPLE_PWM_W_LOW_GPIO, CONFIG_EXAMPLE_CURRENT_U_GPIO,
        CONFIG_EXAMPLE_CURRENT_V_GPIO, CONFIG_EXAMPLE_CURRENT_W_GPIO,
#ifdef CONFIG_EXAMPLE_ROTOR_SENSOR_QUADRATURE
        CONFIG_EXAMPLE_QUADRATURE_GPIO_A, CONFIG_EXAMPLE_QUADRATURE_GPIO_B, CONFIG_EXAMPLE_QUADRATURE_PPR,
#else
        CONFIG_EXAMPLE_ENCODER_SDA_GPIO, CONFIG_EXAMPLE_ENCODER_SCL_GPIO,
        CONFIG_EXAMPLE_ENCODER_I2C_HZ,
#endif
        ENCODER_RATE_HZ, CONFIG_ESP_FOC_SD_COG_BINS,
    };
    return commissioning_store_crc(0, board, sizeof(board));
}

static void save_step(step_t step)
{
    size_t size;
    const void *data = step_data(step, &size);
    const esp_err_t err = commissioning_store_save(step_keys[step], data, size);
    if (err == ESP_OK) {
        log_line("nvs: %s saved", step_names[step]);
    } else {
        log_line("nvs: %s not saved (%s)", step_names[step], esp_err_to_name(err));
    }
}

static void erase_steps_from(step_t first)
{
    for (unsigned s = first; s < STEP_COUNT; s++) {
        const esp_err_t err = commissioning_store_erase(step_keys[s]);
        if (err != ESP_OK) {
            log_line("nvs: %s not erased (%s)", step_names[s], esp_err_to_name(err));
        }
    }
}
#else
static void save_step(step_t step)
{
    (void)step;
}

static void erase_steps_from(step_t first)
{
    (void)first;
}
#endif

static void forget_steps_from(step_t first)
{
    const uint8_t keep = (uint8_t)((1u << first) - 1u);
    steps_done &= keep;
    steps_restored &= keep;
}

static void report_restored(void)
{
    const uint8_t mask = steps_restored;
    if (mask == 0u) {
        return;
    }
    log_line("nvs: running on saved commissioning:%s%s%s; any step can run again",
             (mask & (1u << STEP_MAP)) ? " map" : "", (mask & (1u << STEP_IDENT)) ? " ident" : "",
             (mask & (1u << STEP_COG)) ? " cog" : "");
}

/* ---------------------------------------------------------------------------
 * Commissioning step 1: phase discovery
 *
 * Progress: the 12 candidate maps of stage A cover 0..60 %, the accepted
 * map 70 %, then the encoder stages (sweep, rest, zero, direction).
 * ------------------------------------------------------------------------- */
static void on_discover_event(void *context, const esp_foc_phase_discover_event_t *event)
{
    (void)context;
    switch (event->ev) {
    case ESP_FOC_PHASE_DISCOVER_EV_ATTEMPT:
        log_line("map: attempt %d", event->idx);
        send_progress(STEP_MAP, 0);
        break;
    case ESP_FOC_PHASE_DISCOVER_EV_CANDIDATE:
        send_progress(STEP_MAP, (uint8_t)((event->idx + 1) * 60 / 12));
        break;
    case ESP_FOC_PHASE_DISCOVER_EV_REFUSED:
        log_line("map: attempt refused (%s)", esp_err_to_name(event->err));
        break;
    case ESP_FOC_PHASE_DISCOVER_EV_MAP:
        log_line("map: electrical phase order accepted");
        send_progress(STEP_MAP, 70);
        break;
    case ESP_FOC_PHASE_DISCOVER_EV_WELL:
        log_line("sensor: well travel %+.4f rad, %s", (double)q16_to_float(event->motion),
                 event->good ? "accepted" : "refused");
        send_progress(STEP_MAP, 80);
        break;
    case ESP_FOC_PHASE_DISCOVER_EV_STILL:
        send_progress(STEP_MAP, 85);
        break;
    case ESP_FOC_PHASE_DISCOVER_EV_ZERO:
        send_progress(STEP_MAP, 90);
        break;
    case ESP_FOC_PHASE_DISCOVER_EV_DIR:
        send_progress(STEP_MAP, 100);
        break;
    default:
        break;
    }
}

/* Puts the inverter and the encoder where phase discovery left them. */
static esp_err_t apply_phase_map(void)
{
    static const char letters[] = "UVW";
    const esp_foc_phase_discover_result_t *map = &record.map.result;

    esp_err_t err = inverter->set_phase_map(inverter, &map->map);
    if (err == ESP_OK) {
        err = esp_foc_rotor_sensor_set_offset(encoder, record.map.encoder_zero_rad);
    }
    if (err != ESP_OK) {
        log_line("map: not applied (%s)", esp_err_to_name(err));
        return err;
    }
    /* From here on the encoder counts positive in the motor's forward direction. */
    encoder_set_invert(map->sensor_reversed);
    log_line("map: U->%c(%+d) V->%c(%+d) W->%c(%+d), encoder %s",
             letters[map->map.pwm_to_hw[0] % 3u], map->map.i_sign[0],
             letters[map->map.pwm_to_hw[1] % 3u], map->map.i_sign[1],
             letters[map->map.pwm_to_hw[2] % 3u], map->map.i_sign[2],
             map->sensor_reversed ? "reversed (corrected)" : "as mounted");
    log_line("map: encoder zero %.4f rad", (double)q16_to_float(record.map.encoder_zero_rad));
    return ESP_OK;
}

static esp_err_t discover_phases(void)
{
    esp_foc_phase_discover_t discovery;
    esp_foc_phase_discover_config_t discovery_config;

    log_line("map: finding the phase order and zeroing the encoder");
    /* init() and cleanup() read the object's flags before setting them. */
    memset(&discovery, 0, sizeof(discovery));
    esp_foc_phase_discover_default_config(&discovery_config);
    discovery_config.on_event = on_discover_event;
    memset(&record.map, 0, sizeof(record.map));
    /* Discovery measures the counting direction from the raw sensor. */
    encoder_set_invert(false);
    esp_err_t err = esp_foc_phase_discover_init(&discovery, inverter, encoder, &discovery_config);
    if (err == ESP_OK) {
        err = esp_foc_phase_discover_run(&discovery, &record.map.result);
    }
    esp_foc_phase_discover_cleanup(&discovery);
    if (err == ESP_OK) {
        err = esp_foc_rotor_sensor_get_offset(encoder, &record.map.encoder_zero_rad);
    }
    if (err != ESP_OK) {
        log_line("commissioning: failed at %s (%s), electrical map %s",
                 esp_foc_phase_discover_stage_name(record.map.result.failed_at),
                 esp_err_to_name(err), record.map.result.map_valid ? "preserved" : "not found");
        return err;
    }
    return apply_phase_map();
}

/* ---------------------------------------------------------------------------
 * Commissioning step 2: identification, then the controller
 *
 * Progress: one slice per identification phase (BIAS .. PARK_ANGLE). Phases
 * do not run in enum order and BIAS repeats before every leg, so the bar
 * only moves forward.
 * ------------------------------------------------------------------------- */
static uint8_t ident_percent;

static void on_motor_id_event(void *context, const esp_foc_motor_id_event_t *event)
{
    (void)context;
    const unsigned first = ESP_FOC_MOTOR_ID_BIAS;
    const unsigned span = ESP_FOC_MOTOR_ID_DONE - ESP_FOC_MOTOR_ID_BIAS;

    switch (event->ev) {
    case ESP_FOC_MOTOR_ID_EV_PHASE:
        if ((event->phase >= ESP_FOC_MOTOR_ID_BIAS) && (event->phase <= ESP_FOC_MOTOR_ID_DONE)) {
            const uint8_t pct = (uint8_t)(((unsigned)event->phase - first) * 100u / span);
            if (pct > ident_percent) {
                ident_percent = pct;
                send_progress(STEP_IDENT, pct);
            }
        }
        log_line("ident: %s", esp_foc_motor_id_phase_name(event->phase));
        break;
    case ESP_FOC_MOTOR_ID_EV_DIRECTION:
        log_line("ident: direction pp=%u fm=%.3f/end=%.3f Hz iq=%.3f A offset=%+.3f rad (%s)",
                 (unsigned)event->pass, (double)event->v[0], (double)event->v[3],
                 (double)event->v[1], (double)event->v[2], esp_err_to_name(event->err));
        break;
    case ESP_FOC_MOTOR_ID_EV_MECH:
        log_line("ident: K %.0f rad/s2/A, J %.2e kg m2", (double)event->v[0],
                 (double)event->v[3]);
        break;
    default:
        break;
    }
}

static void on_controller_event(void *context, const esp_foc_sensored_event_t *event)
{
    (void)context;
    switch (event->ev) {
    case ESP_FOC_SD_EV_LEARN_PASS:
        log_line("cog: pass %u, change %.1f mA rms", (unsigned)event->pass,
                 (double)(event->change_rms_a * 1.0e3f));
        send_progress(STEP_COG,
                      (uint8_t)((event->pass * 100u) / controller_config.cogging.passes_max));
        break;
    case ESP_FOC_SD_EV_ABORT: {
        esp_foc_sensored_status_t status;
        const char *why = "none";
        memset(&status, 0, sizeof(status));
        esp_foc_sensored_get_status(MOTOR_AXIS, &status);
        if (event->abort == ESP_FOC_SD_ABORT_OVERSPEED) {
            why = "overspeed";
        } else if (event->abort == ESP_FOC_SD_ABORT_SENSOR_STALE) {
            why = "sensor stale";
        } else if (event->abort == ESP_FOC_SD_ABORT_SENSOR_FAIL) {
            why = "sensor fail";
        }
        log_line("controller: abort %s, we %.0f Hz, err %.0f deg", why,
                 (double)(status.we_rads / TWO_PI),
                 (double)((status.theta_ref_rad - status.theta_m_rad) * DEG_PER_RAD));
        break;
    }
    case ESP_FOC_SD_EV_FAULT:
        log_line("controller: fault %d, bridge off", (int)event->fault);
        break;
    case ESP_FOC_SD_EV_RUN_FAIL:
        log_line("controller: start failed (%d)", (int)event->fail);
        break;
    default:
        break;
    }
}

static void build_controller_config(void)
{
    esp_foc_sensored_default_config(&controller_config);
    controller_config.axis = MOTOR_AXIS;
    /* POSITION is the top level, so all three modes are reachable. */
    controller_config.control = ESP_FOC_SD_CONTROL_POSITION;
    controller_config.pole_pairs = POLE_PAIRS;
    controller_config.fetch_hz = ENCODER_RATE_HZ;
    esp_foc_sensored_config_from_motor_id(&controller_config, &record.plant);
    esp_foc_sensored_config_from_phase_map(&controller_config, &record.map.result);
    controller_config.guard.overspeed_hz =
        rpm_to_hz((float)CONFIG_EXAMPLE_OVERSPEED_RPM);
    controller_config.on_event = on_controller_event;
}

/*
 * Speed and torque can leave the shaft many turns from the origin, past the
 * GUI's position limit, and the slider would then have no room. Entering
 * position mode takes the whole turns off, so the angle comes back within
 * half a turn of the origin, the same physical point the cogging learn set.
 */
static esp_err_t apply_plant(void);
static esp_err_t apply_cogging(void);

static void fold_turns(void)
{
    esp_foc_sensored_status_t status;
    memset(&status, 0, sizeof(status));
    esp_foc_sensored_get_status(MOTOR_AXIS, &status);
    turns_folded = (int32_t)lroundf(status.theta_m_rad * DEG_PER_RAD / 360.0f);
}

static float wrap_pi(float angle)
{
    while (angle > (float)M_PI) {
        angle -= TWO_PI;
    }
    while (angle < -(float)M_PI) {
        angle += TWO_PI;
    }
    return angle;
}

static void rotate_cogging(int shift)
{
    static q16_t rotated[CONFIG_ESP_FOC_SD_COG_BINS];
    const int n = CONFIG_ESP_FOC_SD_COG_BINS;

    shift %= n;
    if (shift < 0) {
        shift += n;
    }
    if (shift == 0) {
        return;
    }
    for (int i = 0; i < n; i++) {
        const int src = i - shift;
        rotated[i] = record.cog.table[(src < 0) ? (src + n) : src];
    }
    memcpy(record.cog.table, rotated, sizeof(record.cog.table));
}

/*
 * PCNT starts at 0 on this boot, wherever the shaft happens to be. The saved
 * park offset and cogging bins were learned on the previous count, so closing
 * the loop on them is positive feedback: one turn, then the overspeed trip.
 * A short d-axis hold measures the new electrical angle and slides the table
 * by the same mechanical delta. An absolute encoder keeps its zero.
 */
static esp_err_t seat_incremental_frame(bool *did)
{
    *did = false;
#ifdef CONFIG_EXAMPLE_ROTOR_SENSOR_QUADRATURE
    if (frame_seated) {
        return ESP_OK;
    }
    const unsigned pp = controller_config.pole_pairs;
    if ((pp == 0u) || !(controller_config.rs_ohm > 0.0f)) {
        return ESP_ERR_INVALID_STATE;
    }

    float volts = 0.7f * controller_config.rs_ohm + record.plant.v_deadzone_v;
    if (volts < 0.4f) {
        volts = 0.4f;
    }
    if (volts > 1.5f) {
        volts = 1.5f;
    }
    q16_t s;
    q16_t c;
    q16_t alpha;
    q16_t beta;
    q16_t du;
    q16_t dv;
    q16_t dw;
    esp_foc_sincos(0, &s, &c);
    esp_foc_inv_park(s, c, q16_from_float(volts / DC_LINK_VOLTS), 0, &alpha, &beta);
    esp_foc_svm(alpha, beta, &du, &dv, &dw);

    if (inverter->is_faulted(inverter)) {
        (void)inverter->clear_fault(inverter);
    }
    /* The sensored DMA hook clears the flag calibrate waits on. */
    inverter->set_dma_callback(inverter, NULL, NULL);
    esp_err_t err = inverter->enable(inverter);
    if (err != ESP_OK) {
        log_line("seat: bridge enable failed (%s)", esp_err_to_name(err));
        return err;
    }
    inverter->set_sense_watchdog(inverter, false);
    inverter->calibrate_currents(inverter, 32);
    inverter->set_duties(inverter, du, dv, dw);

    float i_hold = 0.0f;
    for (int n = 0; n < 12; n++) {
        q16_t iu;
        q16_t iv;
        q16_t iw;
        q16_t ia;
        q16_t ib;
        esp_foc_sleep_ms(50);
        inverter->fetch_currents(inverter, &iu, &iv, &iw);
        esp_foc_clarke(iu, iv, iw, &ia, &ib);
        i_hold = hypotf(q16_to_float(ia), q16_to_float(ib));
        if (i_hold > 2.5f) {
            inverter->disable(inverter);
            log_line("seat: current %.2f A, bridge off", (double)i_hold);
            return ESP_ERR_INVALID_RESPONSE;
        }
    }
    if (inverter->is_faulted(inverter)) {
        inverter->disable(inverter);
        (void)inverter->clear_fault(inverter);
        log_line("seat: inverter fault, bridge off");
        return ESP_ERR_INVALID_STATE;
    }

    esp_foc_rotor_sensor_fetch(encoder);
    esp_foc_rotor_state_t st;
    esp_foc_rotor_sensor_snapshot(encoder, &st);
    inverter->disable(inverter);

    const float w_m = fabsf(q16_to_float(st.omega_m));
    if ((i_hold < 0.15f) || (w_m > 3.0f)) {
        log_line("seat: did not settle, I %.2f A, wm %.2f rad/s", (double)i_hold,
                 (double)w_m);
        return ESP_ERR_INVALID_RESPONSE;
    }

    const float th_e = q16_to_float(st.theta_e);
    const float old_off = controller_config.park_offset_rad;
    const float new_off = wrap_pi(-th_e);
    const bool have_cog = (steps_done & (1u << STEP_COG)) != 0u;
    int shift = 0;
    if (have_cog) {
        const float dmech = wrap_pi(th_e + old_off) / (float)pp;
        shift = (int)lroundf(dmech / TWO_PI * (float)CONFIG_ESP_FOC_SD_COG_BINS);
        rotate_cogging(shift);
    }
    record.plant.park_offset_rad = new_off;
    log_line("seat: park %+.3f -> %+.3f rad, cog %+d, I %.2f A", (double)old_off,
             (double)new_off, shift, (double)i_hold);

    const uint8_t mode = active_mode;
    esp_foc_sensored_deinit(MOTOR_AXIS);
    controller_ready = false;
    err = apply_plant();
    active_mode = mode;
    if (err != ESP_OK) {
        record.plant.park_offset_rad = old_off;
        if (have_cog) {
            rotate_cogging(-shift);
        }
        return err;
    }
    if (have_cog) {
        err = apply_cogging();
        if (err != ESP_OK) {
            record.plant.park_offset_rad = old_off;
            rotate_cogging(-shift);
            esp_foc_sensored_deinit(MOTOR_AXIS);
            controller_ready = false;
            (void)apply_plant();
            active_mode = mode;
            return err;
        }
    }
    frame_seated = true;
    *did = true;
#else
    frame_seated = true;
#endif
    return ESP_OK;
}

static esp_err_t start_controller(void)
{
    bool seated = false;
    esp_err_t err = seat_incremental_frame(&seated);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_foc_sensored_run(MOTOR_AXIS);
    if (err == ESP_OK) {
        esp_foc_sensored_set_speed_slew(MOTOR_AXIS, rpm_to_hz(GUI_ACCEL_RPM_S));
        err = esp_foc_sensored_set_mode(MOTOR_AXIS, (esp_foc_sensored_mode_t)active_mode);
    }
    if (err != ESP_OK) {
        return err;
    }
    if (seated) {
        turns_folded = 0;
        return esp_foc_sensored_set_origin(MOTOR_AXIS);
    }
    if (active_mode == ESP_FOC_SD_MODE_POSITION) {
        fold_turns();
    }
    return ESP_OK;
}

/* Builds the controller from the identified plant; the bridge stays off. */
static esp_err_t apply_plant(void)
{
    build_controller_config();
#ifdef CONFIG_EXAMPLE_ROTOR_SENSOR_QUADRATURE
    esp_foc_rotor_quadrature_set_pole_pairs(encoder, controller_config.pole_pairs);
#else
    esp_foc_rotor_as5600_set_pole_pairs(encoder, controller_config.pole_pairs);
#endif
    log_line("ident: Rs %.3f ohm, Ls %.0f uH, psi %.0f uWb, K %.0f rad/s2/A",
             (double)controller_config.rs_ohm, (double)(controller_config.ls_h * 1.0e6f),
             (double)(controller_config.psi_wb * 1.0e6f), (double)controller_config.k_rads2_a);
    esp_err_t err = esp_foc_sensored_init(inverter, encoder, &controller_config);
    if (err != ESP_OK) {
        log_line("controller: init failed (%s)", esp_err_to_name(err));
        return err;
    }
    controller_ready = true;
    active_mode = ESP_FOC_SD_MODE_POSITION;
    turns_folded = 0;
    return ESP_OK;
}

/* Bridge off, controller gone: identification and discovery own the
 * inverter next. */
static void teardown_controller(void)
{
    if (!controller_ready) {
        return;
    }
    controller_ready = false;
    frame_seated = false;
    esp_foc_sensored_stop(MOTOR_AXIS);
    esp_foc_sensored_deinit(MOTOR_AXIS);
}

static esp_err_t identify_motor(void)
{
    esp_foc_motor_id_config_t id_config;

    log_line("ident: about 90 s, the shaft will spin");
    ident_percent = 0;
    send_progress(STEP_IDENT, 0);
    esp_foc_motor_id_default_config(&id_config);
    id_config.pole_pairs = POLE_PAIRS;
    id_config.vdc = DC_LINK_VOLTS;
    id_config.on_event = on_motor_id_event;
    esp_err_t err = esp_foc_motor_id_run(inverter, encoder, &id_config, &record.plant);
    if (err != ESP_OK) {
        log_line("ident: failed at '%s' (%s)",
                 esp_foc_motor_id_phase_name(record.plant.failed_at), esp_err_to_name(err));
        return err;
    }
    /* The speed loop under position and speed modes is designed from K. */
    if ((record.plant.valid_mask & ESP_FOC_MOTOR_ID_VALID_K) == 0u) {
        log_line("ident: acceleration per amp not measured");
        return ESP_ERR_INVALID_RESPONSE;
    }
    send_progress(STEP_IDENT, 100);

    err = apply_plant();
    if (err != ESP_OK) {
        return err;
    }
    /* This boot's count is the one identification just fitted. */
    frame_seated = true;
    save_step(STEP_IDENT);
    err = start_controller();
    if (err != ESP_OK) {
        teardown_controller();
        log_line("controller: did not start (%s)", esp_err_to_name(err));
        return err;
    }
    log_line("controller: running, holding position");
    return ESP_OK;
}

/* ---------------------------------------------------------------------------
 * Commissioning step 3: cogging learn
 * ------------------------------------------------------------------------- */
static esp_err_t apply_cogging(void)
{
    const esp_foc_sensored_cogging_info_t *cogging = &record.cog.info;
    esp_err_t err = esp_foc_sensored_cogging_load(MOTOR_AXIS, record.cog.table,
                                                  CONFIG_ESP_FOC_SD_COG_BINS, cogging);
    if (err != ESP_OK) {
        log_line("cog: table not loaded (%s)", esp_err_to_name(err));
        return err;
    }
    log_line("cog: %u passes, %s, %.1f mA p-p, %u/%u bins", (unsigned)cogging->passes,
             cogging->converged ? "converged" : "not converged",
             (double)(cogging->p2p_a * 1.0e3f), (unsigned)cogging->bins_mapped,
             (unsigned)cogging->bins);
    return ESP_OK;
}

/* A failed learn leaves no table behind, neither in the loop nor in NVS. */
static void drop_cogging(void)
{
    esp_foc_sensored_cogging_enable(MOTOR_AXIS, false);
#if CONFIG_EXAMPLE_GUI_PERSIST
    esp_foc_sensored_stop(MOTOR_AXIS);
    erase_steps_from(STEP_COG);
#endif
}

static esp_err_t learn_cogging(void)
{
    esp_foc_sensored_cogging_info_t cogging;
    esp_err_t err = ESP_OK;

    /* The sweep needs a running controller in position mode; a STOP during
     * an earlier attempt left it idle. */
    active_mode = ESP_FOC_SD_MODE_POSITION;
    if (esp_foc_sensored_get_state(MOTOR_AXIS) == ESP_FOC_SD_STATE_IDLE) {
        err = start_controller();
    } else {
        err = esp_foc_sensored_set_mode(MOTOR_AXIS, ESP_FOC_SD_MODE_POSITION);
    }
    if (err != ESP_OK) {
        log_line("cog: controller not running (%s)", esp_err_to_name(err));
        drop_cogging();
        return err;
    }

    log_line("cog: slow sweeps both ways, up to 2 min");
    send_progress(STEP_COG, 0);
    memset(&cogging, 0, sizeof(cogging));
    err = esp_foc_sensored_cogging_learn(MOTOR_AXIS, COGGING_LEARN_TIMEOUT_MS, &cogging);
    if (err == ESP_OK) {
        err = esp_foc_sensored_cogging_get_table(MOTOR_AXIS, record.cog.table,
                                                 CONFIG_ESP_FOC_SD_COG_BINS);
    }
    if (err != ESP_OK) {
        log_line("cog: failed (%s)", esp_err_to_name(err));
        drop_cogging();
        return err;
    }
    record.cog.info = cogging;
    /* Reinstalled through the path a restore takes, so both run the same
     * table load. */
    err = apply_cogging();
    if (err != ESP_OK) {
        drop_cogging();
        return err;
    }
    send_progress(STEP_COG, 100);
#if CONFIG_EXAMPLE_GUI_PERSIST
    /* Off for the flash write, then holding again where the sweep ended. */
    esp_foc_sensored_stop(MOTOR_AXIS);
    save_step(STEP_COG);
    err = start_controller();
    if (err != ESP_OK) {
        log_line("controller: did not restart (%s)", esp_err_to_name(err));
        return err;
    }
#endif
    /* Position references from the GUI are measured from here. */
    turns_folded = 0;
    return esp_foc_sensored_set_origin(MOTOR_AXIS);
}

/* ---------------------------------------------------------------------------
 * Restore on boot: each saved step is applied in order, through the same
 * apply_* calls the steps end with. The first step missing or refused ends
 * the restore and discards the steps after it, which were measured on top
 * of it. The bridge stays off; picking a mode starts the controller.
 * ------------------------------------------------------------------------- */
#if CONFIG_EXAMPLE_GUI_PERSIST
static void restore_commissioning(void)
{
    esp_err_t err = commissioning_store_init(board_hash());
    if (err != ESP_OK) {
        log_line("nvs: not available (%s), commissioning will not be kept",
                 esp_err_to_name(err));
        return;
    }

    for (unsigned s = STEP_MAP; s < STEP_COUNT; s++) {
        size_t size;
        void *data = step_data((step_t)s, &size);
        err = commissioning_store_load(step_keys[s], data, size);
        if ((s == STEP_MAP) && (err == ESP_OK)) {
            log_line("nvs: saved commissioning found, restoring");
        }
        if (err == ESP_OK) {
            err = (s == STEP_MAP) ? apply_phase_map() : (s == STEP_IDENT) ? apply_plant()
                                                                         : apply_cogging();
        }
        if (err != ESP_OK) {
            if (err != ESP_ERR_NOT_FOUND) {
                log_line("nvs: %s discarded (%s)", step_names[s], commissioning_store_reason(err));
            }
            memset(data, 0, size);
            erase_steps_from((step_t)s);
            break;
        }
        log_line("nvs: %s restored", step_names[s]);
        steps_done |= 1u << s;
        steps_restored |= 1u << s;
    }
}
#endif

/* ---------------------------------------------------------------------------
 * Worker: one blocking job at a time, off the RX loop so STOP is always heard
 * ------------------------------------------------------------------------- */
static void worker_task(void *arg)
{
    const uint8_t command = (uint8_t)(uintptr_t)arg;
    esp_err_t err;

    switch (command) {
    case CMD_MAP:
        teardown_controller();
        forget_steps_from(STEP_MAP);
        erase_steps_from(STEP_MAP);
        err = discover_phases();
        if (err == ESP_OK) {
            steps_done |= 1u << STEP_MAP;
            save_step(STEP_MAP);
        }
        break;
    case CMD_IDENT:
        teardown_controller();
        forget_steps_from(STEP_IDENT);
        erase_steps_from(STEP_IDENT);
        err = identify_motor();
        if (err == ESP_OK) {
            steps_done |= 1u << STEP_IDENT;
        }
        break;
    case CMD_COG:
        /* The table in NVS goes when the new one is saved, or when the
         * learn fails: either way with the bridge off. */
        forget_steps_from(STEP_COG);
        err = learn_cogging();
        if (err == ESP_OK) {
            steps_done |= 1u << STEP_COG;
        }
        break;
    case CMD_MODE:
        err = start_controller();
        if (err == ESP_OK) {
            if (active_mode == ESP_FOC_SD_MODE_TORQUE) {
                err = esp_foc_sensored_set_iq(MOTOR_AXIS, 0.0f);
            } else if (active_mode == ESP_FOC_SD_MODE_VELOCITY) {
                err = esp_foc_sensored_set_speed_ref_hz(MOTOR_AXIS, 0.0f);
            }
        }
        break;
    default:
        err = ESP_ERR_NOT_SUPPORTED;
        break;
    }
    send_ack(command, err);
    busy = BUSY_NONE;
    esp_foc_task_delete_self();
}

static esp_err_t start_job(uint8_t command, busy_t what)
{
    busy = (uint8_t)what;
    if (esp_foc_task_spawn(worker_task, (void *)(uintptr_t)command, "gui_job", WORKER_STACK,
                           WORKER_PRIORITY, NULL) != 0) {
        busy = BUSY_NONE;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/* ---------------------------------------------------------------------------
 * Commands
 * ------------------------------------------------------------------------- */
static float clampf(float value, float limit)
{
    return (value > limit) ? limit : ((value < -limit) ? -limit : value);
}

static esp_err_t apply_reference(float value)
{
    switch ((esp_foc_sensored_mode_t)active_mode) {
    case ESP_FOC_SD_MODE_TORQUE:
        return esp_foc_sensored_set_iq(MOTOR_AXIS, clampf(value, GUI_MAX_IQ_A));
    case ESP_FOC_SD_MODE_VELOCITY:
        return esp_foc_sensored_set_speed_ref_hz(
            MOTOR_AXIS, rpm_to_hz(clampf(value, GUI_MAX_RPM)));
    case ESP_FOC_SD_MODE_POSITION:
        return esp_foc_sensored_set_position_ref_rad(
            MOTOR_AXIS, (clampf(value, GUI_MAX_DEG) + (float)turns_folded * 360.0f) / DEG_PER_RAD);
    default:
        return ESP_ERR_INVALID_STATE;
    }
}

static void stop(void)
{
    if ((busy == BUSY_MAP) || (busy == BUSY_IDENT)) {
        log_line("stop: cannot cancel this step, rebooting");
        inverter->disable(inverter);
        esp_foc_sleep_ms(50);
        esp_foc_reboot();
    }
    if (!controller_ready) {
        return;
    }
    if (esp_foc_sensored_get_state(MOTOR_AXIS) == ESP_FOC_SD_STATE_FAULT) {
        esp_foc_sensored_clear_fault(MOTOR_AXIS);
        log_line("stop: fault cleared, controller idle");
    } else {
        esp_foc_sensored_stop(MOTOR_AXIS);
        log_line("stop: bridge off, controller idle");
    }
}

static void handle_command(uint8_t type, const uint8_t *payload, uint8_t len)
{
    esp_err_t err = ESP_OK;

    switch (type) {
    case CMD_PING:
        send_info();
        report_restored();
        return;
    case CMD_STOP:
        stop();
        break;
    /* A step runs again whenever the steps before it are done; the worker
     * discards it and the steps after it first. */
    case CMD_MAP:
        err = (busy != BUSY_NONE) ? ESP_ERR_INVALID_STATE : start_job(CMD_MAP, BUSY_MAP);
        if (err == ESP_OK) {
            return; /* the worker acknowledges when the step ends */
        }
        break;
    case CMD_IDENT:
        err = ((busy != BUSY_NONE) || ((steps_done & (1u << STEP_MAP)) == 0u))
                  ? ESP_ERR_INVALID_STATE
                  : start_job(CMD_IDENT, BUSY_IDENT);
        if (err == ESP_OK) {
            return;
        }
        break;
    case CMD_COG:
        if ((busy != BUSY_NONE) || ((steps_done & ((1u << STEP_MAP) | (1u << STEP_IDENT))) !=
                                    ((1u << STEP_MAP) | (1u << STEP_IDENT)))) {
            err = ESP_ERR_INVALID_STATE;
            break;
        }
        err = start_job(CMD_COG, BUSY_COG);
        if (err == ESP_OK) {
            return;
        }
        break;
    case CMD_MODE:
        if ((len != 1u) || (payload[0] > ESP_FOC_SD_MODE_POSITION)) {
            err = ESP_ERR_INVALID_ARG;
            break;
        }
        if ((busy != BUSY_NONE) || ((steps_done & (1u << STEP_COG)) == 0u)) {
            err = ESP_ERR_INVALID_STATE;
            break;
        }
        const uint8_t was = active_mode;
        active_mode = payload[0];
        if (esp_foc_sensored_get_state(MOTOR_AXIS) == ESP_FOC_SD_STATE_RUNNING) {
            err = esp_foc_sensored_set_mode(MOTOR_AXIS, (esp_foc_sensored_mode_t)active_mode);
            if ((err == ESP_OK) && (active_mode == ESP_FOC_SD_MODE_TORQUE)) {
                err = esp_foc_sensored_set_iq(MOTOR_AXIS, 0.0f);
            } else if ((err == ESP_OK) && (active_mode == ESP_FOC_SD_MODE_VELOCITY)) {
                err = esp_foc_sensored_set_speed_ref_hz(MOTOR_AXIS, 0.0f);
            }
            if ((err == ESP_OK) && (was != ESP_FOC_SD_MODE_POSITION) &&
                (active_mode == ESP_FOC_SD_MODE_POSITION)) {
                fold_turns();
            }
        } else if (esp_foc_sensored_get_state(MOTOR_AXIS) == ESP_FOC_SD_STATE_IDLE) {
            /* After a STOP: start the controller again, straight in this mode. */
            err = start_job(CMD_MODE, BUSY_START);
            if (err == ESP_OK) {
                return;
            }
        } else {
            err = ESP_ERR_INVALID_STATE;
        }
        break;
    case CMD_REF: {
        float value;
        if (len != sizeof(value)) {
            err = ESP_ERR_INVALID_ARG;
            break;
        }
        memcpy(&value, payload, sizeof(value));
        if ((busy != BUSY_NONE) || ((steps_done & (1u << STEP_COG)) == 0u) || !isfinite(value)) {
            err = ESP_ERR_INVALID_STATE;
            break;
        }
        err = apply_reference(value);
        break;
    }
    default:
        err = ESP_ERR_NOT_SUPPORTED;
        break;
    }
    send_ack(type, err);
}

/* ---------------------------------------------------------------------------
 * RX: byte-wise frame parser. Anything that is not a valid frame (a terminal
 * typing, a truncated frame) is skipped until the next 0xA5.
 * ------------------------------------------------------------------------- */
typedef enum {
    RX_SYNC,
    RX_TYPE,
    RX_LEN,
    RX_PAYLOAD,
    RX_CRC,
} rx_state_t;

static void rx_loop(void)
{
    rx_state_t state = RX_SYNC;
    /* type, len, payload: the bytes the CRC covers, back to back. */
    uint8_t frame[2 + FRAME_MAX_PAYLOAD];
    uint8_t received = 0;
    uint8_t chunk[64];

    for (;;) {
        const int n = uart_read_bytes(CONSOLE_UART, chunk, sizeof(chunk), esp_foc_ms_to_ticks(20));
        for (int i = 0; i < n; i++) {
            const uint8_t byte = chunk[i];
            switch (state) {
            case RX_SYNC:
                state = (byte == FRAME_SYNC) ? RX_TYPE : RX_SYNC;
                break;
            case RX_TYPE:
                frame[0] = byte;
                state = RX_LEN;
                break;
            case RX_LEN:
                frame[1] = byte;
                received = 0;
                state = (byte > FRAME_MAX_PAYLOAD) ? RX_SYNC : ((byte == 0u) ? RX_CRC : RX_PAYLOAD);
                break;
            case RX_PAYLOAD:
                frame[2 + received++] = byte;
                if (received == frame[1]) {
                    state = RX_CRC;
                }
                break;
            case RX_CRC:
                if (crc8(frame, 2u + frame[1]) == byte) {
                    handle_command(frame[0], &frame[2], frame[1]);
                }
                state = RX_SYNC;
                break;
            }
        }
    }
}

/* ------------------------------------------------------------------------- */
void app_main(void)
{
    /* Plain text before the protocol starts, for whoever opens a terminal. */
    printf("\nespFoC sensored GUI example: binary protocol on this UART from here on.\n"
           "Run examples/foc_sensored_gui/host/espfoc_gui.py on the host.\n");
    fflush(stdout);

    ESP_ERROR_CHECK(uart_driver_install(CONSOLE_UART, UART_RX_BUFFER, UART_TX_BUFFER, 0, NULL, 0));
    esp_log_set_vprintf(log_to_frame);

    esp_err_t err = setup_inverter();
    if (err == ESP_OK) {
        err = setup_encoder();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "board setup failed (%s); check the Kconfig pins", esp_err_to_name(err));
        for (;;) {
            esp_foc_sleep_ms(1000);
        }
    }

    send_info();
#if CONFIG_EXAMPLE_GUI_PERSIST
    restore_commissioning();
#endif
    if (steps_done & (1u << STEP_COG)) {
        log_line("board ready: pick a mode to start, or run any step again");
    } else if (steps_done != 0u) {
        log_line("board ready: continue with the next step, or run any step again");
    } else {
        log_line("board ready: press 1. Phase map");
    }
    if (esp_foc_task_spawn(telemetry_task, NULL, "gui_tel", 3072, TELEMETRY_PRIORITY, NULL) != 0) {
        ESP_LOGE(TAG, "telemetry task not started");
    }
    rx_loop();
}
