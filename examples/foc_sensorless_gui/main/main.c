/*
 * SPDX-FileCopyrightText: 2026 Felipe Neves
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Sensorless motor control driven from a desktop GUI
 * (examples/foc_sensored_gui/host/espfoc_gui.py, which adapts to this board
 * from its INFO frame).
 *
 * The board does nothing on its own. The host walks it through
 * commissioning, one step per button:
 *
 *   1. MAP    phase discovery from the currents alone: phase order and
 *             current signs.
 *   2. IDENT  motor identification (R, L, flux), then the sensorless
 *             controller is built from it and armed.
 *
 * There is no rotor sensor. The controller starts the motor open loop
 * (align, I-f ramp) until its flux observer locks, then hands Park over to
 * the observer. START lets the current reference launch that sequence;
 * every step of it is reported in a STARTUP frame.
 *
 * Torque or speed mode is fixed when the controller is built, so changing
 * mode rebuilds it and is only accepted with the motor stopped.
 *
 * STOP is a dry cut back to armed and clears a latched trip. Phase
 * discovery and identification cannot be cancelled half way, so STOP during
 * them reboots the chip; the bridge stays disabled across the reset.
 */

#include <math.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hal/uart_ll.h"
#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "espFoC/drivers/esp_foc_inverter_mcpwm.h"
#include "espFoC/motor_control/esp_foc_motor_id.h"
#include "espFoC/motor_control/esp_foc_phase_discover.h"
#include "espFoC/motor_control/esp_foc_sensorless.h"
#include "espFoC/osal/esp_foc_osal.h"
#include "commissioning_store.h"

#define MOTOR_AXIS 0u

#define PWM_RATE_HZ ((uint32_t)CONFIG_ESP_FOC_PWM_RATE_HZ)
#define POLE_PAIRS CONFIG_EXAMPLE_MOTOR_POLE_PAIRS
#define DC_LINK_VOLTS ((float)CONFIG_EXAMPLE_DC_LINK_MILLIVOLT * 1.0e-3f)
/* The controller works in electrical Hz: shaft rev/s times pole pairs. */
#define RPM_TO_HZ_EL (1.0f / 60.0f * (float)POLE_PAIRS)
#define RATED_ELECTRICAL_HZ ((float)CONFIG_EXAMPLE_RATED_SPEED_RPM * RPM_TO_HZ_EL)

#if CONFIG_EXAMPLE_BRIDGE_ENABLE_ACTIVE_LOW
#define BRIDGE_ENABLE_ACTIVE_LOW true
#else
#define BRIDGE_ENABLE_ACTIVE_LOW false
#endif

#define GUI_MIN_RPM ((float)CONFIG_EXAMPLE_GUI_MIN_RPM)
#define GUI_MAX_RPM ((float)CONFIG_EXAMPLE_GUI_MAX_RPM)
#define GUI_MIN_IQ_A ((float)CONFIG_EXAMPLE_GUI_MIN_IQ_MA * 1.0e-3f)
#define GUI_MAX_IQ_A ((float)CONFIG_EXAMPLE_GUI_MAX_IQ_MA * 1.0e-3f)
#define GUI_ACCEL_HZ_S ((float)CONFIG_EXAMPLE_GUI_ACCEL_RPM_S * RPM_TO_HZ_EL)

#define TWO_PI (2.0f * (float)M_PI)
#define DEG_PER_RAD (180.0f / (float)M_PI)
#define RPM_PER_RADS_EL (60.0f / TWO_PI / (float)POLE_PAIRS)

/* The console UART is driven straight through its 128-byte FIFOs, polled
 * from tasks: no UART driver, so no UART interrupt and no critical section
 * that would hold off the PWM interrupt, which leaves little CPU spare in
 * sensorless. At 115200 baud the RX FIFO fills in ~11 ms; polled every 5. */
#define CONSOLE_UART_HW UART_LL_GET_HW(CONFIG_ESP_CONSOLE_UART_NUM)
#define RX_POLL_MS 5u

#define TELEMETRY_PERIOD_MS 100u
#define WORKER_STACK 8192u
#define WORKER_PRIORITY 4
#define TELEMETRY_PRIORITY 3

/* ---------------------------------------------------------------------------
 * Wire protocol, shared with examples/foc_sensored_gui (its README has the
 * full layout):
 *
 *   0xA5 | type | len | payload[len] | crc8(type, len, payload)
 * ------------------------------------------------------------------------- */
#define FRAME_SYNC 0xA5u
#define FRAME_MAX_PAYLOAD 64u

/* Host -> board */
#define CMD_PING 0x01u
#define CMD_MAP 0x02u
#define CMD_IDENT 0x03u
#define CMD_COG 0x04u   /* not on this board: ESP_ERR_NOT_SUPPORTED */
#define CMD_MODE 0x05u  /* u8 mode: 0 torque, 1 speed */
#define CMD_REF 0x06u   /* f32 reference: A or rpm, signed */
#define CMD_STOP 0x07u
#define CMD_START 0x08u /* let the reference start the motor */

/* Board -> host */
#define MSG_INFO 0x81u
#define MSG_TEL 0x82u
#define MSG_PROG 0x83u
#define MSG_LOG 0x84u
#define MSG_ACK 0x85u
#define MSG_STARTUP 0x86u /* see on_controller_event() */

/* INFO kind byte: tells the host which controls to show. */
#define BOARD_KIND_SENSORLESS 1u

#define MODE_TORQUE 0u
#define MODE_SPEED 1u

typedef enum {
    STEP_MAP = 0,
    STEP_IDENT,
    STEP_COUNT,
} step_t;

typedef enum {
    BUSY_NONE = 0,
    BUSY_MAP,
    BUSY_IDENT,
} busy_t;

#define TEL_FLAG_RUNNING (1u << 0)
#define TEL_FLAG_ABORT (1u << 1)
#define TEL_FLAG_FAULT (1u << 2)
#define TEL_FLAG_LOCKED (1u << 3)    /* observer locked */
#define TEL_FLAG_OBSERVER (1u << 4)  /* Park on the observer angle */
#define TEL_FLAG_START_FAIL (1u << 5)
#define TEL_FLAG_LAUNCH (1u << 6)    /* START given, references reach the controller */

static const char *TAG = "gui";

static esp_foc_inverter_t *inverter;

/* What commissioning measured, one member per step: enough to rebuild the
 * controller without running a step again. */
typedef struct {
    esp_foc_phase_discover_result_t phase_map;
    esp_foc_motor_id_result_t plant;
} commissioning_t;

static commissioning_t record;
static esp_foc_sensorless_config_t controller_config;

static volatile uint8_t steps_done;
static volatile uint8_t busy;
static volatile bool controller_ready;
static volatile uint8_t active_mode = MODE_SPEED;
/* The last reference from the host, applied once START was given. */
static volatile float reference;
static volatile bool launch_enabled;
static uint64_t startup_t0_us;

/* ------------------------------------------------------------------------- */

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

/* Several tasks send frames. The flag keeps a frame whole without a
 * critical section; a writer that finds it taken, or the FIFO short of
 * room, sleeps a tick. */
static atomic_flag tx_busy = ATOMIC_FLAG_INIT;

static void uart_write_frame(const uint8_t *frame, size_t len)
{
    while (atomic_flag_test_and_set_explicit(&tx_busy, memory_order_acquire)) {
        esp_foc_sleep_ms(1);
    }
    while (uart_ll_get_txfifo_len(CONSOLE_UART_HW) < len) {
        esp_foc_sleep_ms(1);
    }
    uart_ll_write_txfifo(CONSOLE_UART_HW, frame, (uint32_t)len);
    atomic_flag_clear_explicit(&tx_busy, memory_order_release);
}

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
    uart_write_frame(frame, len + 4u);
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

/* The sensored INFO (pole pairs, max rpm, max iq, max deg) followed by the
 * kind byte and the lowest usable references. */
static void send_info(void)
{
    uint8_t payload[22];

    payload[0] = (uint8_t)POLE_PAIRS;
    put_f32(&payload[1], GUI_MAX_RPM);
    put_f32(&payload[5], GUI_MAX_IQ_A);
    put_f32(&payload[9], 0.0f);
    payload[13] = BOARD_KIND_SENSORLESS;
    put_f32(&payload[14], GUI_MIN_RPM);
    put_f32(&payload[18], GUI_MIN_IQ_A);
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
 * Telemetry: the sensored TEL layout. State is esp_foc_sensorless_state_t,
 * flags are TEL_FLAG_* above, the position fields are 0.
 * ------------------------------------------------------------------------- */
static void send_telemetry(void)
{
    uint8_t payload[35];
    esp_foc_sensorless_status_t status;
    uint8_t flags = launch_enabled ? TEL_FLAG_LAUNCH : 0u;

    memset(&status, 0, sizeof(status));
    status.vdc_v = DC_LINK_VOLTS;
    if (controller_ready) {
        esp_foc_sensorless_get_status(MOTOR_AXIS, &status);
        flags |= (status.state == ESP_FOC_SL_STATE_RUNNING) ? TEL_FLAG_RUNNING : 0u;
        flags |= (status.abort != ESP_FOC_SL_ABORT_NONE) ? TEL_FLAG_ABORT : 0u;
        flags |= (status.fault != ESP_FOC_FAULT_NONE) ? TEL_FLAG_FAULT : 0u;
        flags |= (status.fail != ESP_FOC_SL_FAIL_NONE) ? TEL_FLAG_START_FAIL : 0u;
        /* The controller keeps both after a cut; they only mean something
         * while it drives the bridge. */
        if ((status.state > ESP_FOC_SL_STATE_ARMED) && (status.state < ESP_FOC_SL_STATE_FAULT)) {
            flags |= status.observer_locked ? TEL_FLAG_LOCKED : 0u;
            flags |= status.park_on_observer ? TEL_FLAG_OBSERVER : 0u;
        }
        /* Torque mode has no speed reference; show the measurement. */
        if (active_mode == MODE_TORQUE) {
            status.w_ref_rads = status.we_rads;
        }
    }

    payload[0] = steps_done;
    payload[1] = busy;
    payload[2] = controller_ready ? (uint8_t)status.state : 0u;
    payload[3] = active_mode;
    payload[4] = flags;
    put_f32(&payload[5], status.w_ref_rads * RPM_PER_RADS_EL);
    put_f32(&payload[9], status.we_rads * RPM_PER_RADS_EL);
    put_f32(&payload[13], 0.0f);
    put_f32(&payload[17], 0.0f);
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
 * Board bring-up (same wiring as the other examples, no rotor sensor)
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

/* ---------------------------------------------------------------------------
 * Commissioning results in NVS (CONFIG_EXAMPLE_GUI_PERSIST)
 *
 * One blob per step. Running a step discards it and the step after it,
 * which was measured on top of it. Saves and erases happen with the bridge
 * off: the controller is torn down or only armed.
 * ------------------------------------------------------------------------- */
/* Steps running on restored results; reported again on PING. */
static volatile uint8_t steps_restored;

#if CONFIG_EXAMPLE_GUI_PERSIST
static const char *const step_keys[STEP_COUNT] = {"map", "ident"};
static const char *const step_names[STEP_COUNT] = {"phase map", "identification"};

static void *step_data(step_t step, size_t *size)
{
    if (step == STEP_MAP) {
        *size = sizeof(record.phase_map);
        return &record.phase_map;
    }
    *size = sizeof(record.plant);
    return &record.plant;
}

/* Everything a stored result depends on besides the motor itself. */
static uint32_t board_hash(void)
{
    const int32_t board[] = {
        POLE_PAIRS, CONFIG_EXAMPLE_DC_LINK_MILLIVOLT, CONFIG_ESP_FOC_PWM_RATE_HZ,
        CONFIG_EXAMPLE_PWM_DEAD_TIME_NS, CONFIG_EXAMPLE_SHUNT_MILLIOHM,
        CONFIG_EXAMPLE_CURRENT_AMP_GAIN, CONFIG_EXAMPLE_PWM_U_HIGH_GPIO,
        CONFIG_EXAMPLE_PWM_U_LOW_GPIO, CONFIG_EXAMPLE_PWM_V_HIGH_GPIO,
        CONFIG_EXAMPLE_PWM_V_LOW_GPIO, CONFIG_EXAMPLE_PWM_W_HIGH_GPIO,
        CONFIG_EXAMPLE_PWM_W_LOW_GPIO, CONFIG_EXAMPLE_CURRENT_U_GPIO,
        CONFIG_EXAMPLE_CURRENT_V_GPIO, CONFIG_EXAMPLE_CURRENT_W_GPIO,
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
    log_line("nvs: running on saved commissioning:%s%s; any step can run again",
             (mask & (1u << STEP_MAP)) ? " map" : "", (mask & (1u << STEP_IDENT)) ? " ident" : "");
}

/* ---------------------------------------------------------------------------
 * Commissioning step 1: phase discovery, currents only
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
        send_progress(STEP_MAP, (uint8_t)((event->idx + 1) * 90 / 12));
        break;
    case ESP_FOC_PHASE_DISCOVER_EV_REFUSED:
        log_line("map: attempt refused (%s)", esp_err_to_name(event->err));
        break;
    case ESP_FOC_PHASE_DISCOVER_EV_MAP:
        send_progress(STEP_MAP, 100);
        break;
    default:
        break;
    }
}

/* Puts the inverter where phase discovery left it. */
static esp_err_t apply_phase_map(void)
{
    static const char letters[] = "UVW";
    const esp_foc_phase_map_t *map = &record.phase_map.map;

    esp_err_t err = inverter->set_phase_map(inverter, map);
    if (err != ESP_OK) {
        log_line("map: not applied (%s)", esp_err_to_name(err));
        return err;
    }
    log_line("map: U->%c(%+d) V->%c(%+d) W->%c(%+d)", letters[map->pwm_to_hw[0] % 3u],
             map->i_sign[0], letters[map->pwm_to_hw[1] % 3u], map->i_sign[1],
             letters[map->pwm_to_hw[2] % 3u], map->i_sign[2]);
    return ESP_OK;
}

static esp_err_t discover_phases(void)
{
    esp_foc_phase_discover_t discovery;
    esp_foc_phase_discover_config_t discovery_config;

    log_line("map: finding the phase order");
    /* init() and cleanup() read the object's flags before setting them. */
    memset(&discovery, 0, sizeof(discovery));
    esp_foc_phase_discover_default_config(&discovery_config);
    discovery_config.on_event = on_discover_event;
    memset(&record.phase_map, 0, sizeof(record.phase_map));
    esp_err_t err = esp_foc_phase_discover_init(&discovery, inverter, NULL, &discovery_config);
    if (err == ESP_OK) {
        err = esp_foc_phase_discover_run(&discovery, &record.phase_map);
    }
    esp_foc_phase_discover_cleanup(&discovery);
    if (err != ESP_OK) {
        log_line("map: failed (%s)", esp_err_to_name(err));
        return err;
    }
    return apply_phase_map();
}

/* ---------------------------------------------------------------------------
 * The controller and its start-up events
 *
 * STARTUP payload, 17 bytes, one frame per controller event:
 *   u8  event (esp_foc_sensorless_ev_t)
 *   u8  state at the event (esp_foc_sensorless_state_t)
 *   u8  start-up failure (esp_foc_sensorless_fail_t)
 *   u8  abort reason (esp_foc_sensorless_abort_t)
 *   u8  fault reason (esp_foc_fault_reason_t)
 *   f32 observer speed [rpm]
 *   f32 observer minus open-loop angle [deg]
 *   f32 time since the start-up began [s]
 * ------------------------------------------------------------------------- */
static void on_controller_event(void *context, const esp_foc_sensorless_event_t *event)
{
    (void)context;
    uint8_t payload[17];
    const float rpm = event->we_rads * RPM_PER_RADS_EL;

    if (event->ev == ESP_FOC_SL_EV_STARTUP) {
        startup_t0_us = esp_foc_now_us();
    }
    payload[0] = (uint8_t)event->ev;
    payload[1] = (uint8_t)event->state;
    payload[2] = (uint8_t)event->fail;
    payload[3] = (uint8_t)event->abort;
    payload[4] = (uint8_t)event->fault;
    put_f32(&payload[5], rpm);
    put_f32(&payload[9], event->dang_rad * DEG_PER_RAD);
    put_f32(&payload[13], (float)(esp_foc_now_us() - startup_t0_us) * 1.0e-6f);
    send_frame(MSG_STARTUP, payload, sizeof(payload));

    switch (event->ev) {
    case ESP_FOC_SL_EV_STARTUP:
        log_line("startup: align, then open-loop ramp (%s)", event->dir < 0 ? "reverse" : "forward");
        break;
    case ESP_FOC_SL_EV_LOCKED:
        log_line("startup: observer locked at %.0f rpm, angle error %.1f deg", (double)rpm,
                 (double)(event->dang_rad * DEG_PER_RAD));
        break;
    case ESP_FOC_SL_EV_HANDOFF:
        log_line("startup: handoff at %.0f rpm, Park on the observer", (double)rpm);
        break;
    case ESP_FOC_SL_EV_RUNNING:
        /* The start-up used the controller's own slow slew; the GUI's
         * references move at the configured rate from here on. */
        if (active_mode == MODE_SPEED) {
            esp_foc_sensorless_set_speed_slew(MOTOR_AXIS, GUI_ACCEL_HZ_S);
        }
        log_line("startup: running");
        break;
    case ESP_FOC_SL_EV_REVERSING:
        log_line("reversing: slowing down, relaunching the other way");
        break;
    case ESP_FOC_SL_EV_CUT:
        log_line("controller: bridge off, coasting");
        break;
    case ESP_FOC_SL_EV_STARTUP_FAILED:
        log_line("startup: failed (reason %d, in state %d); STOP to clear", (int)event->fail,
                 (int)event->state);
        break;
    case ESP_FOC_SL_EV_ABORT:
        log_line("controller: abort %d, bridge off; STOP to clear", (int)event->abort);
        break;
    case ESP_FOC_SL_EV_FAULT:
        log_line("controller: fault %d, bridge off; STOP to clear", (int)event->fault);
        break;
    default:
        break;
    }
}

/* Builds, initialises and arms the controller in the active mode. */
static esp_err_t build_controller(void)
{
    esp_foc_sensorless_default_config(&controller_config);
    controller_config.axis = MOTOR_AXIS;
    controller_config.pole_pairs = POLE_PAIRS;
    controller_config.fe_rated_hz = RATED_ELECTRICAL_HZ;
    esp_foc_sensorless_config_from_motor_id(&controller_config, &record.plant);
    esp_foc_sensorless_config_from_phase_map(&controller_config, &record.phase_map);
    controller_config.speed_loop = (active_mode == MODE_SPEED);
    /* Up to 10 Hz electrical the spin-up drives voltage (V/f) rather than
     * current, which breaks a cogging rotor loose. */
    controller_config.startup.vf_hz = 10.0f;
    controller_config.on_event = on_controller_event;

    esp_err_t err = esp_foc_sensorless_init(inverter, &controller_config);
    if (err != ESP_OK) {
        log_line("controller: init failed (%s)", esp_err_to_name(err));
        return err;
    }
    controller_ready = true;
    err = esp_foc_sensorless_run(MOTOR_AXIS);
    if (err != ESP_OK) {
        controller_ready = false;
        esp_foc_sensorless_deinit(MOTOR_AXIS);
        log_line("controller: did not arm (%s)", esp_err_to_name(err));
    }
    return err;
}

/* Builds and arms the controller from the identified plant. */
static esp_err_t apply_plant(void)
{
    esp_err_t err = build_controller();
    if (err == ESP_OK) {
        log_line("ident: Rs %.3f ohm, Ls %.0f uH, psi %.0f uWb", (double)controller_config.rs_ohm,
                 (double)(controller_config.ls_h * 1.0e6f),
                 (double)(controller_config.psi_wb * 1.0e6f));
    }
    return err;
}

static void teardown_controller(void)
{
    controller_ready = false;
    esp_foc_sensorless_deinit(MOTOR_AXIS);
}

/* Bridge off and controller gone, from any state: a step that runs again
 * owns the inverter next. */
static void drop_controller(void)
{
    if (!controller_ready) {
        return;
    }
    launch_enabled = false;
    reference = 0.0f;
    esp_foc_sensorless_stop(MOTOR_AXIS);
    teardown_controller();
}

/* ---------------------------------------------------------------------------
 * Commissioning step 2: identification, then the controller
 * ------------------------------------------------------------------------- */
static uint8_t ident_percent;

static void on_motor_id_event(void *context, const esp_foc_motor_id_event_t *event)
{
    (void)context;
    const unsigned first = ESP_FOC_MOTOR_ID_BIAS;
    const unsigned span = ESP_FOC_MOTOR_ID_DONE - ESP_FOC_MOTOR_ID_BIAS;

    if ((event->ev == ESP_FOC_MOTOR_ID_EV_PHASE) && (event->phase >= ESP_FOC_MOTOR_ID_BIAS) &&
        (event->phase <= ESP_FOC_MOTOR_ID_DONE)) {
        /* Phases do not run in enum order and BIAS repeats before every
         * leg, so the bar only moves forward. */
        const uint8_t pct = (uint8_t)(((unsigned)event->phase - first) * 100u / span);
        if (pct > ident_percent) {
            ident_percent = pct;
            send_progress(STEP_IDENT, pct);
        }
        log_line("ident: %s", esp_foc_motor_id_phase_name(event->phase));
    }
}

static esp_err_t identify_motor(void)
{
    esp_foc_motor_id_config_t id_config;

    log_line("ident: about 20 s, the shaft will spin");
    ident_percent = 0;
    send_progress(STEP_IDENT, 0);
    esp_foc_motor_id_default_config(&id_config);
    id_config.pole_pairs = POLE_PAIRS;
    id_config.vdc = DC_LINK_VOLTS;
    id_config.on_event = on_motor_id_event;
    /* No rotor sensor: identification runs on the currents and voltages. */
    esp_err_t err = esp_foc_motor_id_run(inverter, NULL, &id_config, &record.plant);
    if (err != ESP_OK) {
        log_line("ident: failed at '%s' (%s)",
                 esp_foc_motor_id_phase_name(record.plant.failed_at), esp_err_to_name(err));
        return err;
    }
    send_progress(STEP_IDENT, 100);

    err = apply_plant();
    if (err == ESP_OK) {
        save_step(STEP_IDENT);
        log_line("controller: armed, press Start");
    }
    return err;
}

/* ---------------------------------------------------------------------------
 * Restore on boot: each saved step is applied in order, through the same
 * apply_* calls the steps end with. The first step missing or refused ends
 * the restore and discards the step after it. The controller ends armed,
 * bridge off, as after identification.
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
            err = (s == STEP_MAP) ? apply_phase_map() : apply_plant();
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

    if (command == CMD_MAP) {
        drop_controller();
        forget_steps_from(STEP_MAP);
        erase_steps_from(STEP_MAP);
        err = discover_phases();
        if (err == ESP_OK) {
            steps_done |= 1u << STEP_MAP;
            save_step(STEP_MAP);
        }
    } else {
        drop_controller();
        forget_steps_from(STEP_IDENT);
        erase_steps_from(STEP_IDENT);
        err = identify_motor();
        if (err == ESP_OK) {
            steps_done |= 1u << STEP_IDENT;
        }
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
#define COMMISSIONED ((1u << STEP_MAP) | (1u << STEP_IDENT))

static float clampf(float value, float limit)
{
    return (value > limit) ? limit : ((value < -limit) ? -limit : value);
}

/* A reference under the minimum is a stop request to the controller: it
 * cuts to armed. The sign is the direction; a sign change while running
 * slows down and relaunches the other way. */
static esp_err_t apply_reference(void)
{
    if (active_mode == MODE_TORQUE) {
        return esp_foc_sensorless_set_iq(MOTOR_AXIS, clampf(reference, GUI_MAX_IQ_A));
    }
    return esp_foc_sensorless_set_speed_ref_hz(MOTOR_AXIS,
                                               clampf(reference, GUI_MAX_RPM) * RPM_TO_HZ_EL);
}

static bool reference_can_start(void)
{
    const float magnitude = fabsf(reference);
    return (active_mode == MODE_TORQUE) ? (magnitude >= GUI_MIN_IQ_A) : (magnitude >= GUI_MIN_RPM);
}

static bool tripped(void)
{
    esp_foc_sensorless_status_t status;
    esp_foc_sensorless_get_status(MOTOR_AXIS, &status);
    return (status.state == ESP_FOC_SL_STATE_FAULT) || (status.fail != ESP_FOC_SL_FAIL_NONE) ||
           (status.abort != ESP_FOC_SL_ABORT_NONE) || (status.fault != ESP_FOC_FAULT_NONE);
}

static void stop(void)
{
    if ((busy == BUSY_MAP) || (busy == BUSY_IDENT)) {
        log_line("stop: cannot cancel this step, rebooting");
        inverter->disable(inverter);
        esp_foc_sleep_ms(50);
        esp_foc_reboot();
    }
    launch_enabled = false;
    if (!controller_ready) {
        return;
    }
    /* Armed with a reference above the cut is a launch: clear it first. */
    esp_foc_sensorless_set_speed_ref_hz(MOTOR_AXIS, 0.0f);
    esp_foc_sensorless_set_iq(MOTOR_AXIS, 0.0f);
    if (tripped()) {
        esp_foc_sensorless_clear_fault(MOTOR_AXIS);
        log_line("stop: trip cleared, armed");
        return;
    }
    /* Dry cut to IDLE, then armed again for the next START. */
    esp_foc_sensorless_stop(MOTOR_AXIS);
    esp_foc_sensorless_run(MOTOR_AXIS);
    log_line("stop: bridge off, armed");
}

static esp_err_t change_mode(uint8_t mode)
{
    const esp_foc_sensorless_state_t state = esp_foc_sensorless_get_state(MOTOR_AXIS);

    if ((state != ESP_FOC_SL_STATE_IDLE) && (state != ESP_FOC_SL_STATE_ARMED)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (mode == active_mode) {
        return ESP_OK;
    }
    /* The mode is part of the controller's design: rebuild it. */
    launch_enabled = false;
    teardown_controller();
    active_mode = mode;
    reference = 0.0f;
    esp_err_t err = build_controller();
    if (err == ESP_OK) {
        log_line("controller: %s mode, armed", (mode == MODE_TORQUE) ? "torque" : "speed");
    }
    return err;
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
    /* A step runs again whenever the step before it is done; the worker
     * discards it and the step after it first. */
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
        err = ESP_ERR_NOT_SUPPORTED;
        break;
    case CMD_MODE:
        if ((len != 1u) || (payload[0] > MODE_SPEED)) {
            err = ESP_ERR_INVALID_ARG;
        } else if ((busy != BUSY_NONE) || (steps_done != COMMISSIONED)) {
            err = ESP_ERR_INVALID_STATE;
        } else {
            err = change_mode(payload[0]);
        }
        break;
    case CMD_REF: {
        float value;
        if (len != sizeof(value)) {
            err = ESP_ERR_INVALID_ARG;
            break;
        }
        memcpy(&value, payload, sizeof(value));
        if ((busy != BUSY_NONE) || (steps_done != COMMISSIONED) || !isfinite(value)) {
            err = ESP_ERR_INVALID_STATE;
            break;
        }
        reference = value;
        if (launch_enabled) {
            err = apply_reference();
        }
        break;
    }
    case CMD_START:
        if ((busy != BUSY_NONE) || (steps_done != COMMISSIONED) ||
            (esp_foc_sensorless_get_state(MOTOR_AXIS) != ESP_FOC_SL_STATE_ARMED) || tripped()) {
            err = ESP_ERR_INVALID_STATE;
        } else if (!reference_can_start()) {
            err = ESP_ERR_INVALID_ARG;
            log_line("start: set a reference of at least %.0f rpm / %.2f A first",
                     (double)GUI_MIN_RPM, (double)GUI_MIN_IQ_A);
        } else {
            launch_enabled = true;
            if (active_mode == MODE_SPEED) {
                esp_foc_sensorless_set_speed_slew(MOTOR_AXIS, controller_config.wref_slew_hz_s);
            }
            err = apply_reference();
        }
        break;
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
        uint32_t n = uart_ll_get_rxfifo_len(CONSOLE_UART_HW);
        if (n == 0u) {
            esp_foc_sleep_ms(RX_POLL_MS);
            continue;
        }
        if (n > sizeof(chunk)) {
            n = sizeof(chunk);
        }
        uart_ll_read_rxfifo(CONSOLE_UART_HW, chunk, n);
        for (uint32_t i = 0; i < n; i++) {
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
    printf("\nespFoC sensorless GUI example: binary protocol on this UART from here on.\n"
           "Run examples/foc_sensored_gui/host/espfoc_gui.py on the host.\n");
    fflush(stdout);

    esp_log_set_vprintf(log_to_frame);

    esp_err_t err = setup_inverter();
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
    if (steps_done == COMMISSIONED) {
        log_line("board ready: armed, set a reference and press Start, or run any step again");
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
