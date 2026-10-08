/*
 * espFoC example: sensored servo
 *
 * A PMSM/BLDC motor on a three-phase bridge with inline current shunts and an
 * AS5600 magnetic encoder on the shaft, run as a position servo. The example
 * commissions the motor from nothing but its pole pairs and the supply
 * voltage, then waits for commands on the console:
 *
 *   move <deg>                      go to an absolute shaft angle
 *   traj <deg>:<s> [<deg>:<s> ...]  pass through waypoints, each reached
 *                                   after the given seconds (same angle twice
 *                                   is a dwell)
 *   where                           print the shaft angle
 *   stop                            stop the controller, bridge off
 *
 * Angles are mechanical degrees from the origin, which is where the shaft was
 * when the servo started. servo.py next to this file sends these commands and
 * generates trajectories (sine, steps); any serial terminal works too.
 *
 * Every move is a minimum-jerk profile: position, speed and acceleration are
 * smooth, start and end at rest, and are fed to the controller together, so
 * the position loop only corrects what the profile did not predict. Each
 * command answers with
 *
 *   at <deg> deg, moving! ...
 *   stopped at <deg> deg (requested <deg> deg, error <deg> deg)
 *
 * Commissioning, every boot, before the servo takes commands:
 *
 *   1. Phase discovery finds which bridge output drives which motor phase,
 *      zeroes the encoder on the rotor's magnetic axis and checks which way
 *      it counts. Motor leads can be connected in any order.
 *   2. Motor identification measures resistance, inductance, flux linkage,
 *      how fast the shaft accelerates per amp, and the encoder's offset and
 *      delay. The controller gains are designed from these numbers.
 *   3. Cogging learn: slow sweeps both ways map the torque the magnets pull
 *      at every shaft angle. The controller cancels it from then on; without
 *      it the shaft settles into the nearest magnetic detent instead of the
 *      requested angle.
 *
 * All three spin the shaft: keep it free and keep hands off. Commissioning
 * takes about 3 to 4 minutes. Pins, motor and move settings are in
 * menuconfig under "espFoC example: sensored servo".
 */
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_err.h"
#include "sdkconfig.h"
#include "espFoC/drivers/esp_foc_inverter_mcpwm.h"
#include "espFoC/drivers/esp_foc_rotor_as5600.h"
#include "espFoC/motor_control/esp_foc_motor_id.h"
#include "espFoC/motor_control/esp_foc_phase_discover.h"
#include "espFoC/motor_control/esp_foc_sensored.h"
#include "espFoC/osal/esp_foc_osal.h"

#define MOTOR_AXIS 0u

#define PWM_RATE_HZ ((uint32_t)CONFIG_ESP_FOC_PWM_RATE_HZ)
/* The controller reads the encoder at this rate; it must divide the PWM rate. */
#define ENCODER_RATE_HZ ((uint32_t)CONFIG_ESP_FOC_SD_FETCH_HZ)

#define POLE_PAIRS CONFIG_EXAMPLE_MOTOR_POLE_PAIRS
#define DC_LINK_VOLTS ((float)CONFIG_EXAMPLE_DC_LINK_MILLIVOLT * 1.0e-3f)

#ifdef CONFIG_EXAMPLE_BRIDGE_ENABLE_ACTIVE_LOW
#define BRIDGE_ENABLE_ACTIVE_LOW true
#else
#define BRIDGE_ENABLE_ACTIVE_LOW false
#endif

#define DEG_PER_RAD (180.0f / (float)M_PI)
#define RAD_PER_DEG ((float)M_PI / 180.0f)

#define MOVE_MAX_SPEED_RAD_S ((float)CONFIG_EXAMPLE_MOVE_MAX_SPEED_DEG_S * RAD_PER_DEG)
#define MOVE_MIN_TIME_S ((float)CONFIG_EXAMPLE_MOVE_MIN_TIME_MS * 1.0e-3f)

/*
 * A minimum-jerk move peaks at 1.875 times its average speed, halfway
 * through. Durations are checked against this so the peak, not the average,
 * stays under the configured speed.
 */
#define MIN_JERK_PEAK_SPEED_RATIO 1.875f

/* The profile is sampled at 50 Hz. The position reference steps between
 * samples; the speed and acceleration feedforward keep the motion smooth. */
#define PROFILE_STEP_MS 20u

/* How often the console loop looks for a controller trip while idle. */
#define CONSOLE_POLL_MS 20u

#define MAX_WAYPOINTS 32u
#define COMMAND_LINE_MAX 512u

/* Up to six passes of two-turn sweeps each way at 0.25 rev/s, plus rests. */
#define COGGING_LEARN_TIMEOUT_MS 150000u

typedef struct {
    float target_rad;
    float duration_s;
} waypoint_t;

static esp_foc_inverter_t *inverter;
static esp_foc_rotor_sensor_t *encoder;

/* Amps of torque current per mechanical rad/s^2, from the identified K. */
static float acceleration_feedforward_a;

/*
 * Written by the controller's event callback, which runs on the controller's
 * own task: the main loop only polls them between setpoint updates.
 */
static volatile bool controller_tripped;
static volatile esp_foc_sensored_abort_t trip_abort_reason;
static volatile esp_foc_fault_reason_t trip_fault_reason;
static volatile esp_foc_sensored_fail_t run_fail_reason;

static void print_banner(void)
{
    printf("\n");
    printf("                  ______          _____\n");
    printf("                 |  ____|        / ____|\n");
    printf("  ___  ___ _ __  | |__     ___  | |\n");
    printf(" / _ \\/ __| '_ \\ |  __|   / _ \\ | |\n");
    printf("|  __/\\__ \\ |_) || |     | (_) || |____\n");
    printf(" \\___||___/ .__/ |_|      \\___/  \\_____|\n");
    printf("          | |\n");
    printf("          |_|   sensored servo example\n");
    printf("\n");
}

static void print_gpio_row(const char *function, int gpio)
{
    if (gpio < 0) {
        printf("  | %-26s | %-21s |\n", function, "not wired");
    } else {
        printf("  | %-26s | GPIO %-16d |\n", function, gpio);
    }
}

static void print_board_table(void)
{
    char text[24];

    printf("  Board\n");
    printf("  +----------------------------+-----------------------+\n");
    printf("  | Function                   | Pin / value           |\n");
    printf("  +----------------------------+-----------------------+\n");
    print_gpio_row("PWM phase U high side", CONFIG_EXAMPLE_PWM_U_HIGH_GPIO);
    print_gpio_row("PWM phase U low side", CONFIG_EXAMPLE_PWM_U_LOW_GPIO);
    print_gpio_row("PWM phase V high side", CONFIG_EXAMPLE_PWM_V_HIGH_GPIO);
    print_gpio_row("PWM phase V low side", CONFIG_EXAMPLE_PWM_V_LOW_GPIO);
    print_gpio_row("PWM phase W high side", CONFIG_EXAMPLE_PWM_W_HIGH_GPIO);
    print_gpio_row("PWM phase W low side", CONFIG_EXAMPLE_PWM_W_LOW_GPIO);
    if (CONFIG_EXAMPLE_BRIDGE_ENABLE_GPIO < 0) {
        print_gpio_row("Bridge enable", -1);
    } else {
        snprintf(text, sizeof(text), "GPIO %d, active %s", CONFIG_EXAMPLE_BRIDGE_ENABLE_GPIO,
                 BRIDGE_ENABLE_ACTIVE_LOW ? "low" : "high");
        printf("  | %-26s | %-21s |\n", "Bridge enable", text);
    }
    print_gpio_row("Current sense U (ADC)", CONFIG_EXAMPLE_CURRENT_U_GPIO);
    print_gpio_row("Current sense V (ADC)", CONFIG_EXAMPLE_CURRENT_V_GPIO);
    print_gpio_row("Current sense W (ADC)", CONFIG_EXAMPLE_CURRENT_W_GPIO);
    print_gpio_row("Encoder AS5600 SDA", CONFIG_EXAMPLE_ENCODER_SDA_GPIO);
    print_gpio_row("Encoder AS5600 SCL", CONFIG_EXAMPLE_ENCODER_SCL_GPIO);
    printf("  +----------------------------+-----------------------+\n");
    snprintf(text, sizeof(text), "%u kHz", (unsigned)(PWM_RATE_HZ / 1000u));
    printf("  | %-26s | %-21s |\n", "PWM frequency", text);
    snprintf(text, sizeof(text), "%d ns", CONFIG_EXAMPLE_PWM_DEAD_TIME_NS);
    printf("  | %-26s | %-21s |\n", "Dead time", text);
    snprintf(text, sizeof(text), "%d mOhm x %d", CONFIG_EXAMPLE_SHUNT_MILLIOHM,
             CONFIG_EXAMPLE_CURRENT_AMP_GAIN);
    printf("  | %-26s | %-21s |\n", "Shunt x amplifier gain", text);
    snprintf(text, sizeof(text), "%.2f A", (double)(CONFIG_EXAMPLE_BRIDGE_TRIP_MA * 1.0e-3f));
    printf("  | %-26s | %-21s |\n", "Current trip", text);
    snprintf(text, sizeof(text), "%u Hz", (unsigned)ENCODER_RATE_HZ);
    printf("  | %-26s | %-21s |\n", "Encoder sample rate", text);
    printf("  +----------------------------+-----------------------+\n\n");
}

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

static esp_err_t setup_encoder(void)
{
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
        /* Lets the driver predict the angle between I2C reads at PWM rate. */
        .pwm_hz = PWM_RATE_HZ,
    };
    return esp_foc_rotor_as5600_init(encoder, &encoder_config);
}

static const char *phase_letter(unsigned index)
{
    static const char *const letters[] = {"U", "V", "W"};
    return (index < 3u) ? letters[index] : "?";
}

static esp_err_t discover_phases(esp_foc_phase_discover_result_t *phase_map)
{
    esp_foc_phase_discover_t discovery;
    esp_foc_phase_discover_config_t discovery_config;

    printf("  Phase discovery: finding the phase order and zeroing the encoder...\n");
    esp_foc_phase_discover_default_config(&discovery_config);
    memset(phase_map, 0, sizeof(*phase_map));
    esp_err_t err = esp_foc_phase_discover_init(&discovery, inverter, encoder, &discovery_config);
    if (err == ESP_OK) {
        err = esp_foc_phase_discover_run(&discovery, phase_map);
    }
    esp_foc_phase_discover_cleanup(&discovery);
    if (err != ESP_OK) {
        printf("  Phase discovery failed: %s\n", esp_err_to_name(err));
        return err;
    }

    /* From here on the encoder counts positive in the motor's forward direction. */
    esp_foc_rotor_as5600_set_invert(encoder, phase_map->sensor_reversed);

    printf("\n  Phase map\n");
    printf("  +-------------+---------------+--------------+\n");
    printf("  | Motor phase | Bridge output | Current sign |\n");
    printf("  +-------------+---------------+--------------+\n");
    for (unsigned phase = 0; phase < 3u; phase++) {
        printf("  | %-11s | %-13s | %+12d |\n", phase_letter(phase),
               phase_letter(phase_map->map.pwm_to_hw[phase]), phase_map->map.i_sign[phase]);
    }
    printf("  +-------------+---------------+--------------+\n");
    printf("  Attempts %u, encoder zeroed: %s, encoder direction: %s\n\n",
           (unsigned)phase_map->attempts, phase_map->sensor_zeroed ? "yes" : "no",
           phase_map->sensor_reversed ? "reversed (corrected)" : "as mounted");
    return ESP_OK;
}

static const char *identified_or_not(uint32_t valid_mask, uint32_t bit)
{
    return ((valid_mask & bit) != 0u) ? "identified" : "not fitted";
}

static void print_motor_table(const esp_foc_motor_id_result_t *plant,
                              const esp_foc_sensored_config_t *controller_config)
{
    const uint32_t valid = plant->valid_mask;
    char text[24];

    printf("\n  Motor\n");
    printf("  +----------------------------+-----------------+------------+\n");
    printf("  | Parameter                  | Value           | Source     |\n");
    printf("  +----------------------------+-----------------+------------+\n");
    snprintf(text, sizeof(text), "%d", POLE_PAIRS);
    printf("  | %-26s | %-15s | %-10s |\n", "Pole pairs", text, "Kconfig");
    snprintf(text, sizeof(text), "%.2f V", (double)DC_LINK_VOLTS);
    printf("  | %-26s | %-15s | %-10s |\n", "DC link", text, "Kconfig");
    snprintf(text, sizeof(text), "%.3f ohm", (double)controller_config->rs_ohm);
    printf("  | %-26s | %-15s | %-10s |\n", "Phase resistance", text,
           identified_or_not(valid, ESP_FOC_MOTOR_ID_VALID_R_LOOP | ESP_FOC_MOTOR_ID_VALID_RS));
    snprintf(text, sizeof(text), "%.0f uH", (double)(controller_config->ls_h * 1.0e6f));
    printf("  | %-26s | %-15s | %-10s |\n", "Phase inductance", text,
           identified_or_not(valid, ESP_FOC_MOTOR_ID_VALID_LS));
    snprintf(text, sizeof(text), "%.0f uWb", (double)(controller_config->psi_wb * 1.0e6f));
    printf("  | %-26s | %-15s | %-10s |\n", "Flux linkage", text,
           identified_or_not(valid, ESP_FOC_MOTOR_ID_VALID_PSI_F));
    snprintf(text, sizeof(text), "%.0f rad/s2/A", (double)controller_config->k_rads2_a);
    printf("  | %-26s | %-15s | %-10s |\n", "Acceleration per amp", text,
           identified_or_not(valid, ESP_FOC_MOTOR_ID_VALID_K));
    snprintf(text, sizeof(text), "%.2e kg m2", (double)controller_config->j_kgm2);
    printf("  | %-26s | %-15s | %-10s |\n", "Rotor inertia", text,
           identified_or_not(valid, ESP_FOC_MOTOR_ID_VALID_J));
    snprintf(text, sizeof(text), "%+.2f deg el",
             (double)(controller_config->park_offset_rad * (180.0f / (float)M_PI)));
    printf("  | %-26s | %-15s | %-10s |\n", "Encoder angle offset", text,
           identified_or_not(valid, ESP_FOC_MOTOR_ID_VALID_PARK));
    snprintf(text, sizeof(text), "%.1f us", (double)(controller_config->park_lead_s * 1.0e6f));
    printf("  | %-26s | %-15s | %-10s |\n", "Encoder delay", text,
           identified_or_not(valid, ESP_FOC_MOTOR_ID_VALID_PARK));
    printf("  +----------------------------+-----------------+------------+\n");
}

static esp_err_t identify_motor(esp_foc_motor_id_result_t *plant)
{
    esp_foc_motor_id_config_t id_config;

    printf("  Motor identification: about 90 s, the shaft will spin...\n");
    esp_foc_motor_id_default_config(&id_config);
    id_config.pole_pairs = POLE_PAIRS;
    id_config.vdc = DC_LINK_VOLTS;
    esp_err_t err = esp_foc_motor_id_run(inverter, encoder, &id_config, plant);
    if (err != ESP_OK) {
        printf("  Motor identification failed at '%s': %s\n",
               esp_foc_motor_id_phase_name(plant->failed_at), esp_err_to_name(err));
        return err;
    }
    /* The speed loop under the position loop is designed from K, and the
     * moves' acceleration feedforward is scaled by it. */
    if ((plant->valid_mask & ESP_FOC_MOTOR_ID_VALID_K) == 0u) {
        printf("  Motor identification could not measure the acceleration per amp\n");
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static void on_controller_event(void *context, const esp_foc_sensored_event_t *event)
{
    (void)context;
    switch (event->ev) {
    case ESP_FOC_SD_EV_ABORT:
        trip_abort_reason = event->abort;
        controller_tripped = true;
        break;
    case ESP_FOC_SD_EV_FAULT:
        trip_fault_reason = event->fault;
        controller_tripped = true;
        break;
    case ESP_FOC_SD_EV_RUN_FAIL:
        run_fail_reason = event->fail;
        break;
    default:
        break;
    }
}

static void print_trip_reason(void)
{
    static const char *const abort_names[] = {
        [ESP_FOC_SD_ABORT_NONE] = "none",
        [ESP_FOC_SD_ABORT_OVERSPEED] = "overspeed",
        [ESP_FOC_SD_ABORT_SENSOR_STALE] = "encoder stopped answering",
        [ESP_FOC_SD_ABORT_SENSOR_FAIL] = "encoder read errors",
    };
    static const char *const fault_names[] = {
        [ESP_FOC_FAULT_NONE] = "none",
        [ESP_FOC_FAULT_ILIMIT] = "current trip",
        [ESP_FOC_FAULT_GPIO] = "fault pin",
        [ESP_FOC_FAULT_SOFT_TRIP] = "software trip",
        [ESP_FOC_FAULT_SENSE_STALE] = "current sense stopped",
    };
    const unsigned abort_reason = (unsigned)trip_abort_reason;
    const unsigned fault_reason = (unsigned)trip_fault_reason;
    printf("  Controller tripped: abort '%s', fault '%s'. Bridge is off.\n",
           (abort_reason < 4u) ? abort_names[abort_reason] : "?",
           (fault_reason < 5u) ? fault_names[fault_reason] : "?");
}

static void build_controller_config(esp_foc_sensored_config_t *controller_config,
                                    const esp_foc_phase_discover_result_t *phase_map,
                                    const esp_foc_motor_id_result_t *plant)
{
    esp_foc_sensored_default_config(controller_config);
    controller_config->axis = MOTOR_AXIS;
    controller_config->control = ESP_FOC_SD_CONTROL_POSITION;
    controller_config->pole_pairs = POLE_PAIRS;
    controller_config->fetch_hz = ENCODER_RATE_HZ;
    /* Everything measured: R, L, flux, K, inertia, encoder offset and delay. */
    esp_foc_sensored_config_from_motor_id(controller_config, plant);
    /* Which bridge output drives which phase, and the current signs. */
    esp_foc_sensored_config_from_phase_map(controller_config, phase_map);
    controller_config->guard.overspeed_hz =
        (float)CONFIG_EXAMPLE_OVERSPEED_RPM / 60.0f * (float)POLE_PAIRS;
    controller_config->on_event = on_controller_event;
}

static void print_controller_table(void)
{
    esp_foc_sensored_tuning_t tuning;
    char text[24];

    esp_foc_sensored_get_tuning(MOTOR_AXIS, &tuning);
    printf("\n  Controller (designed from the identified motor)\n");
    printf("  +----------------------------+-----------------+\n");
    snprintf(text, sizeof(text), "%.4f", (double)tuning.kp_i);
    printf("  | %-26s | %-15s |\n", "Current loop Kp", text);
    snprintf(text, sizeof(text), "%.1f", (double)tuning.ki_i);
    printf("  | %-26s | %-15s |\n", "Current loop Ki", text);
    snprintf(text, sizeof(text), "%.3f rad/s el", (double)tuning.ripple_rads);
    printf("  | %-26s | %-15s |\n", "Speed noise at rest", text);
    snprintf(text, sizeof(text), "%.1f Hz", (double)tuning.speed_bw_hz);
    printf("  | %-26s | %-15s |\n", "Speed loop bandwidth", text);
    snprintf(text, sizeof(text), "%.1f Hz", (double)tuning.speed_fc_hz);
    printf("  | %-26s | %-15s |\n", "Speed feedback filter", text);
    snprintf(text, sizeof(text), "%.2e", (double)tuning.kp_w);
    printf("  | %-26s | %-15s |\n", "Speed loop Kp [A/(rad/s)]", text);
    snprintf(text, sizeof(text), "%.2e", (double)tuning.ki_w);
    printf("  | %-26s | %-15s |\n", "Speed loop Ki", text);
    snprintf(text, sizeof(text), "%.2f 1/s", (double)tuning.kp_pos);
    printf("  | %-26s | %-15s |\n", "Position loop Kp", text);
    snprintf(text, sizeof(text), "%d deg/s", CONFIG_EXAMPLE_MOVE_MAX_SPEED_DEG_S);
    printf("  | %-26s | %-15s |\n", "Move peak speed", text);
    printf("  +----------------------------+-----------------+\n");
}

static esp_err_t learn_cogging(void)
{
    esp_foc_sensored_cogging_info_t cogging;
    char text[24];

    printf("\n  Cogging learn: the shaft sweeps slowly both ways, up to 2 min...\n");
    memset(&cogging, 0, sizeof(cogging));
    const esp_err_t err = esp_foc_sensored_cogging_learn(MOTOR_AXIS, COGGING_LEARN_TIMEOUT_MS,
                                                         &cogging);
    if (err != ESP_OK) {
        printf("  Cogging learn failed: %s\n", esp_err_to_name(err));
        return err;
    }

    printf("\n  Cogging\n");
    printf("  +----------------------------+-----------------+\n");
    snprintf(text, sizeof(text), "%u", (unsigned)cogging.passes);
    printf("  | %-26s | %-15s |\n", "Passes", text);
    printf("  | %-26s | %-15s |\n", "Converged", cogging.converged ? "yes" : "no");
    snprintf(text, sizeof(text), "%.1f mA", (double)(cogging.change_rms_a * 1.0e3f));
    printf("  | %-26s | %-15s |\n", "Last pass change (RMS)", text);
    snprintf(text, sizeof(text), "%.1f mA", (double)(cogging.p2p_a * 1.0e3f));
    printf("  | %-26s | %-15s |\n", "Cogging torque, peak-peak", text);
    snprintf(text, sizeof(text), "%u / %u", (unsigned)cogging.bins_mapped,
             (unsigned)cogging.bins);
    printf("  | %-26s | %-15s |\n", "Angle bins mapped", text);
    printf("  +----------------------------+-----------------+\n");
    return ESP_OK;
}

static float shaft_position_deg(void)
{
    esp_foc_sensored_status_t status;
    esp_foc_sensored_get_status(MOTOR_AXIS, &status);
    return status.theta_m_rad * DEG_PER_RAD;
}

/* Where the reference is now: the start of the next segment, so a new
 * command never steps the reference even if the shaft is a hair off it. */
static float reference_position_rad(void)
{
    esp_foc_sensored_status_t status;
    esp_foc_sensored_get_status(MOTOR_AXIS, &status);
    return status.theta_ref_rad;
}

/*
 * Minimum-jerk profile from 0 to distance over duration, at time t:
 * position follows 10s^3 - 15s^4 + 6s^5 of the normalised time s, so speed
 * and acceleration are zero at both ends.
 */
static void min_jerk_sample(float distance, float duration_s, float t_s, float *position,
                            float *speed, float *acceleration)
{
    const float s = t_s / duration_s;
    const float s2 = s * s;
    const float s3 = s2 * s;
    const float s4 = s3 * s;
    const float s5 = s4 * s;
    *position = distance * (10.0f * s3 - 15.0f * s4 + 6.0f * s5);
    *speed = (distance / duration_s) * (30.0f * s2 - 60.0f * s3 + 30.0f * s4);
    *acceleration = (distance / (duration_s * duration_s)) * (60.0f * s - 180.0f * s2 + 120.0f * s3);
}

static bool follow_segment(float start_rad, const waypoint_t *waypoint)
{
    const float distance = waypoint->target_rad - start_rad;
    const uint32_t duration_ms = (uint32_t)lrintf(waypoint->duration_s * 1000.0f);

    for (uint32_t t_ms = PROFILE_STEP_MS; t_ms < duration_ms; t_ms += PROFILE_STEP_MS) {
        if (controller_tripped) {
            return false;
        }
        float position;
        float speed;
        float acceleration;
        min_jerk_sample(distance, waypoint->duration_s, (float)t_ms * 1.0e-3f, &position,
                        &speed, &acceleration);
        esp_foc_sensored_set_joint(MOTOR_AXIS, start_rad + position, speed,
                                   acceleration * acceleration_feedforward_a);
        esp_foc_sleep_ms(PROFILE_STEP_MS);
    }
    esp_foc_sensored_set_joint(MOTOR_AXIS, waypoint->target_rad, 0.0f, 0.0f);
    return !controller_tripped;
}

/* Waits for the controller's in-position flag, then reports where the shaft
 * ended against where it was sent. */
static bool settle_and_report(float requested_rad)
{
    esp_foc_sensored_status_t status;
    uint32_t waited_ms = 0;

    esp_foc_sensored_get_status(MOTOR_AXIS, &status);
    while (!status.inpos && (waited_ms < CONFIG_EXAMPLE_SETTLE_TIMEOUT_MS)) {
        if (controller_tripped) {
            return false;
        }
        esp_foc_sleep_ms(CONSOLE_POLL_MS);
        waited_ms += CONSOLE_POLL_MS;
        esp_foc_sensored_get_status(MOTOR_AXIS, &status);
    }

    const float reached_deg = status.theta_m_rad * DEG_PER_RAD;
    const float requested_deg = requested_rad * DEG_PER_RAD;
    printf("stopped at %.2f deg (requested %.2f deg, error %+.2f deg)%s\n", (double)reached_deg,
           (double)requested_deg, (double)(reached_deg - requested_deg),
           status.inpos ? "" : ", not in position yet");
    return !controller_tripped;
}

static bool run_waypoints(const waypoint_t *waypoints, unsigned count)
{
    float start_rad = reference_position_rad();
    for (unsigned i = 0; i < count; i++) {
        if (!follow_segment(start_rad, &waypoints[i])) {
            return false;
        }
        start_rad = waypoints[i].target_rad;
    }
    return settle_and_report(waypoints[count - 1u].target_rad);
}

static bool move_to(float target_deg)
{
    const float target_rad = target_deg * RAD_PER_DEG;
    const float distance_rad = fabsf(target_rad - reference_position_rad());
    const float duration_for_speed_s = MIN_JERK_PEAK_SPEED_RATIO * distance_rad /
                                       MOVE_MAX_SPEED_RAD_S;
    const waypoint_t waypoint = {
        .target_rad = target_rad,
        .duration_s = fmaxf(MOVE_MIN_TIME_S, duration_for_speed_s),
    };

    printf("at %.2f deg, moving! to %.2f deg in %.2f s\n", (double)shaft_position_deg(),
           (double)target_deg, (double)waypoint.duration_s);
    return run_waypoints(&waypoint, 1u);
}

/*
 * "traj 90:1.5 -45:2 0:1" -> waypoints. Every segment is checked before the
 * shaft moves: a trajectory either runs whole or not at all.
 */
static bool run_trajectory(char *arguments)
{
    static waypoint_t waypoints[MAX_WAYPOINTS];
    unsigned count = 0;
    float previous_rad = reference_position_rad();
    float total_s = 0.0f;

    for (char *token = strtok(arguments, " "); token != NULL; token = strtok(NULL, " ")) {
        char *end = NULL;
        const float target_deg = strtof(token, &end);
        if ((end == token) || (*end != ':')) {
            printf("rejected: waypoint '%s' is not <deg>:<seconds>\n", token);
            return true;
        }
        char *duration_text = end + 1;
        const float duration_s = strtof(duration_text, &end);
        if ((end == duration_text) || (*end != '\0') || !(duration_s > 0.0f)) {
            printf("rejected: waypoint '%s' needs a duration above 0 s\n", token);
            return true;
        }
        if (count == MAX_WAYPOINTS) {
            printf("rejected: more than %u waypoints\n", (unsigned)MAX_WAYPOINTS);
            return true;
        }
        const float target_rad = target_deg * RAD_PER_DEG;
        const float peak_speed_rad_s = MIN_JERK_PEAK_SPEED_RATIO *
                                       fabsf(target_rad - previous_rad) / duration_s;
        if (peak_speed_rad_s > MOVE_MAX_SPEED_RAD_S) {
            printf("rejected: waypoint %u (%.2f deg in %.2f s) peaks at %.0f deg/s, "
                   "limit %d deg/s\n",
                   count + 1u, (double)target_deg, (double)duration_s,
                   (double)(peak_speed_rad_s * DEG_PER_RAD), CONFIG_EXAMPLE_MOVE_MAX_SPEED_DEG_S);
            return true;
        }
        waypoints[count].target_rad = target_rad;
        waypoints[count].duration_s = duration_s;
        previous_rad = target_rad;
        total_s += duration_s;
        count++;
    }
    if (count == 0u) {
        printf("rejected: traj needs at least one <deg>:<seconds>\n");
        return true;
    }

    printf("at %.2f deg, moving! through %u waypoints to %.2f deg in %.2f s\n",
           (double)shaft_position_deg(), count, (double)(previous_rad * DEG_PER_RAD),
           (double)total_s);
    return run_waypoints(waypoints, count);
}

/*
 * The console UART is shared with printf. Installing the UART driver behind
 * stdin gives buffered, interrupt-driven input; non-blocking reads let the
 * loop keep watching for a controller trip while no command arrives.
 */
static void console_input_init(void)
{
    setvbuf(stdin, NULL, _IONBF, 0);
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 2 * COMMAND_LINE_MAX, 0, 0,
                                        NULL, 0));
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    fcntl(fileno(stdin), F_SETFL, O_NONBLOCK);
}

/* Returns false if the controller tripped while waiting for a line. */
static bool read_command_line(char *line, size_t size)
{
    size_t length = 0;

    for (;;) {
        if (controller_tripped) {
            return false;
        }
        const int c = fgetc(stdin);
        if (c == EOF) {
            clearerr(stdin);
            esp_foc_sleep_ms(CONSOLE_POLL_MS);
            continue;
        }
        if ((c == '\r') || (c == '\n')) {
            if (length == 0u) {
                continue;
            }
            line[length] = '\0';
            return true;
        }
        if ((c >= ' ') && (c < 0x7f) && ((length + 1u) < size)) {
            line[length++] = (char)c;
        }
    }
}

static void print_commands(void)
{
    printf("\n  Servo ready. Angles in degrees from the origin. Commands:\n");
    printf("    move <deg>\n");
    printf("    traj <deg>:<s> [<deg>:<s> ...]\n");
    printf("    where\n");
    printf("    stop\n");
    printf("ready\n");
}

/* Returns false when the servo should end: stop command or a trip. */
static bool handle_command(char *line)
{
    if (strcmp(line, "where") == 0) {
        printf("at %.2f deg\n", (double)shaft_position_deg());
        return true;
    }
    if (strcmp(line, "stop") == 0) {
        return false;
    }
    if (strncmp(line, "move ", 5) == 0) {
        char *end = NULL;
        const float target_deg = strtof(line + 5, &end);
        if ((end == line + 5) || (*end != '\0')) {
            printf("rejected: move needs one angle in degrees\n");
            return true;
        }
        return move_to(target_deg);
    }
    if (strncmp(line, "traj ", 5) == 0) {
        return run_trajectory(line + 5);
    }
    printf("rejected: unknown command '%s' (move, traj, where, stop)\n", line);
    return true;
}

static void shut_down(void)
{
    esp_foc_sensored_stop(MOTOR_AXIS);
    esp_foc_sensored_deinit(MOTOR_AXIS);
    inverter->disable(inverter);
}

void app_main(void)
{
    print_banner();
    print_board_table();

    esp_err_t err = setup_inverter();
    if (err != ESP_OK) {
        printf("  Inverter setup failed: %s\n", esp_err_to_name(err));
        return;
    }
    err = setup_encoder();
    if (err != ESP_OK) {
        printf("  Encoder setup failed (check SDA/SCL and the AS5600 supply): %s\n",
               esp_err_to_name(err));
        return;
    }

    esp_foc_phase_discover_result_t phase_map;
    if (discover_phases(&phase_map) != ESP_OK) {
        return;
    }

    esp_foc_motor_id_result_t plant;
    memset(&plant, 0, sizeof(plant));
    if (identify_motor(&plant) != ESP_OK) {
        return;
    }

    esp_foc_sensored_config_t controller_config;
    build_controller_config(&controller_config, &phase_map, &plant);
    print_motor_table(&plant, &controller_config);
    /* K is electrical rad/s^2 per amp; a shaft acceleration is pole pairs
     * times that in electrical terms. */
    acceleration_feedforward_a = (float)POLE_PAIRS / controller_config.k_rads2_a;

    err = esp_foc_sensored_init(inverter, encoder, &controller_config);
    if (err != ESP_OK) {
        printf("  Controller init failed: %s\n", esp_err_to_name(err));
        return;
    }
    /* Arms the bridge, waits for a still shaft, designs the speed loop and
     * holds the position where the shaft is. */
    err = esp_foc_sensored_run(MOTOR_AXIS);
    if (err != ESP_OK) {
        static const char *const run_fail_names[] = {
            [ESP_FOC_SD_FAIL_NONE] = "unknown",
            [ESP_FOC_SD_FAIL_ENABLE] = "the bridge refused to enable",
            [ESP_FOC_SD_FAIL_NOT_STILL] = "the shaft did not come to rest",
            [ESP_FOC_SD_FAIL_DESIGN] = "the speed loop design refused the motor",
        };
        const unsigned reason = (unsigned)run_fail_reason;
        printf("  Controller did not start: %s\n",
               (reason < 4u) ? run_fail_names[reason] : esp_err_to_name(err));
        shut_down();
        return;
    }
    print_controller_table();

    /* The angle the shaft rests at now is 0 deg for every command. */
    esp_foc_sensored_set_origin(MOTOR_AXIS);
    if (learn_cogging() != ESP_OK) {
        shut_down();
        if (controller_tripped) {
            print_trip_reason();
        }
        return;
    }

    /* The sweeps end wherever the last leg stopped; start the session at 0. */
    printf("\n");
    if (!move_to(0.0f)) {
        shut_down();
        print_trip_reason();
        return;
    }

    console_input_init();
    print_commands();

    static char command_line[COMMAND_LINE_MAX];
    while (read_command_line(command_line, sizeof(command_line))) {
        if (!handle_command(command_line)) {
            break;
        }
    }

    shut_down();
    if (controller_tripped) {
        print_trip_reason();
    } else {
        printf("Done: servo stopped, bridge off.\n");
    }
}
