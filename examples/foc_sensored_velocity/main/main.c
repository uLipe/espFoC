/*
 * espFoC example: sensored velocity control
 *
 * A PMSM/BLDC motor on a three-phase bridge with inline current shunts and an
 * AS5600 magnetic encoder on the shaft. The example commissions the motor
 * from nothing but its pole pairs and the supply voltage, then runs a speed
 * cycle in one direction, forever or for a set number of cycles:
 *
 *   accelerating  the speed reference ramps from 0 to the target
 *   cruising      the target speed is held
 *   decelerating  the speed reference ramps back to 0
 *   stopped       the speed loop holds the shaft at 0 rpm
 *
 * The speed loop drives the torque current; the controller limits that
 * current, so a load the motor cannot carry shows up as a speed error, not
 * as an over-current trip.
 *
 * Commissioning, every boot, before the controller runs:
 *
 *   1. Phase discovery finds which bridge output drives which motor phase,
 *      zeroes the encoder on the rotor's magnetic axis and checks which way
 *      it counts. Motor leads can be connected in any order.
 *   2. Motor identification measures resistance, inductance, flux linkage,
 *      how fast the shaft accelerates per amp, and the encoder's offset and
 *      delay. The controller gains are designed from these numbers.
 *
 * Both steps spin the shaft: keep it free and keep hands off. Identification
 * takes about 90 s. Pins, motor and cycle settings are in menuconfig under
 * "espFoC example: sensored velocity".
 *
 * When the speed loop starts, the controller waits for the shaft to be at
 * rest, measures how noisy the encoder speed is, and picks the speed loop
 * bandwidth from that noise and the identified acceleration per amp.
 */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

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
#define TARGET_SPEED_RPM ((float)CONFIG_EXAMPLE_TARGET_SPEED_RPM)
/* The controller takes speed as electrical Hz: shaft rev/s times pole pairs. */
#define TARGET_ELECTRICAL_HZ (TARGET_SPEED_RPM / 60.0f * (float)POLE_PAIRS)

#ifdef CONFIG_EXAMPLE_BRIDGE_ENABLE_ACTIVE_LOW
#define BRIDGE_ENABLE_ACTIVE_LOW true
#else
#define BRIDGE_ENABLE_ACTIVE_LOW false
#endif

/* How often the main loop looks for a controller trip while it waits. */
#define TRIP_POLL_MS 10u

static esp_foc_inverter_t *inverter;
static esp_foc_rotor_sensor_t *encoder;

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
    printf("          |_|   sensored velocity example\n");
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
    /* The speed loop is designed from K: without it there is no speed PI. */
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
    controller_config->control = ESP_FOC_SD_CONTROL_VELOCITY;
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
    snprintf(text, sizeof(text), "%.0f rpm", (double)TARGET_SPEED_RPM);
    printf("  | %-26s | %-15s |\n", "Cruise speed", text);
    printf("  +----------------------------+-----------------+\n\n");
}

static float shaft_speed_rpm(void)
{
    esp_foc_sensored_status_t status;
    esp_foc_sensored_get_status(MOTOR_AXIS, &status);
    /* The controller reports electrical rad/s; divide by pole pairs for the shaft. */
    return status.we_rads / (2.0f * (float)M_PI) / (float)POLE_PAIRS * 60.0f;
}

static void announce(unsigned cycle, const char *phase)
{
    printf("  cycle %-4u %-13s %6.0f rpm\n", cycle, phase, (double)shaft_speed_rpm());
}

/* Sleeps in short steps so a trip is noticed within TRIP_POLL_MS. */
static bool wait_for(uint32_t duration_ms)
{
    for (uint32_t elapsed_ms = 0; elapsed_ms < duration_ms; elapsed_ms += TRIP_POLL_MS) {
        if (controller_tripped) {
            return false;
        }
        esp_foc_sleep_ms(TRIP_POLL_MS);
    }
    return !controller_tripped;
}

/*
 * The controller slews its speed reference at a set rate, so a ramp is one
 * call: pick the rate that covers the change in the given time, set the end
 * value, then wait for the ramp to finish.
 */
static bool ramp_speed(float from_hz, float to_hz, uint32_t duration_ms)
{
    const float rate_hz_per_s = fabsf(to_hz - from_hz) * 1000.0f / (float)duration_ms;
    esp_foc_sensored_set_speed_slew(MOTOR_AXIS, rate_hz_per_s);
    esp_foc_sensored_set_speed_ref_hz(MOTOR_AXIS, to_hz);
    return wait_for(duration_ms);
}

static bool run_one_cycle(unsigned cycle)
{
    announce(cycle, "accelerating");
    if (!ramp_speed(0.0f, TARGET_ELECTRICAL_HZ, CONFIG_EXAMPLE_ACCELERATION_MS)) {
        return false;
    }
    announce(cycle, "cruising");
    if (!wait_for(CONFIG_EXAMPLE_CRUISE_MS)) {
        return false;
    }
    announce(cycle, "decelerating");
    if (!ramp_speed(TARGET_ELECTRICAL_HZ, 0.0f, CONFIG_EXAMPLE_DECELERATION_MS)) {
        return false;
    }
    announce(cycle, "stopped");
    return wait_for(CONFIG_EXAMPLE_STOPPED_MS);
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

    err = esp_foc_sensored_init(inverter, encoder, &controller_config);
    if (err != ESP_OK) {
        printf("  Controller init failed: %s\n", esp_err_to_name(err));
        return;
    }
    /* Arms the bridge, waits for a still shaft, designs and starts the speed loop. */
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

    const unsigned cycles_to_run = CONFIG_EXAMPLE_CYCLE_COUNT;
    for (unsigned cycle = 1; (cycles_to_run == 0u) || (cycle <= cycles_to_run); cycle++) {
        if (!run_one_cycle(cycle)) {
            shut_down();
            print_trip_reason();
            return;
        }
    }

    shut_down();
    printf("  Done: %u cycles, bridge off.\n", cycles_to_run);
}
