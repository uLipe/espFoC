/*
 * espFoC example: sensorless torque control
 *
 * A PMSM/BLDC motor on a three-phase bridge with inline current shunts and
 * no rotor sensor: a flux observer estimates the rotor angle from the phase
 * currents and voltages. The example commissions the motor from its pole
 * pairs, rated speed and supply voltage, then runs a torque cycle in one
 * direction, forever or for a set number of cycles:
 *
 *   starting      the minimum torque current is asked for; the controller
 *                 aligns the rotor, spins it up open loop until the observer
 *                 locks, then hands control over to the observer angle
 *   accelerating  q-axis current ramps from the minimum to the cruise value
 *   cruising      the cruise current is held
 *   decelerating  the current ramps back to the minimum
 *   slow hold     the minimum current is held
 *   stopped       the current request goes to 0, the controller turns the
 *                 bridge off and the shaft coasts
 *
 * The observer only sees the rotor while it spins fast enough to make back
 * EMF, which is why a sensorless motor never runs at zero current or zero
 * speed: below the minimum the controller stops it, and every cycle starts
 * again from rest. In torque mode the controller regulates current, not
 * speed: with a free shaft the motor keeps speeding up while a current is
 * held.
 *
 * Commissioning, every boot, before the controller runs:
 *
 *   1. Phase discovery finds which bridge output drives which motor phase,
 *      so the motor leads can be connected in any order.
 *   2. Motor identification measures resistance, inductance and flux
 *      linkage. The current loop and the observer are designed from them.
 *
 * Both steps move the shaft: keep it free and keep hands off. Pins, motor and
 * cycle settings are in menuconfig under "espFoC example: sensorless torque".
 */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "sdkconfig.h"
#include "espFoC/drivers/esp_foc_inverter_mcpwm.h"
#include "espFoC/motor_control/esp_foc_motor_id.h"
#include "espFoC/motor_control/esp_foc_phase_discover.h"
#include "espFoC/motor_control/esp_foc_sensorless.h"
#include "espFoC/osal/esp_foc_osal.h"

#define MOTOR_AXIS 0u

#define PWM_RATE_HZ ((uint32_t)CONFIG_ESP_FOC_PWM_RATE_HZ)

#define POLE_PAIRS CONFIG_EXAMPLE_MOTOR_POLE_PAIRS
#define DC_LINK_VOLTS ((float)CONFIG_EXAMPLE_DC_LINK_MILLIVOLT * 1.0e-3f)
/* The controller works in electrical Hz: shaft rev/s times pole pairs. */
#define RATED_ELECTRICAL_HZ ((float)CONFIG_EXAMPLE_RATED_SPEED_RPM / 60.0f * (float)POLE_PAIRS)
#define MINIMUM_TORQUE_CURRENT_A ((float)CONFIG_EXAMPLE_MINIMUM_TORQUE_CURRENT_MA * 1.0e-3f)
#define TARGET_TORQUE_CURRENT_A ((float)CONFIG_EXAMPLE_TARGET_TORQUE_CURRENT_MA * 1.0e-3f)

#ifdef CONFIG_EXAMPLE_BRIDGE_ENABLE_ACTIVE_LOW
#define BRIDGE_ENABLE_ACTIVE_LOW true
#else
#define BRIDGE_ENABLE_ACTIVE_LOW false
#endif

/* Current setpoint updates during a ramp; 100 per second is smooth enough. */
#define RAMP_STEP_MS 10u

/* Align, open-loop spin-up, observer lock and handoff. The controller also
 * waits out its coast time first when the shaft was just stopped. */
#define START_TIMEOUT_MS 20000u
/* From the zero request to the bridge off. */
#define STOP_TIMEOUT_MS 3000u

static esp_foc_inverter_t *inverter;

/*
 * Written by the controller's event callback, which runs on the controller's
 * own task: the main loop only polls them between setpoint updates.
 */
static volatile bool controller_running;
static volatile bool controller_tripped;
static volatile esp_foc_sensorless_fail_t trip_start_failure;
static volatile esp_foc_sensorless_abort_t trip_abort_reason;
static volatile esp_foc_fault_reason_t trip_fault_reason;

static float electrical_rads_to_rpm(float electrical_rads)
{
    return electrical_rads / (2.0f * (float)M_PI) / (float)POLE_PAIRS * 60.0f;
}

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
    printf("          |_|   sensorless torque example\n");
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
    printf("  | %-26s | %-21s |\n", "Rotor sensor", "none (flux observer)");
    printf("  +----------------------------+-----------------------+\n");
    snprintf(text, sizeof(text), "%u kHz", (unsigned)(PWM_RATE_HZ / 1000u));
    printf("  | %-26s | %-21s |\n", "PWM frequency", text);
    snprintf(text, sizeof(text), "%d ns", CONFIG_EXAMPLE_PWM_DEAD_TIME_NS);
    printf("  | %-26s | %-21s |\n", "Dead time", text);
    snprintf(text, sizeof(text), "%d mOhm x %d", CONFIG_EXAMPLE_SHUNT_MILLIOHM,
             CONFIG_EXAMPLE_CURRENT_AMP_GAIN);
    printf("  | %-26s | %-21s |\n", "Shunt x amplifier gain", text);
    snprintf(text, sizeof(text), "%d Hz", CONFIG_EXAMPLE_CURRENT_FILTER_HZ);
    printf("  | %-26s | %-21s |\n", "Current sense filter", text);
    snprintf(text, sizeof(text), "%.2f A", (double)(CONFIG_EXAMPLE_BRIDGE_TRIP_MA * 1.0e-3f));
    printf("  | %-26s | %-21s |\n", "Current trip", text);
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
        .i_filt_fc_hz = (float)CONFIG_EXAMPLE_CURRENT_FILTER_HZ,
        .gpio_fault = -1,
    };
    return esp_foc_inverter_mcpwm_init(inverter, &inverter_config);
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

    printf("  Phase discovery: finding the phase order...\n");
    esp_foc_phase_discover_default_config(&discovery_config);
    memset(phase_map, 0, sizeof(*phase_map));
    /* No rotor sensor: the phase order is found from the currents alone. */
    esp_err_t err = esp_foc_phase_discover_init(&discovery, inverter, NULL, &discovery_config);
    if (err == ESP_OK) {
        err = esp_foc_phase_discover_run(&discovery, phase_map);
    }
    esp_foc_phase_discover_cleanup(&discovery);
    if (err != ESP_OK) {
        printf("  Phase discovery failed: %s\n", esp_err_to_name(err));
        return err;
    }

    printf("\n  Phase map\n");
    printf("  +-------------+---------------+--------------+\n");
    printf("  | Motor phase | Bridge output | Current sign |\n");
    printf("  +-------------+---------------+--------------+\n");
    for (unsigned phase = 0; phase < 3u; phase++) {
        printf("  | %-11s | %-13s | %+12d |\n", phase_letter(phase),
               phase_letter(phase_map->map.pwm_to_hw[phase]), phase_map->map.i_sign[phase]);
    }
    printf("  +-------------+---------------+--------------+\n");
    printf("  Attempts %u\n\n", (unsigned)phase_map->attempts);
    return ESP_OK;
}

static const char *identified_or_not(uint32_t valid_mask, uint32_t bit)
{
    return ((valid_mask & bit) != 0u) ? "identified" : "not fitted";
}

static void print_motor_table(const esp_foc_motor_id_result_t *plant,
                              const esp_foc_sensorless_config_t *controller_config)
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
    snprintf(text, sizeof(text), "%d rpm", CONFIG_EXAMPLE_RATED_SPEED_RPM);
    printf("  | %-26s | %-15s | %-10s |\n", "Rated speed", text, "Kconfig");
    snprintf(text, sizeof(text), "%.3f ohm", (double)controller_config->rs_ohm);
    printf("  | %-26s | %-15s | %-10s |\n", "Phase resistance", text,
           identified_or_not(valid, ESP_FOC_MOTOR_ID_VALID_R_LOOP | ESP_FOC_MOTOR_ID_VALID_RS));
    snprintf(text, sizeof(text), "%.0f uH", (double)(controller_config->ls_h * 1.0e6f));
    printf("  | %-26s | %-15s | %-10s |\n", "Phase inductance", text,
           identified_or_not(valid, ESP_FOC_MOTOR_ID_VALID_LS));
    snprintf(text, sizeof(text), "%.0f uWb", (double)(controller_config->psi_wb * 1.0e6f));
    printf("  | %-26s | %-15s | %-10s |\n", "Flux linkage", text,
           identified_or_not(valid, ESP_FOC_MOTOR_ID_VALID_PSI_F));
    printf("  +----------------------------+-----------------+------------+\n");
}

static esp_err_t identify_motor(esp_foc_motor_id_result_t *plant)
{
    esp_foc_motor_id_config_t id_config;

    printf("  Motor identification: the shaft will spin...\n");
    esp_foc_motor_id_default_config(&id_config);
    id_config.pole_pairs = POLE_PAIRS;
    id_config.vdc = DC_LINK_VOLTS;
    /* No rotor sensor: identification runs on the currents and voltages. */
    esp_err_t err = esp_foc_motor_id_run(inverter, NULL, &id_config, plant);
    if (err != ESP_OK) {
        printf("  Motor identification failed at '%s': %s\n",
               esp_foc_motor_id_phase_name(plant->failed_at), esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

/*
 * The start-up runs inside the controller; its events are the only window
 * into it, so they are printed as they happen. This callback runs on the
 * controller's task, never in an interrupt.
 */
static void on_controller_event(void *context, const esp_foc_sensorless_event_t *event)
{
    (void)context;
    switch (event->ev) {
    case ESP_FOC_SL_EV_STARTUP:
        printf("               aligning the rotor, spinning it up open loop\n");
        break;
    case ESP_FOC_SL_EV_LOCKED:
        printf("               observer locked at %.0f rpm\n",
               (double)electrical_rads_to_rpm(event->we_rads));
        break;
    case ESP_FOC_SL_EV_HANDOFF:
        printf("               handoff at %.0f rpm: control on the observer angle\n",
               (double)electrical_rads_to_rpm(event->we_rads));
        break;
    case ESP_FOC_SL_EV_RUNNING:
        printf("               start-up done, running\n");
        controller_running = true;
        break;
    case ESP_FOC_SL_EV_CUT:
        printf("               bridge off, coasting\n");
        controller_running = false;
        break;
    case ESP_FOC_SL_EV_STARTUP_FAILED:
        trip_start_failure = event->fail;
        controller_tripped = true;
        break;
    case ESP_FOC_SL_EV_ABORT:
        trip_abort_reason = event->abort;
        controller_tripped = true;
        break;
    case ESP_FOC_SL_EV_FAULT:
        trip_fault_reason = event->fault;
        controller_tripped = true;
        break;
    default:
        break;
    }
}

static void print_trip_reason(void)
{
    static const char *const start_failure_names[] = {
        [ESP_FOC_SL_FAIL_NONE] = "none",
        [ESP_FOC_SL_FAIL_ENABLE] = "the bridge refused to enable",
        [ESP_FOC_SL_FAIL_SEQUENCE] = "start-up settings refused",
        [ESP_FOC_SL_FAIL_NO_CURRENT] = "no current on the start-up ramp",
        [ESP_FOC_SL_FAIL_NO_FOLLOW] = "the shaft did not follow the start-up",
        [ESP_FOC_SL_FAIL_PLL_TIMEOUT] = "the observer never locked",
        [ESP_FOC_SL_FAIL_WE_LOW] = "too slow at the handoff",
        [ESP_FOC_SL_FAIL_WE_DROP] = "slowed down after the handoff",
    };
    static const char *const abort_names[] = {
        [ESP_FOC_SL_ABORT_NONE] = "none",
        [ESP_FOC_SL_ABORT_LOCK_LOSS] = "observer lost the rotor",
        [ESP_FOC_SL_ABORT_OVERSPEED] = "overspeed",
        [ESP_FOC_SL_ABORT_BEMF] = "back EMF too low",
        [ESP_FOC_SL_ABORT_COLLAPSE] = "current collapsed",
    };
    static const char *const fault_names[] = {
        [ESP_FOC_FAULT_NONE] = "none",
        [ESP_FOC_FAULT_ILIMIT] = "current trip",
        [ESP_FOC_FAULT_GPIO] = "fault pin",
        [ESP_FOC_FAULT_SOFT_TRIP] = "software trip",
        [ESP_FOC_FAULT_SENSE_STALE] = "current sense stopped",
    };
    const unsigned start_failure = (unsigned)trip_start_failure;
    const unsigned abort_reason = (unsigned)trip_abort_reason;
    const unsigned fault_reason = (unsigned)trip_fault_reason;
    printf("  Controller tripped: start-up '%s', abort '%s', fault '%s'. Bridge is off.\n",
           (start_failure < 8u) ? start_failure_names[start_failure] : "?",
           (abort_reason < 5u) ? abort_names[abort_reason] : "?",
           (fault_reason < 5u) ? fault_names[fault_reason] : "?");
}

static void build_controller_config(esp_foc_sensorless_config_t *controller_config,
                                    const esp_foc_phase_discover_result_t *phase_map,
                                    const esp_foc_motor_id_result_t *plant)
{
    esp_foc_sensorless_default_config(controller_config);
    controller_config->axis = MOTOR_AXIS;
    controller_config->pole_pairs = POLE_PAIRS;
    controller_config->fe_rated_hz = RATED_ELECTRICAL_HZ;
    /* R, L and flux linkage from the identification. */
    esp_foc_sensorless_config_from_motor_id(controller_config, plant);
    /* Which bridge output drives which phase, and the current signs. */
    esp_foc_sensorless_config_from_phase_map(controller_config, phase_map);
    /* Torque only: the current request is the command, no speed loop. */
    controller_config->speed_loop = false;
    controller_config->on_event = on_controller_event;
}

static void print_controller_table(void)
{
    esp_foc_sensorless_tuning_t tuning;
    char text[24];

    esp_foc_sensorless_get_tuning(MOTOR_AXIS, &tuning);
    printf("\n  Controller (designed from the identified motor)\n");
    printf("  +----------------------------+-----------------+\n");
    snprintf(text, sizeof(text), "%.4f", (double)tuning.kp_i);
    printf("  | %-26s | %-15s |\n", "Current loop Kp", text);
    snprintf(text, sizeof(text), "%.1f", (double)tuning.ki_i);
    printf("  | %-26s | %-15s |\n", "Current loop Ki", text);
    snprintf(text, sizeof(text), "%.1f Hz", (double)tuning.track_bw_hz);
    printf("  | %-26s | %-15s |\n", "Observer tracking band", text);
    snprintf(text, sizeof(text), "%.2f A", (double)MINIMUM_TORQUE_CURRENT_A);
    printf("  | %-26s | %-15s |\n", "Start / slow-hold current", text);
    snprintf(text, sizeof(text), "%.2f A", (double)TARGET_TORQUE_CURRENT_A);
    printf("  | %-26s | %-15s |\n", "Cruise torque current", text);
    printf("  +----------------------------+-----------------+\n\n");
}

static float shaft_speed_rpm(void)
{
    esp_foc_sensorless_status_t status;
    esp_foc_sensorless_get_status(MOTOR_AXIS, &status);
    return electrical_rads_to_rpm(status.we_rads);
}

static void announce(unsigned cycle, const char *phase)
{
    printf("  cycle %-4u %-13s %6.0f rpm\n", cycle, phase, (double)shaft_speed_rpm());
}

/* Sleeps in short steps so a trip is noticed within RAMP_STEP_MS. */
static bool hold_for(uint32_t duration_ms)
{
    for (uint32_t elapsed_ms = 0; elapsed_ms < duration_ms; elapsed_ms += RAMP_STEP_MS) {
        if (controller_tripped) {
            return false;
        }
        esp_foc_sleep_ms(RAMP_STEP_MS);
    }
    return !controller_tripped;
}

static bool ramp_torque_current(float from_a, float to_a, uint32_t duration_ms)
{
    const uint32_t steps = duration_ms / RAMP_STEP_MS;
    for (uint32_t step = 1; step <= steps; step++) {
        if (controller_tripped) {
            return false;
        }
        const float current_a = from_a + (to_a - from_a) * (float)step / (float)steps;
        esp_foc_sensorless_set_iq(MOTOR_AXIS, current_a);
        esp_foc_sleep_ms(RAMP_STEP_MS);
    }
    return !controller_tripped;
}

/* The current request alone starts the motor; RUNNING says it is on the
 * observer. */
static bool start_motor(unsigned cycle)
{
    printf("  cycle %-4u starting      %.2f A\n", cycle, (double)MINIMUM_TORQUE_CURRENT_A);
    esp_foc_sensorless_set_iq(MOTOR_AXIS, MINIMUM_TORQUE_CURRENT_A);
    for (uint32_t waited_ms = 0; !controller_running; waited_ms += RAMP_STEP_MS) {
        if (controller_tripped) {
            return false;
        }
        if (waited_ms >= START_TIMEOUT_MS) {
            printf("  The motor did not reach the observer in %u ms\n",
                   (unsigned)START_TIMEOUT_MS);
            return false;
        }
        esp_foc_sleep_ms(RAMP_STEP_MS);
    }
    return true;
}

/* A request under the controller's minimum is a stop: bridge off, coast,
 * back to armed. */
static bool stop_motor(void)
{
    esp_foc_sensorless_set_iq(MOTOR_AXIS, 0.0f);
    for (uint32_t waited_ms = 0; esp_foc_sensorless_get_state(MOTOR_AXIS) != ESP_FOC_SL_STATE_ARMED;
         waited_ms += RAMP_STEP_MS) {
        if (controller_tripped) {
            return false;
        }
        if (waited_ms >= STOP_TIMEOUT_MS) {
            printf("  The controller did not turn the bridge off in %u ms\n",
                   (unsigned)STOP_TIMEOUT_MS);
            return false;
        }
        esp_foc_sleep_ms(RAMP_STEP_MS);
    }
    return true;
}

static bool run_one_cycle(unsigned cycle)
{
    if (!start_motor(cycle)) {
        return false;
    }
    announce(cycle, "accelerating");
    if (!ramp_torque_current(MINIMUM_TORQUE_CURRENT_A, TARGET_TORQUE_CURRENT_A,
                             CONFIG_EXAMPLE_ACCELERATION_MS)) {
        return false;
    }
    announce(cycle, "cruising");
    if (!hold_for(CONFIG_EXAMPLE_CRUISE_MS)) {
        return false;
    }
    announce(cycle, "decelerating");
    if (!ramp_torque_current(TARGET_TORQUE_CURRENT_A, MINIMUM_TORQUE_CURRENT_A,
                             CONFIG_EXAMPLE_DECELERATION_MS)) {
        return false;
    }
    announce(cycle, "slow hold");
    if (!hold_for(CONFIG_EXAMPLE_MINIMUM_HOLD_MS)) {
        return false;
    }
    announce(cycle, "stopped");
    if (!stop_motor()) {
        return false;
    }
    return hold_for(CONFIG_EXAMPLE_STOPPED_MS);
}

static void shut_down(void)
{
    esp_foc_sensorless_stop(MOTOR_AXIS);
    esp_foc_sensorless_deinit(MOTOR_AXIS);
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

    esp_foc_phase_discover_result_t phase_map;
    if (discover_phases(&phase_map) != ESP_OK) {
        return;
    }

    esp_foc_motor_id_result_t plant;
    memset(&plant, 0, sizeof(plant));
    if (identify_motor(&plant) != ESP_OK) {
        return;
    }

    esp_foc_sensorless_config_t controller_config;
    build_controller_config(&controller_config, &phase_map, &plant);
    print_motor_table(&plant, &controller_config);
    /* The controller refuses current requests above its ceiling
     * (CONFIG_ESP_FOC_SL_I_MAX_MA in the espFoC menu). */
    if (TARGET_TORQUE_CURRENT_A > controller_config.i_max_a) {
        printf("  Cruise current %.2f A is above the controller ceiling %.2f A\n",
               (double)TARGET_TORQUE_CURRENT_A, (double)controller_config.i_max_a);
        return;
    }

    err = esp_foc_sensorless_init(inverter, &controller_config);
    if (err != ESP_OK) {
        printf("  Controller init failed: %s\n", esp_err_to_name(err));
        return;
    }
    /* Arms the controller; the bridge stays off until a current is asked for. */
    err = esp_foc_sensorless_run(MOTOR_AXIS);
    if (err != ESP_OK) {
        printf("  Controller did not arm: %s\n", esp_err_to_name(err));
        shut_down();
        return;
    }
    print_controller_table();

    const unsigned cycles_to_run = CONFIG_EXAMPLE_CYCLE_COUNT;
    for (unsigned cycle = 1; (cycles_to_run == 0u) || (cycle <= cycles_to_run); cycle++) {
        if (!run_one_cycle(cycle)) {
            shut_down();
            if (controller_tripped) {
                print_trip_reason();
            } else {
                printf("  Stopped, bridge off.\n");
            }
            return;
        }
    }

    shut_down();
    printf("  Done: %u cycles, bridge off.\n", cycles_to_run);
}
