/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * Console REPL on ESP_CONSOLE UART — see doc/SHELL.md
 */

#include <sdkconfig.h>

#if defined(CONFIG_ESPFOC_SHELL)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "espFoC/esp_foc.h"
#include "espFoC/osal/os_interface.h"
#include "espFoC/esp_foc_estimator_q16.h"
#include "espFoC/calibration/esp_foc_calibration.h"
#include "espFoC/shell/espfoc_shell.h"
#include "espFoC/shell/espfoc_shell_parse.h"
#include "espFoC/utils/esp_foc_q16.h"

#define ESPFOC_SHELL_MAX_AXES    4
#define ESPFOC_SHELL_LINE_MAX    256
#define ESPFOC_SHELL_MAX_TOKENS  8
#define ESPFOC_SHELL_TASK_STACK  4096
#define ESPFOC_SHELL_TASK_PRIO   12

#define streq_ci espfoc_shell_streq_ci

static esp_foc_axis_t *s_axes[ESPFOC_SHELL_MAX_AXES] = { NULL };

static void shell_write(const char *s)
{
    if (s == NULL) {
        return;
    }
    size_t n = strlen(s);
    if (n > 0U) {
        (void)esp_foc_console_write(s, n);
    }
}

static void shell_printf(const char *fmt, ...)
{
    char buf[384];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) {
        size_t len = (size_t)n;
        if (len >= sizeof(buf)) {
            len = sizeof(buf) - 1U;
        }
        (void)esp_foc_console_write(buf, len);
    }
}

static void shell_println(const char *line)
{
    if (line != NULL) {
        shell_write(line);
    }
    shell_write("\n");
}

static int shell_read_line(char *line, size_t cap)
{
    size_t pos = 0;

    if (cap < 2) {
        return -1;
    }

    for (;;) {
        int c = esp_foc_console_read_byte(-1);
        if (c < 0) {
            continue;
        }
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            line[pos] = '\0';
            shell_write("\n");
            return (int)pos;
        }
        if (c == 0x7f || c == 0x08) {
            if (pos > 0) {
                pos--;
                shell_write("\b \b");
            }
            continue;
        }
        if (c < 0x20 || c > 0x7e) {
            continue;
        }
        if (pos + 1 >= cap) {
            continue;
        }
        line[pos++] = (char)c;
        char echo[2] = { (char)c, '\0' };
        shell_write(echo);
    }
}

typedef struct {
    const char *name;
    const char *aliases[3];
} shell_field_t;

static const shell_field_t s_config_fields[] = {
    { "kp", { NULL } },
    { "ki", { NULL } },
    { "kd", { NULL } },
    { "kff", { NULL } },
    { "integrator_limit", { "ilim", NULL } },
    { "current_filter_fc_hz", { "lpf", "i_lpf", NULL } },
    { "pole_pairs", { "pp", NULL } },
};

esp_foc_err_t espfoc_shell_register_axis(uint8_t axis_id, esp_foc_axis_t *axis)
{
    if (axis == NULL || axis_id >= ESPFOC_SHELL_MAX_AXES) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    s_axes[axis_id] = axis;
    return ESP_FOC_OK;
}

static esp_foc_axis_t *shell_axis(uint8_t axis_id)
{
    if (axis_id >= ESPFOC_SHELL_MAX_AXES) {
        return NULL;
    }
    return s_axes[axis_id];
}

static bool shell_token_is_axis_id(const char *tok, long *axis_id_out)
{
    return espfoc_shell_parse_axis_id(tok, ESPFOC_SHELL_MAX_AXES, axis_id_out);
}

static esp_foc_axis_t *shell_require_axis(long axis_id)
{
    esp_foc_axis_t *axis = shell_axis((uint8_t)axis_id);
    if (axis == NULL) {
        shell_printf("axis %ld not registered\n", axis_id);
    }
    return axis;
}

static const char *err_str(esp_foc_err_t err)
{
    switch (err) {
    case ESP_FOC_OK:
        return "ok";
    case ESP_FOC_ERR_NOT_ALIGNED:
        return "not_aligned";
    case ESP_FOC_ERR_INVALID_ARG:
        return "invalid_arg";
    case ESP_FOC_ERR_AXIS_INVALID_STATE:
        return "invalid_state";
    case ESP_FOC_ERR_ALIGNMENT_IN_PROGRESS:
        return "aligning";
    case ESP_FOC_ERR_TIMESTEP_TOO_SMALL:
        return "timestep_too_small";
    case ESP_FOC_ERR_ROTOR_STARTUP:
        return "rotor_startup";
    case ESP_FOC_ERR_ROTOR_STARTUP_PI:
        return "rotor_startup_pi";
    case ESP_FOC_ERR_NOT_SUPPORTED:
        return "not_supported";
    default:
        return "error";
    }
}

static const char *state_str(esp_foc_axis_state_t st)
{
    switch (st) {
    case ESP_FOC_AXIS_STATE_IDLE:
        return "idle";
    case ESP_FOC_AXIS_STATE_ALIGNING:
        return "aligning";
    case ESP_FOC_AXIS_STATE_ALIGNED:
        return "aligned";
    case ESP_FOC_AXIS_STATE_RUNNING:
        return "running";
    case ESP_FOC_AXIS_STATE_BENCH:
        return "bench";
    default:
        return "?";
    }
}

static void cmd_list_registered_axes(void)
{
    int n = 0;
    shell_printf("registered axes:\n");
    for (int i = 0; i < ESPFOC_SHELL_MAX_AXES; i++) {
        if (s_axes[i] == NULL) {
            continue;
        }
        shell_printf("  %d  state=%s\n", i, state_str(s_axes[i]->state));
        n++;
    }
    if (n == 0) {
        shell_printf("  (none)\n");
    }
}

static const char *strip_config_prefix(const char *key)
{
    if (key != NULL && strncmp(key, "config.", 7) == 0) {
        return key + 7;
    }
    return key;
}

static const shell_field_t *find_config_field(const char *key)
{
    const char *k = strip_config_prefix(key);
    if (k == NULL) {
        return NULL;
    }
    for (size_t i = 0; i < sizeof(s_config_fields) / sizeof(s_config_fields[0]); i++) {
        const shell_field_t *f = &s_config_fields[i];
        if (streq_ci(k, f->name)) {
            return f;
        }
        for (int a = 0; a < 3 && f->aliases[a] != NULL; a++) {
            if (streq_ci(k, f->aliases[a])) {
                return f;
            }
        }
    }
    return NULL;
}

static bool field_get_float(esp_foc_axis_t *axis, const shell_field_t *f, float *out)
{
    q16_t kp, ki, kd, kff, ilim;
    esp_foc_axis_get_current_loop_gains_q16(axis, &kp, &ki, &kd, &kff, &ilim);

    if (streq_ci(f->name, "kp")) {
        *out = q16_to_float(kp);
        return true;
    }
    if (streq_ci(f->name, "ki")) {
        *out = q16_to_float(ki);
        return true;
    }
    if (streq_ci(f->name, "kd")) {
        *out = q16_to_float(kd);
        return true;
    }
    if (streq_ci(f->name, "kff")) {
        *out = q16_to_float(kff);
        return true;
    }
    if (streq_ci(f->name, "integrator_limit")) {
        *out = q16_to_float(ilim);
        return true;
    }
    if (streq_ci(f->name, "current_filter_fc_hz")) {
        *out = q16_to_float(axis->current_filter_fc_hz_q16);
        return true;
    }
    if (streq_ci(f->name, "pole_pairs")) {
        *out = (float)axis->motor_pole_pairs;
        return true;
    }
    return false;
}

static esp_foc_err_t field_set_float(esp_foc_axis_t *axis, const shell_field_t *f,
                                     float val)
{
    q16_t kp, ki, kd, kff, ilim;
    esp_foc_axis_get_current_loop_gains_q16(axis, &kp, &ki, &kd, &kff, &ilim);

    if (streq_ci(f->name, "kp")) {
        return esp_foc_axis_set_current_loop_gains_q16(
            axis, q16_from_float(val), ki, kd, kff, ilim);
    }
    if (streq_ci(f->name, "ki")) {
        return esp_foc_axis_set_current_loop_gains_q16(
            axis, kp, q16_from_float(val), kd, kff, ilim);
    }
    if (streq_ci(f->name, "kd")) {
        return esp_foc_axis_set_current_loop_gains_q16(
            axis, kp, ki, q16_from_float(val), kff, ilim);
    }
    if (streq_ci(f->name, "kff")) {
        return esp_foc_axis_set_current_loop_gains_q16(
            axis, kp, ki, kd, q16_from_float(val), ilim);
    }
    if (streq_ci(f->name, "integrator_limit")) {
        return esp_foc_axis_set_current_loop_gains_q16(
            axis, kp, ki, kd, kff, q16_from_float(val));
    }
    if (streq_ci(f->name, "current_filter_fc_hz")) {
        if (val < 0.0f) {
            return ESP_FOC_ERR_INVALID_ARG;
        }
        axis->current_filter_fc_hz_q16 = q16_from_float(val);
        if (axis->inverter_driver != NULL &&
            axis->inverter_driver->set_filter_cutoff != NULL) {
            float fs = q16_to_float(axis->current_filter_fs_hz_q16);
            axis->inverter_driver->set_filter_cutoff(
                axis->inverter_driver, val, fs);
        }
        return ESP_FOC_OK;
    }
    if (streq_ci(f->name, "pole_pairs")) {
        if (val < 1.0f || val > 64.0f) {
            return ESP_FOC_ERR_INVALID_ARG;
        }
        axis->motor_pole_pairs = (int)val;
        esp_foc_axis_refresh_encoder_q16_scales(axis);
        if (axis->rotor_estimator.pole_pairs != (int)val) {
            esp_foc_estimator_q16_set_pole_pairs(&axis->rotor_estimator, (int)val);
        }
        return ESP_FOC_OK;
    }
    return ESP_FOC_ERR_INVALID_ARG;
}

static void cmd_help(const char *topic)
{
    if (topic == NULL || streq_ci(topic, "all")) {
        shell_println("Grammar: <verb> [subcommand] [args...] <axis_id>");
        shell_println("         axis id is always last (except list axis, erase, reboot)");
        shell_println("        help [config|axis]");
        shell_println("Verbs:");
        shell_println("  list axis [id]       — no id: all registered axes");
        shell_println("  list config <id>");
        shell_println("  get config.<field> <id>");
        shell_println("  set config.<field> <val> <id> | set id|iq <A> <id>");
        shell_println("  align <id> | run <id> | stop <id> | store <id>");
        shell_println("  erase | reboot");
        shell_println("Examples:");
        shell_println("  list axis");
        shell_println("  list config 0");
        shell_println("  align 0");
        shell_println("  set iq 1.0 0");
        shell_println("  run 0");
        return;
    }
    if (streq_ci(topic, "config")) {
        shell_printf("config.* — current-loop gains, LPF Hz, pole pairs\n");
        return;
    }
    if (streq_ci(topic, "axis")) {
        shell_printf("list axis — all ids; list axis <id> — state detail\n");
        return;
    }
    shell_printf("unknown help topic (try: help config)\n");
}

static void cmd_list_config(esp_foc_axis_t *axis, long axis_id)
{
    float v;
    shell_printf("axis %ld config (live):\n", axis_id);
    for (size_t i = 0; i < sizeof(s_config_fields) / sizeof(s_config_fields[0]); i++) {
        if (field_get_float(axis, &s_config_fields[i], &v)) {
            shell_printf("  config.%s = %.6g\n", s_config_fields[i].name, (double)v);
        }
    }
#if defined(CONFIG_ESP_FOC_CALIBRATION_NVS)
    shell_printf("  nvs.calibration = %s\n",
           esp_foc_calibration_present(axis->cal.axis_id) ? "yes" : "no");
#else
    shell_printf("  nvs.calibration = (disabled)\n");
#endif
}

static void cmd_list_axis(esp_foc_axis_t *axis, long axis_id)
{
    shell_printf("axis %ld:\n", axis_id);
    shell_printf("  state = %s\n", state_str(axis->state));
    shell_printf("  mode = %d\n", (int)axis->mode);
    shell_printf("  target_id_A = %.4f\n", (double)q16_to_float(axis->target_i_d.raw));
    shell_printf("  target_iq_A = %.4f\n", (double)q16_to_float(axis->target_i_q.raw));
    shell_printf("  id_A = %.4f\n", (double)q16_to_float(axis->i_d.raw));
    shell_printf("  iq_A = %.4f\n", (double)q16_to_float(axis->i_q.raw));
    shell_printf("  theta_meas_turn = %.5f\n",
           (double)q16_to_float(axis->rotor_estimator.theta_meas_mech));
    shell_printf("  theta_est_turn = %.5f\n",
           (double)q16_to_float(axis->rotor_estimator.theta_est_mech));
    shell_printf("  omega_est_turn_s = %.5f\n",
           (double)q16_to_float(axis->rotor_estimator.omega_est_mech));
    shell_printf("  pole_pairs = %d\n", axis->motor_pole_pairs);
}

static void cmd_get(esp_foc_axis_t *axis, const char *key)
{
    const shell_field_t *f = find_config_field(key);
    if (f == NULL) {
        shell_printf("unknown key (try: get config.kp 0)\n");
        return;
    }
    float v;
    if (!field_get_float(axis, f, &v)) {
        shell_printf("get failed\n");
        return;
    }
    shell_printf("config.%s = %.6g\n", f->name, (double)v);
}

static void cmd_set(esp_foc_axis_t *axis, const char *key, const char *val_s)
{
    if (key == NULL || val_s == NULL) {
        shell_printf("usage: set <key> <value> <axis_id>\n");
        return;
    }

    if (streq_ci(key, "id")) {
        axis->target_i_d.raw = q16_from_float((float)atof(val_s));
        shell_printf("ok\n");
        return;
    }
    if (streq_ci(key, "iq")) {
        axis->target_i_q.raw = q16_from_float((float)atof(val_s));
        shell_printf("ok\n");
        return;
    }

    const shell_field_t *f = find_config_field(key);
    if (f == NULL) {
        shell_printf("unknown key\n");
        return;
    }
    esp_foc_err_t err = field_set_float(axis, f, (float)atof(val_s));
    shell_printf("%s\n", err_str(err));
}

static void shell_exec_tokens(int argc, char **argv)
{
    long axis_id;

    if (argc == 0) {
        return;
    }

    if (streq_ci(argv[0], "help")) {
        cmd_help(argc > 1 ? argv[1] : NULL);
        return;
    }

    if (argc >= 1 && shell_token_is_axis_id(argv[0], &axis_id)) {
        shell_printf("axis id goes last (e.g. align 0, list axis 0)\n");
        return;
    }

    if (streq_ci(argv[0], "list")) {
        if (argc < 2) {
            shell_printf("usage: list axis [id] | list config <id>\n");
            return;
        }
        if (streq_ci(argv[1], "axis")) {
            if (argc == 2) {
                cmd_list_registered_axes();
                return;
            }
            if (argc == 3 && shell_token_is_axis_id(argv[2], &axis_id)) {
                esp_foc_axis_t *axis = shell_require_axis(axis_id);
                if (axis != NULL) {
                    cmd_list_axis(axis, axis_id);
                }
                return;
            }
            shell_printf("usage: list axis | list axis <id>\n");
            return;
        }
        if (streq_ci(argv[1], "config")) {
            if (argc == 3 && shell_token_is_axis_id(argv[2], &axis_id)) {
                esp_foc_axis_t *axis = shell_require_axis(axis_id);
                if (axis != NULL) {
                    cmd_list_config(axis, axis_id);
                }
                return;
            }
            shell_printf("usage: list config <axis_id>\n");
            return;
        }
        shell_printf("usage: list axis [id] | list config <id>\n");
        return;
    }

    if (streq_ci(argv[0], "get")) {
        if (argc == 3 && shell_token_is_axis_id(argv[2], &axis_id)) {
            esp_foc_axis_t *axis = shell_require_axis(axis_id);
            if (axis != NULL) {
                cmd_get(axis, argv[1]);
            }
            return;
        }
        shell_printf("usage: get config.<field> <axis_id>\n");
        return;
    }

    if (streq_ci(argv[0], "set")) {
        if (argc == 4 && shell_token_is_axis_id(argv[3], &axis_id)) {
            esp_foc_axis_t *axis = shell_require_axis(axis_id);
            if (axis != NULL) {
                cmd_set(axis, argv[1], argv[2]);
            }
            return;
        }
        shell_printf("usage: set <key> <value> <axis_id>\n");
        return;
    }

    if (streq_ci(argv[0], "align")) {
        if (argc == 2 && shell_token_is_axis_id(argv[1], &axis_id)) {
            esp_foc_axis_t *axis = shell_require_axis(axis_id);
            if (axis != NULL) {
                shell_printf("%s\n", err_str(esp_foc_align_axis(axis)));
            }
            return;
        }
        shell_printf("usage: align <axis_id>\n");
        return;
    }

    if (streq_ci(argv[0], "run")) {
        if (argc == 2 && shell_token_is_axis_id(argv[1], &axis_id)) {
            esp_foc_axis_t *axis = shell_require_axis(axis_id);
            if (axis != NULL) {
                shell_printf("%s\n", err_str(esp_foc_run(axis)));
            }
            return;
        }
        shell_printf("usage: run <axis_id>\n");
        return;
    }

    if (streq_ci(argv[0], "stop")) {
        if (argc == 2 && shell_token_is_axis_id(argv[1], &axis_id)) {
            esp_foc_axis_t *axis = shell_require_axis(axis_id);
            if (axis != NULL) {
                shell_printf("%s\n", err_str(esp_foc_stop(axis)));
            }
            return;
        }
        shell_printf("usage: stop <axis_id>\n");
        return;
    }

    if (streq_ci(argv[0], "store")) {
        if (argc == 2 && shell_token_is_axis_id(argv[1], &axis_id)) {
            esp_foc_axis_t *axis = shell_require_axis(axis_id);
            if (axis != NULL) {
                shell_printf("%s\n",
                              err_str(esp_foc_calibration_axis_store_live(axis)));
            }
            return;
        }
        shell_printf("usage: store <axis_id>\n");
        return;
    }

    if (streq_ci(argv[0], "erase")) {
        shell_printf("%s\n", err_str(esp_foc_calibration_erase()));
        return;
    }

    if (streq_ci(argv[0], "reboot")) {
        shell_printf("rebooting...\n");
        esp_foc_reboot();
        return;
    }

    shell_printf("unknown verb (try: help)\n");
}

static void shell_reader_task(void *arg)
{
    (void)arg;
    char line[ESPFOC_SHELL_LINE_MAX];
    char *tok[ESPFOC_SHELL_MAX_TOKENS];

    shell_write("\nespfoc shell — type 'help'\n");

    for (;;) {
        shell_write("espFoC> ");
        if (shell_read_line(line, sizeof(line)) < 0) {
            continue;
        }
        espfoc_shell_trim_line(line);
        if (line[0] == '\0') {
            continue;
        }
        int n = espfoc_shell_split_tokens(line, tok, ESPFOC_SHELL_MAX_TOKENS);
        shell_exec_tokens(n, tok);
    }
}

void espfoc_shell_start(void)
{
    esp_foc_console_init();
    shell_write("try: list axis | list config 0 | align 0 | run 0\n");
    if (esp_foc_task_spawn(shell_reader_task, NULL, ESPFOC_SHELL_TASK_STACK,
                           ESPFOC_SHELL_TASK_PRIO, NULL) != 0) {
        shell_write("espfoc_shell: failed to start task\n");
    }
}

#endif /* CONFIG_ESPFOC_SHELL */
