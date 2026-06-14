# espFoC

Field Oriented Control (FOC) library for PMSM / BLDC motors on the
ESP32 family, built on ESP-IDF.

![Build](https://github.com/uLipe/espFoC/workflows/Build/badge.svg)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

![espFoC running on hardware](doc/images/espfoc_demo.gif)

espFoC covers the inner control chain: inverter drive, current
sensing, rotor feedback and the Id/Iq torque loop. Velocity and
position loops live in the application's regulation callback; the
library stays focused and the hot path runs without floating-point
math. Gains can be synthesised at build time, retuned live from the
firmware API, persisted to NVS, or dialled in via the console shell.

Targets: ESP32, **ESP32-C6**, ESP32-S3, ESP32-P4 (ESP-IDF v5+, MCPWM required).

> **3.0 is a breaking release.** The legacy continuous-time PI
> formula and the `motor_resistance / motor_inductance / motor_inertia`
> fields are gone — gains come from build-time synthesis, NVS calibration,
> or the console shell. The 3-PWM LEDC driver was also dropped.
> `esp_foc_controls.h` tunables are Kconfig options (`CONFIG_ESP_FOC_LOW_SPEED_DOWNSAMPLING`,
> `CONFIG_ESP_FOC_ISENSOR_CALIBRATION_ROUNDS`). The motor regulation callback
> is three arguments only (`id_ref`, `iq_ref`). See [`changelog.txt`](changelog.txt) for
> the full migration list.

---

## Install

Via the IDF component registry:

```bash
idf.py add-dependency "ulipe/espfoc^3.0.0"
```

Or clone the repo and add it to your project:

```cmake
set(EXTRA_COMPONENT_DIRS "path/to/espFoC")
```

Then pick an example as a starting point:

```bash
cd examples/axis_shell
idf.py set-target esp32c6
idf.py build flash monitor
```

Also supported: `esp32s3`, `esp32p4`.

---

## Architecture

![espFoC architecture](doc/images/architecture.png)

Each axis owns one **unified inverter** (PWM + current sense) and one
**encoder**. You implement a **regulation callback** that updates
`id_ref` / `iq_ref` each outer-loop tick; espFoC runs Clarke/Park, the
current PIs, inverse Park, SVPWM and PWM updates. The inner current loop
and modulation run in the **PWM ISR** at carrier frequency; a dedicated
task reads the encoder (~2 kHz by default) and feeds an **encoder PLL**
(`esp_foc_estimator_q16`) that integrates mechanical angle at **PWM rate**
(20 kHz). Only the PWM ISR writes `rotor_elec_angle`; your regulation
callback runs on the slow task at roughly **PWM rate ÷
`CONFIG_ESP_FOC_LOW_SPEED_DOWNSAMPLING`** (see *espFoC Settings → Control
loop* in `menuconfig`). Platform primitives (tasks, critical sections,
timers) go through **`include/espFoC/osal/os_interface.h`** so motor code
does not depend on FreeRTOS headers.

Inverter and encoder drivers are pluggable factories:

- Inverters: `esp_foc_inverter_mcpwm_6pwm_new`, `esp_foc_inverter_mcpwm_3pwm_new` (PWM + ADC shunt, ETM wired internally).
- Encoders: AS5600, AS5048A, quadrature via PCNT, simulated (`esp_foc_encoder_simu_new` + optional `esp_foc_encoder_simu_wire_ud_uq`).

---

### Reference firmware: `axis_shell`

Motor control + **console shell** ([`doc/SHELL.md`](doc/SHELL.md), UART used by
`idf monitor`). Default build uses **FITL** (simulated plant); HW pin options in
[`doc/AXIS_SHELL.md`](doc/AXIS_SHELL.md). After flash: `align 0`, `run 0` on the monitor.

```bash
cd examples/axis_shell
idf.py set-target esp32c6
idf.py build flash monitor
```

---

## Minimal example

Unified MCPWM inverter + AS5600 encoder. PI gains default to bypass at init; NVS calibration
or the shell `store` command can persist tuned values.

The snippet below is **illustrative** (placeholders for pins and ADC
config will not compile until you fill them in). For a **complete,
buildable** wiring and init sequence, use
[`examples/axis_shell/main/main.c`](examples/axis_shell/main/main.c).

```c
#include "espFoC/esp_foc.h"
#include "espFoC/drivers/esp_foc_inverter_mcpwm.h"
#include "esp_foc_encoder_as5600.h"

static esp_foc_axis_t axis;
static esp_foc_motor_control_settings_t settings = {
    .motor_pole_pairs  = 4,
    .natural_direction = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW,
    .motor_unit        = 0,
};

static void regulation_callback(esp_foc_axis_t *axis_cb,
                                esp_foc_d_current_q16_t *id_ref,
                                esp_foc_q_current_q16_t *iq_ref)
{
    (void)axis_cb;
    id_ref->raw = 0;
    iq_ref->raw = q16_from_float(2.0f);
}

void app_main(void)
{
    esp_foc_inverter_t *inv = esp_foc_inverter_mcpwm_6pwm_new(
        /* u_hi, u_lo, v_hi, v_lo, w_hi, w_lo, en, vdc, port, adc_u, adc_v, gain, shunt */);
    esp_foc_encoder_t *enc = esp_foc_encoder_as5600_new(/* sda, scl, port */);

    esp_foc_initialize_axis(&axis, inv, enc, settings);
    esp_foc_align_axis(&axis);
    esp_foc_run(&axis);
    esp_foc_set_regulation_callback(&axis, regulation_callback);
}
```

At init, shunt calibration uses **`CONFIG_ESP_FOC_ISENSOR_CALIBRATION_ROUNDS`**
averages (same *Control loop* menu as downsampling).

---

## Examples

- `examples/axis_shell` — reference firmware (console shell + NVS).
- `examples/unit_test_runner` — Unity suite for CI / QEMU.
- `examples/test_drivers` — inverter / encoder / shunt bring-up.

---

## Numerical format

Q16.16 fixed-point (`q16_t`) is used everywhere in the hot path:
currents, voltages, angles, PID, filters, SVPWM. A Q1.31 (IQ31) LUT
backs sin/cos for Park transforms. Float is reserved for setup-time
conversions via `q16_from_float()` / `q16_to_float()`; the control
loop contains no floating-point operations.

---

## Repository layout

```
espFoC/
├── doc/
├── doc/images/         # architecture, demo gifs
├── examples/           # axis_shell / unit_test_runner / test_drivers
├── include/espFoC/     # public API (esp_foc_inverter, esp_foc_encoder, axis, shell)
├── scripts/
├── source/
│   ├── calibration/
│   ├── drivers/        # MCPWM inverter, encoders, FITL tick
│   ├── motor_control/  # axis core, FOC ISR, Q16 helpers
│   ├── shell/          # espfoc_shell REPL
│   └── osal/
└── test/
```

---

## Changelog

Per-release notes live in [`changelog.txt`](changelog.txt) (consumed
by the GitHub release notes generator).

---

## License

MIT — see `LICENSE`.

## Contributing

Issues, feature requests and pull requests are welcome.

Maintainer: **Felipe Neves** — `ryukokki.felipe@gmail.com`
