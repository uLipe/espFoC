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
firmware API, persisted to NVS, or dialled in interactively through
the bundled TunerStudio GUI.

Targets: ESP32, ESP32-S3, ESP32-P4 (ESP-IDF v5+).

> **3.0 is a breaking release.** The legacy continuous-time PI
> formula and the `motor_resistance / motor_inductance / motor_inertia`
> fields are gone — gains come from the build-time autotuner or the
> runtime tuner. The 3-PWM LEDC driver was also dropped. The former
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
cd examples/axis_simple
idf.py set-target esp32s3
idf.py build flash monitor
```

---

## Architecture

![espFoC architecture](doc/images/architecture.png)

Each axis owns one inverter, one rotor sensor and (optionally) one
current sensor. You implement a **regulation callback** that updates
`id_ref` / `iq_ref` each outer-loop tick; espFoC runs Clarke/Park, the
current PIs, inverse Park, SVPWM and PWM updates. The inner current loop
and modulation run in the **PWM ISR** at carrier frequency; a dedicated
task reads the encoder and invokes your callback at roughly **PWM rate
÷ `CONFIG_ESP_FOC_LOW_SPEED_DOWNSAMPLING`** (see *espFoC Settings → Control
loop* in `menuconfig`). Platform primitives (tasks, critical sections,
timers) go through **`include/espFoC/osal/os_interface.h`** so motor code
does not depend on FreeRTOS headers.

Inverter and rotor drivers are pluggable:

- Inverters: 3-PWM MCPWM, 6-PWM MCPWM (hardware dead-time).
- Rotor sensors: AS5600, AS5048A, quadrature via PCNT, simulated rotor (`rotor_sensor_simu` + optional `rotor_sensor_simu_wire_ud_uq` for open-loop bring-up).
- Current sensing: ADC shunt (continuous or one-shot).

---

## Tuning

![TunerStudio demo](doc/images/tuner_studio.gif)

espFoC ships with **TunerStudio**, a PySide6 + pyqtgraph desktop app
that speaks the runtime tuner protocol over UART or USB-CDC. In a
single window you get:

- live axis state and gain readout with in-place editing;
- one-click rotor alignment with auto-detected natural direction;
- save / load / erase calibration to NVS so the next boot comes up
  already tuned;
- predicted step response, Bode, pole-zero and root-locus plots;
- firmware scope stream with per-channel colour, toggle and cursor;
- SVPWM hexagon with the three phase projections and the resultant
  voltage vector rotating as the motor is driven.

### Launch TunerStudio

```bash
pip install -r tools/espfoc_studio/requirements.txt
PYTHONPATH=tools python3 -m espfoc_studio.gui --port /dev/ttyACM0
```

### Talk to a real target

Two paths:

1. **`tuner_studio_target` (recommended for bring-up).** A dedicated
   service-mode firmware that boots, parks the motor, and waits for
   the GUI. Advertises `TSGX` as its firmware-type for host identification.

   ```bash
   cd examples/tuner_studio_target
   idf.py set-target esp32s3        # USB-CDC default
   idf.py menuconfig                # adjust the pin map
   idf.py build flash monitor
   ```

2. **Your own firmware.** Enable a transport bridge in `menuconfig`
   (`CONFIG_ESP_FOC_BRIDGE_UART` for plain ESP32,
   `CONFIG_ESP_FOC_BRIDGE_USBCDC` for S2/S3/P4) and set
   `CONFIG_ESP_FOC_TUNER_ENABLE=y`.

Then:

```bash
PYTHONPATH=tools python3 -m espfoc_studio.gui --port /dev/ttyACM0
```

### Scripted tuning

A companion CLI (`tunerctl`) lets you drive the same protocol from
scripts and CI jobs. Build-time autotuning from motor profiles, the
runtime C API, the wire-level protocol and the CLI commands are
covered in [`doc/TUNING.md`](doc/TUNING.md).

---

## Minimal example

Encoder-based current mode with a 6-PWM MCPWM inverter, an AS5600 encoder
and an ADC shunt. PI gains come from the build-time autotuner for the
motor profile selected via `CONFIG_ESP_FOC_MOTOR_PROFILE`; the runtime
tuner / TunerStudio can rewrite them later.

The snippet below is **illustrative** (placeholders for pins and ADC
config will not compile until you fill them in). For a **complete,
buildable** wiring and init sequence, use
[`examples/axis_simple/main/axis_simple.c`](examples/axis_simple/main/axis_simple.c).

```c
#include "esp_log.h"
#include "esp_err.h"
#include "espFoC/inverter_6pwm_mcpwm.h"
#include "espFoC/current_sensor_adc.h"
#include "espFoC/rotor_sensor_as5600.h"
#include "espFoC/esp_foc.h"
#include "espFoC/utils/esp_foc_q16.h"

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
    esp_foc_inverter_t     *inv    = inverter_6pwm_mpcwm_new(/* pins... */);
    esp_foc_rotor_sensor_t *rotor  = rotor_sensor_as5600_new(/* i2c... */);
    esp_foc_isensor_t      *shunts = /* ... ADC shunt config ... */;

    esp_foc_initialize_axis(&axis, inv, rotor, shunts, settings);
    esp_foc_align_axis(&axis);
    esp_foc_run(&axis);
    esp_foc_set_regulation_callback(&axis, regulation_callback);
}
```

At init, shunt calibration uses **`CONFIG_ESP_FOC_ISENSOR_CALIBRATION_ROUNDS`**
averages (same *Control loop* menu as downsampling).

---

## Examples

- `examples/axis_simple` — reference bring-up (encoder + current-mode FOC).
- `examples/tuner_studio_target` — service-mode firmware for live tuning
  with TunerStudio (`--port`).
- `examples/tuner_demo` — runs in QEMU, exercises autogen gains,
  runtime retune and the tuner protocol.
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
│   ├── images/         # architecture, TunerStudio screenshot, demo gif
│   └── TUNING.md       # deep dive: autogen, runtime API, protocol, CLI
├── examples/           # axis_simple / tuner_studio_target /
│                       # tuner_demo / unit_test_runner / ...
├── include/espFoC/     # public API
├── scripts/
│   ├── gen_pi_gains.py # build-time MPZ autotuner
│   └── motors/*.json   # motor profiles consumed by the autotuner
├── source/
│   ├── calibration/    # NVS calibration format and axis helpers
│   ├── drivers/        # inverters, encoders, shunts, tuner bridges
│   ├── gui_link/       # binary link codec, scope, tuner reactor
│   ├── motor_control/  # axis core (FOC ISR + slow loop), MPZ, Q16 helpers
│   └── osal/           # OS abstraction (tasks, critical sections, esp_timer)
├── test/               # Unity unit tests (run via examples/unit_test_runner)
└── tools/espfoc_studio # PySide6 + pyqtgraph GUI, CLI, host protocol
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
