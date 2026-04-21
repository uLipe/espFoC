# espFoC
### Field Oriented Control (FOC) library for PMSM / BLDC motors on ESP32

![Build](https://github.com/uLipe/espFoC/workflows/Build/badge.svg)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

![alt text](doc/images/espfoc_demo.gif)

espFoC is a **modular, real-time Field Oriented Control (FOC) library** for **PMSM and BLDC motors** on the **ESP32 family** using **ESP-IDF**. It provides motor driving and the full FOC signal chain: inverter control, current sensing, rotor feedback (sensor or observer), and torque (Id/Iq) regulation. Velocity and position loops are not included; you implement them in the regulation callback using the axis state (e.g. `axis->current_speed`, `axis->rotor_position`).

The project focuses on:
- deterministic timing
- clear separation between control logic and hardware drivers
- extensibility (inverters, sensors, control strategies)
- real hardware usage (not simulation)

---

## Key Features

- Voltage-mode FOC (open-loop and sensored)
- Current-mode FOC (Id/Iq) with ADC shunt sensing and rotor sensor (sensored) or observer (sensorless, experimental)
- Modular driver set:
  - Inverters (3-PWM, 6-PWM MCPWM)
  - Rotor sensors (encoders, observers; optional for open-loop)
  - Current sensors (ADC shunt)
- Hardware-synchronized PWM loop and MCPWM dead-time insertion
- **ESP-IDF v5+**

---

## Quick Start

### 1. Install via ESP-IDF (recommended)

```bash
idf.py add-dependency "ulipe/espfoc^2.0.0"
```

Then:

```bash
idf.py set-target esp32        # or esp32s3 / esp32p4
idf.py menuconfig
idf.py build flash monitor
```

---

## Manual Component Integration

Alternatively, clone this repository and add it to your project:

```cmake
set(EXTRA_COMPONENT_DIRS "path/to/espFoC")
```

Build an example:

```bash
cd examples/axis_sensored
idf.py build flash monitor
```

Sensored quick start:

```bash
cd examples/axis_sensored
idf.py set-target esp32s3
idf.py build flash monitor
```

---

## Architectural Overview

espFoC is built around a **motor axis abstraction**. Each axis owns one
inverter, one rotor sensor, and (optionally) one current sensor. The
application provides a regulation callback that sets Id/Iq references;
espFoC handles everything from the PID current loop down to PWM duty
generation. Velocity and position loops are left to the application.

![espFoC architecture](doc/images/architecture.png)

This design allows mixing and matching hardware blocks without changing the control core. espFoC is limited to motor driving and the torque (current) loop; velocity and position control are left to the application via the regulation callback.

---

## Control Modes

### Voltage Mode
- Direct control of Ud/Uq
- Supports open-loop and sensored operation
- Typical use: motor bring-up, observer tuning, simple drives

### Current Mode
- Closed-loop Id/Iq control
- Requires a current sensor; sensored mode also requires a rotor sensor (encoder or equivalent)
- Enables torque control (e.g. thrust or effort in EVs, drones, tools)

---

## Inverter Drivers

### 3-PWM Inverters
- LEDC based
- MCPWM based
- Single output per phase
- Suitable for integrated power stages

### 6-PWM MCPWM Inverter
- Complementary outputs per phase (high-side / low-side)
- Hardware dead-time insertion
- Suitable for:
  - discrete MOSFET bridges
  - external gate drivers

Dead-time is handled **in hardware** using the MCPWM peripheral and is
configured conservatively by default.

> Dead-time configuration assumes complementary outputs and appropriate
> external power stage protection.

---

## Rotor Sensors

Supported rotor sensing methods include:
- Open-loop (no sensor)
- Incremental encoders
- Magnetic encoders (e.g. AS5600)
- Observer-based estimation

Rotor sensors are implemented as pluggable drivers and can be replaced
without modifying the control logic.

---

## Current Sensing

espFoC supports **ADC shunt current sensing**:
- dual or three-shunt configurations
- Synchronized with PWM events
- Used for:
  - current-mode FOC
  - observer feedback

---

## Real-Time Design

espFoC is designed around **hardware-driven timing**:

- PWM timer drives the control loop
- ADC sampling is synchronized to PWM
- High-speed ISR captures timestamps and samples
- Control computation runs in a deterministic high priority task

---

## PI Tuning Workflow

espFoC ships with three complementary mechanisms for the current-loop PI:
build-time autotuning, runtime retune, and a transport-agnostic tuner
backend with optional signal injection. All three share the same
**Matched Pole-Zero (MPZ) discrete-time synthesis** so the gains are
consistent across boot, host tooling, and live retuning.

The closed-form design assumes the standard first-order PMSM current
plant `G(s) = 1 / (R + L·s)` with ZOH discretization at the loop sample
period `Ts = decimation / f_pwm`:

```
alpha = exp(-R*Ts/L)              # discrete plant pole
beta  = exp(-2*pi*bw_hz*Ts)       # desired closed-loop pole
Kp    = R*(1 - beta)/(1 - alpha)
Ki    = R*(1 - beta)/Ts
```

### 1. Build-time autotuner

Motor profiles live under `scripts/motors/*.json`. Select one with
`CONFIG_ESP_FOC_MOTOR_PROFILE` and the build runs `scripts/gen_pi_gains.py`,
emitting `esp_foc_autotuned_gains.h` into the **build directory** (never the
source tree, defended both by `.gitignore` rules and by the script's
`--allow-in-tree` guard). The gains are picked up at boot whenever
`esp_foc_motor_control_settings_t.motor_resistance` and `motor_inductance`
are zero.

Sample build-time output:

```
$ python3 scripts/gen_pi_gains.py \
      --motor scripts/motors/default.json \
      --output build/.../gen/esp_foc_autotuned_gains.h \
      --report build/.../gen/esp_foc_autotuned_report.txt \
      --pwm-rate-hz 20000 --decimation 20
WARNING: phase margin (with 1-sample delay) is only 36.7 deg.
espFoC PI autotuner: build/.../esp_foc_autotuned_gains.h generated
                     (Kp=1.4610 V/A, Ki=659.17 V/(A*s), PM_delay=36.7 deg)
```

The report sidecar captures the full design context:

```
espFoC PI autotuning report
===========================
Profile     : iPower GBM5208-200T (gimbal reference) (default.json)
Motor       : R = 1.080000 ohm, L = 1800.000 uH
Loop        : PWM = 20000 Hz, decimation = 20, Ts = 1000 us
Target BW   : 150.000 Hz (bw*Ts = 0.1500)
V max       : 12.000 V

Discrete plant pole alpha = exp(-R*Ts/L) = 0.548812
Closed-loop pole    beta  = exp(-w_bw*Ts) = 0.389661

Kp                 = 1.460955 V/A
Ki                 = 659.166 V/(A*s)
Integrator limit   = 12.000 V

Phase margin (no delay)        : 72.2 deg
Phase margin (1-sample delay)  : 36.7 deg
```

The script aborts the build when the design is unsafe. Two examples:

```
# bw above Nyquist: bw_hz * Ts >= 0.5
ERROR: design failed: bandwidth 1000.0 Hz exceeds Nyquist for Ts=1000.0us
       (bw*Ts=1.000 >= 0.5)
exit=4

# phase margin (with 1-sample delay) below the profile's safety floor
ERROR: phase margin (with 1-sample delay) 27.1 deg < required 60.0 deg.
       Reduce target bandwidth or revisit the motor profile.
exit=5
```

### 2. Runtime retune API

`include/espFoC/esp_foc_axis_tuning.h` exposes three primitives:

```c
esp_foc_axis_retune_current_pi_q16(axis, R_q16, L_q16, bw_hz_q16);
esp_foc_axis_set_current_pi_gains_q16(axis, Kp, Ki, integrator_limit);
esp_foc_axis_get_current_pi_gains_q16(axis, &Kp, &Ki, &lim);
```

`retune` invokes the same MPZ math the Python generator uses (now in
fixed-point Q16, `source/motor_control/esp_foc_design_mpz.c`) and swaps
the gains atomically through a critical section. The integrator and
previous-error history are reset to keep the post-swap response
well-defined.

### 3. Tuner backend & signal injection

Behind `CONFIG_ESP_FOC_TUNER_ENABLE`, `esp_foc_tuner.c` exposes a
transport-agnostic request handler that maps to read / write / exec
opcodes (`include/espFoC/esp_foc_tuner.h`). Three weak callbacks
(`esp_foc_tuner_init_bus_callback`, `_recv_callback`, `_send_callback`)
let the application plug UART, USB-CDC, BLE or TCP without touching
the core.

With `CONFIG_ESP_FOC_INJECTION_ENABLE` the q-axis reference can be
augmented with a step or chirp from `esp_foc_axis_inject_step_q16()` /
`esp_foc_axis_inject_chirp_q16()`, which closes the loop with the
existing scope: arm injection, capture, plot, repeat.

### 4. End-to-end demo (runs in QEMU)

`examples/tuner_demo` is a self-contained app that exercises every
piece above. From the example directory:

```bash
. $IDF_PATH/export.sh
idf.py set-target esp32
idf.py build
python run_qemu.py
```

Output captured from QEMU:

```
I (..) tuner-demo: espFoC tuner demo starting (PWM=20000 Hz, decimation=20)
I (..) tuner-demo: === Initial gains (build-time autogen, default.json) ===
I (..) tuner-demo: autogen default        Kp=   1.4610 V/A   Ki=   659.17 V/(A*s)   ILim= 12.00 V
I (..) tuner-demo: === Runtime retune via MPZ helper ===
I (..) tuner-demo: retune iPower (R=1.08,L=1.8mH): R=1.080 L=1.8000mH bw=150Hz
I (..) tuner-demo: iPower (R=1.08,L=1.8mH) Kp=   1.4613 V/A   Ki=   659.15 V/(A*s)   ILim=  0.58 V
I (..) tuner-demo: retune low-R servo  (R=0.25,L=0.8mH): R=0.250 L=0.8000mH bw=200Hz
I (..) tuner-demo: low-R servo  (R=0.25,L=0.8mH) Kp=   0.6618 V/A   Ki=   178.85 V/(A*s)   ILim=  0.58 V
I (..) tuner-demo: retune high-L motor (R=0.80,L=5.0mH): R=0.800 L=5.0000mH bw=50Hz
I (..) tuner-demo: high-L motor (R=0.80,L=5.0mH) Kp=   1.4600 V/A   Ki=   215.67 V/(A*s)   ILim=  0.58 V
I (..) tuner-demo: Asking for a bandwidth above Nyquist (bw=600Hz, Ts=1ms):
I (..) tuner-demo: retune above-nyquist demo: R=0.500 L=1.0000mH bw=600Hz
W (..) tuner-demo: retune REJECTED (err=-2) -- bw probably above Nyquist for the current loop period
I (..) tuner-demo: === Tuner protocol round-trip ===
I (..) tuner-demo: READ Kp -> 1.4600 V/A
I (..) tuner-demo: WRITE Kp = 2.345 (manual override)
I (..) tuner-demo: READ Kp -> 2.3450 V/A (after write)
I (..) tuner-demo: EXEC RECOMPUTE_GAINS R=1.08 L=1.8mH bw=150Hz
I (..) tuner-demo: READ Kp -> 1.4613 V/A (after MPZ recompute)
I (..) tuner-demo: READ Ki -> 659.15 V/(A*s)
I (..) tuner-demo: === Signal injection (step) ===
I (..) tuner-demo: arm_step amplitude=0.5A duration=5ms -> OK
I (..) tuner-demo: step sample[0] = 0.500 A
I (..) tuner-demo: step sample[1] = 0.500 A
I (..) tuner-demo: step sample[2] = 0.500 A
I (..) tuner-demo: step sample[3] = 0.500 A
I (..) tuner-demo: step sample[4] = 0.000 A
I (..) tuner-demo: === Signal injection (chirp 100->500 Hz) ===
I (..) tuner-demo: arm_chirp amplitude=0.25A 100->500Hz over 10ms -> OK
I (..) tuner-demo: chirp observed peak = 0.250 A (expected ~0.25)
I (..) tuner-demo: === Demo complete ===
```

A few sanity checks on those numbers:

- **autogen default vs iPower retune** produce the same Kp/Ki because
  `default.json` *is* the iPower gimbal motor — the runtime path matches
  the build-time path within Q16 quantization (1 LSB).
- **low-R / high-L** motors yield clearly different Kp values, showing the
  MPZ design adapting to the plant.
- The integrator limit drops from `12.00 V` (build-time, from JSON
  `v_max_volts`) to `0.58 V` after runtime retune. That's expected: the
  runtime path uses the **actual** `axis->max_voltage`, derived from the
  inverter's reported DC link (here `1.0 V` with SVPWM →
  `1/√3 ≈ 0.577 V`).
- The above-Nyquist request is rejected cleanly (`err=-2`) instead of
  silently corrupting the gains.
- Step injection delivers exactly five samples at 0.5 A then auto-disables
  (5 ms @ 1 ms loop period, less the Q16 quantization of `Ts`).
- Chirp peak matches the configured amplitude (0.25 A).

### Cross-validation Python ↔ C

`test/golden_motors.json` is the single source of truth for the regression
table. `scripts/verify_goldens.py` checks both that the Python math agrees
with the JSON and that the C test mirror in `test/test_design_mpz.c`
matches it byte-for-byte:

```
$ python3 scripts/verify_goldens.py
All 5 goldens agree across JSON, Python math, and C mirror.
```

### 5. TunerStudio (host GUI + CLI)

`tools/espfoc_studio/` hosts the PySide6 + pyqtgraph desktop app and the
`tunerctl` CLI that drive the firmware through the runtime tuner. The
whole stack (link codec, protocol client, motor model, GUI) is pure
Python and can be exercised against a simulated firmware with no hardware
attached:

```
pip install -r tools/espfoc_studio/requirements.txt
PYTHONPATH=tools python3 -m espfoc_studio.gui --demo
```

Swap `--demo` for `--port /dev/ttyACM0` once you flash a board built with
`CONFIG_ESP_FOC_BRIDGE_UART` or `CONFIG_ESP_FOC_BRIDGE_USBCDC`. See
`tools/espfoc_studio/README.md` for the full layout.

---

## Examples

The repository includes multiple examples demonstrating:
- Sensored mode (`examples/axis_sensored`)
- Sensorless mode (`examples/axis_sensorless`) — **temporarily disabled**:
  the rotor observer is not yet wired into the axis. The example aborts at
  compile time with a clear error unless `CONFIG_EXAMPLE_BUILD_INCOMPLETE_SENSORLESS`
  is set, and even then app_main logs an error and returns without driving
  the inverter.
- Inverter and sensor bring-up (`examples/test_drivers`)
- **tuner_demo** (`examples/tuner_demo`): self-contained app that exercises
  build-time autogen, runtime retune, the tuner protocol, and signal
  injection. Runs unchanged in QEMU via `python run_qemu.py`.
- **unit_test_runner**: app that runs the Unity unit tests (for CI or local
  validation). Build with `idf.py -D TEST_COMPONENTS=espFoC build` from
  `examples/unit_test_runner`.

Examples are located in the `examples/` directory.

---

## Sample Code

Below is a minimal example that initializes espFoC and runs the motor in **sensored current mode** with:

- MCPWM inverter
- rotor sensor (e.g. AS5600)
- current sensor (ADC shunt)

```c
#include "esp_log.h"
#include "esp_err.h"

#include "espFoC/inverter_6pwm_mcpwm.h"
#include "espFoC/current_sensor_adc.h"
#include "espFoC/rotor_sensor_as5600.h"
#include "espFoC/esp_foc.h"
#include "espFoC/utils/esp_foc_q16.h"

static const char *TAG = "esp-foc-example";

static esp_foc_inverter_t *inverter;
static esp_foc_isensor_t  *shunts;
static esp_foc_rotor_sensor_t *sensor;

static esp_foc_axis_t axis;
static esp_foc_motor_control_settings_t settings = {
    .motor_pole_pairs = 4,
    .motor_inductance = 0,
    .motor_resistance = 0,
    .natural_direction = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW,
    .motor_unit = 0,
};

static void regulation_callback(esp_foc_axis_t *axis_cb,
                                esp_foc_d_current_q16_t *id_ref,
                                esp_foc_q_current_q16_t *iq_ref,
                                esp_foc_d_voltage_q16_t *ud_forward,
                                esp_foc_q_voltage_q16_t *uq_forward)
{
    (void)axis_cb;
    uq_forward->raw = q16_from_float(0.0f);
    ud_forward->raw = q16_from_float(0.0f);
    iq_ref->raw = q16_from_float(2.0f);
    id_ref->raw = q16_from_float(0.0f);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing espFoC");

    settings.motor_inductance = q16_from_float(0.0018f);
    settings.motor_resistance = q16_from_float(1.08f);

    inverter = inverter_6pwm_mpcwm_new(/* ... pin config ... */);
    shunts   = /* ... ADC shunt config ... */;
    sensor   = rotor_sensor_as5600_new(/* ... I2C pins ... */);

    esp_foc_initialize_axis(&axis, inverter, sensor, shunts, settings);
    esp_foc_align_axis(&axis);
    esp_foc_run(&axis);
    esp_foc_set_regulation_callback(&axis, regulation_callback);
}
```

## Numerical Format

espFoC uses **Q16.16 fixed-point** (`q16_t`) for all runtime control signals
(currents, voltages, angles, PID, filters, SVPWM). An IQ31 (Q1.31) LUT
provides sin/cos and is also used inside observers. **Float** is used only in
the public API for setup / parameter conversion (e.g. `q16_from_float()`);
the hot-path control loop contains no floating-point operations.

---

## Repository Structure

```
  espFoC/
  ├── doc
  │   └── images
  ├── examples   # Examples with examples projects and bring-up code
  │   ├── axis_sensored
  │   ├── axis_sensorless
  │   └── test_drivers
  ├── include   # Public API header files
  │   └── espFoC
  └── source
      ├── drivers         # Platform specific drivers
      ├── motor_control   # Motor control algorithms.
      ├── osal            # OS abstraction layer
      └── rust            # plain FFI for future rust usage

```

---

## Project Status

espFoC is a personal project. The 2.x API focuses on motor driving and FOC only; velocity and position loops are not part of the library. Future releases may introduce breaking changes.

---

## License

espFoC is released under the **MIT License**.

See `LICENSE` for details.

---

## Contributing

Issues, feature requests, and pull requests are welcome.

Maintainer: **Felipe Neves**
`ryukokki.felipe@gmail.com`

---
