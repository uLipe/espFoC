# espFoC – Vector FOC Controller for PMSM / BLDC Motors on ESP32

![Build](https://github.com/uLipe/espFoC/workflows/Build/badge.svg)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

![alt text](doc/images/espfoc_demo.gif)

**espFoC** is a Field-Oriented Control (**FOC**) library for **PMSM / BLDC motors** running on **ESP32 SoCs**. It is distributed as an **ESP-IDF component**, providing a clean and extensible architecture for voltage-mode and current-mode motor control, speed/position loops, and multiple sensor/inverter backends.

The goal of espFoC is to provide a **simple, readable, and flexible** FOC engine that can be integrated into research projects.

---

## ✨ Features

- **Voltage-mode FOC**
  Apply direct d/q-axis voltages (`Ud`, `Uq`). The library handles Park/Clarke/SVPWM automatically.

- **Current-mode FOC (Id/Iq control)**
  When a current sensor backend (`esp_foc_isensor_t`) is provided, the axis runs **current-regulated FOC** using internal PI controllers.
  This enables higher torque bandwidth, better dynamic performance, and supports closed-loop speed/position control.

- **Closed-loop speed and position control**
  Configure outer loops through `esp_foc_motor_control_settings_t`.

- **Rotor sensor backends included**
  - Simulated/open-loop rotor sensor (electrical + mechanical model)
  - I²C encoder (e.g., AS5600)
  - Analog encoder
  - Dummy/no-sensor mode for test projects

- **Inverter driver backends**
  - 3-PWM inverter using ESP32 MCPWM/LEDC (`inverter_3pwm_mcpwm`)

- **Current sensor backend**
  - ADC shunt-based measurement (`current_sensor_adc.h`)

- **FOC loop synchronized to PWM**
  Ensures deterministic sampling and consistent current/angle alignment.

- **Built-in scope & telemetry**
  Compatible with **Better Serial Plotter** for real-time visualization.

- **ESP-IDF integration**
  Install directly from the Espressif Component Registry.

---

## 🧱 High-Level Architecture

The core of espFoC is the **axis**, represented by:

### `esp_foc_axis_t`
This object contains:
- Motor configuration
- Internal FOC controller
- Optional current controller
- Speed and position loops
- Links to hardware drivers (inverter, current sensor, rotor sensor)

The axis is composed of three independent backends:

### `esp_foc_inverter_t`
Generates the 3-phase PWM output.

### `esp_foc_rotor_sensor_t`
Provides electrical/mechanical angle and speed.

### `esp_foc_isensor_t` (optional)
Provides phase current samples for **current-mode control**.
If not provided, the axis runs in **voltage-mode**.

### Configuration structure:
#### `esp_foc_motor_control_settings_t`
Includes:
- Motor parameters (pole pairs, direction)
- Current-loop gains (if in current mode)
- Speed and position controller gains
- Downsampling rates
- Output limits

---

## 📚 Examples

All examples can be created directly via the Espressif Component Registry:

| Example          | Description                                                                 |
|------------------|-----------------------------------------------------------------------------|
| `open_loop`      | Open-loop voltage FOC using the simulated rotor sensor.                     |
| `current_control`| **Current-mode FOC** using ADC shunts and simulated rotor sensor.           |
| `simple_velocity`| Basic speed loop using a 3-PWM inverter + analog encoder.                   |
| `simple_servo`   | Closed-loop **position** control using I²C encoder.                         |
| `speed_control`  | Closed-loop **speed** control using I²C encoder.                            |
| `test_project`   | Testbench for voltage and current FOC using simulated rotor sensor.         |

For micro-ROS integration, see `microROS_FoC`.

---

## 🚀 Quick Start

### 1. Install via ESP-IDF (recommended)

```bash
idf.py add-dependency "ulipe/espfoc^1.5.1"
````

Then:

```bash
idf.py set-target esp32        # or esp32s3 / esp32p4
idf.py menuconfig
idf.py build flash monitor
```

---

## 📦 Manual Component Integration

Alternatively, clone this repository and add it to your project:

```cmake
set(EXTRA_COMPONENT_DIRS "path/to/espFoC")
```

Build an example:

```bash
cd examples/current_control
idf.py build flash monitor
```

---

## 🌪️ Minimal Open-Loop Voltage-Mode Example

```c
#include "esp_log.h"
#include "esp_err.h"

#include "espFoC/rotor_sensor_open_loop.h"
#include "espFoC/inverter_3pwm_mcpwm.h"
#include "espFoC/esp_foc.h"

static const char *TAG = "esp-foc-example";

static esp_foc_inverter_t     *inverter;
static esp_foc_rotor_sensor_t *sensor;
static esp_foc_axis_t          axis;

static esp_foc_motor_control_settings_t settings = {
    .downsampling_position_rate = 0,
    .downsampling_speed_rate    = 0,
    .motor_pole_pairs           = 4,
    .natural_direction          = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW,
};

static void initialize_foc_drivers(void)
{
    inverter = inverter_3pwm_mpcwm_new(
        CONFIG_FOC_PWM_U_PIN,
        CONFIG_FOC_PWM_V_PIN,
        CONFIG_FOC_PWM_W_PIN,
        CONFIG_FOC_PWM_EN_PIN,
        24.0f,
        0
    );

    sensor = rotor_sensor_open_loop_new(
        1.5f,
        0.012f,
        &axis.target_u_q.raw,
        &axis.dt
    );
}

void app_main(void)
{
    esp_foc_q_voltage uq = { .raw = 5.0f };

    initialize_foc_drivers();

    esp_foc_initialize_axis(&axis, inverter, sensor, NULL, settings);
    esp_foc_align_axis(&axis);
    esp_foc_run(&axis);

    esp_foc_set_target_voltage(&axis, uq, (esp_foc_d_voltage){ .raw = 0.0f });
}
```

---

## ⚡ Current-Mode Example Overview

In **current-mode**, the user must provide a current sensor backend:

```c
esp_foc_isensor_t *cs = current_sensor_adc_new(...);

esp_foc_initialize_axis(
    &axis,
    inverter,
    sensor,
    cs,      // enables current-mode FOC (Id/Iq control)
    settings
);
```

The FOC engine will:

* Read currents via ADC
* Regulate Id → 0 A
* Regulate Iq → desired torque-producing current
* Apply SVPWM based on controller output

See `examples/current_control`.

---

## 🔌 Typical Hardware Setup

* ESP32 / ESP32-S3 / ESP32-P4 board
* 3-PWM inverter (L6234, DRV83xx, MakerBase FoC board, etc.)
* PMSM / BLDC motor
* One of:

  * I²C encoder (AS5600, AS5048, etc.)
  * Analog encoder
  * Simulated rotor sensor (testing, no real motor needed)
* Optional:

  * ADC-based current sensing for **current-mode FOC**

Pinouts are fully configurable through `menuconfig`. For the voltage-mode
The typical wiring is shown below:

![Wiring](/doc/images/wiring.png)

---

## 🧪 Supported Devices

* **ESP32**
* **ESP32-S3**
* **ESP32-P4**

Requires **ESP-IDF v5.0+**.

---

## 📈 Real-Time Debugging

Enable scope output in menuconfig:

```
CONFIG_ESP_FOC_SCOPE
```

Then use **Better Serial Plotter** to visualize:

* d/q currents
* rotor position
* PWM duty cycles
* controller outputs

---

## 🤝 Contributing

Issues, feature requests, and pull requests are welcome.

Maintainer: **Felipe Neves**
`ryukokki.felipe@gmail.com`

---

## 📄 License

espFoC is licensed under the **MIT License**.
See `LICENSE` for details.