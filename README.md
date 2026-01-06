# espFoC
### Field Oriented Control (FOC) library for PMSM / BLDC motors on ESP32


![Build](https://github.com/uLipe/espFoC/workflows/Build/badge.svg)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

![alt text](doc/images/espfoc_demo.gif)

espFoC is a **modular, real-time oriented Field Oriented Control (FOC) library**
designed for **PMSM and BLDC motors**, targeting the **ESP32 family** using
**ESP-IDF**.

The project focuses on:
- deterministic timing
- clean separation between control logic and hardware drivers
- extensibility (inverters, sensors, control strategies)
- real hardware usage (not simulations)

espFoC behaves like a **smart motor drive implemented as a library**.

---

## ✨ Key Features

- Voltage-mode FOC (open-loop and sensored)
- Current-mode FOC (Id / Iq) using ADC shunt sensing
- Speed and position control loops
- Open-loop operation with observer support
- Modular driver architecture:
  - Inverters (3-PWM, 6-PWM MCPWM)
  - Rotor sensors (encoders, observers, open-loop)
  - Current sensors (ADC shunt)
- Hardware-synchronized PWM loop
- Dead-time insertion using MCPWM hardware
- Designed for **ESP-IDF v5+**

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

## 🧠 Architectural Overview

![Wiring](/doc/images/wiring.png)

espFoC is built around a **motor axis abstraction**.

Each axis is composed of:
- one **inverter driver**
- one **rotor sensor** (optional for open-loop)
- one **current sensor** (optional)
- one **control strategy**

```

Application
|
v
esp_foc_axis
|
+-- inverter (PWM generation)
+-- rotor sensor (position / speed)
+-- current sensor (Id / Iq)
+-- control strategy (voltage / current / speed / position)

```

This design allows mixing and matching hardware blocks without
changing the control core.

---

## ⚙️ Control Modes

### Voltage Mode
- Direct control of Ud / Uq
- Supports:
  - open-loop operation
  - sensored operation
- Commonly used for:
  - motor bring-up
  - observer initialization
  - simple control applications

### Current Mode
- Closed-loop Id / Iq control
- Requires current sensor
- Enables:
  - torque control
  - robust speed control
  - position control

### Speed & Position Control
- Implemented as outer control loops
- Executed deterministically inside espFoC
- Applications only provide setpoints

---

## 🔌 Inverter Drivers

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

> ⚠️ Dead-time configuration assumes complementary outputs and appropriate
> external power stage protection.

---

## 📐 Rotor Sensors

Supported rotor sensing methods include:
- Open-loop (no sensor)
- Incremental encoders
- Magnetic encoders (e.g. AS5600)
- Observer-based estimation

Rotor sensors are implemented as pluggable drivers and can be replaced
without modifying the control logic.

---

## ⚡ Current Sensing

espFoC supports **ADC shunt current sensing**:
- Single or multi-shunt configurations
- Synchronized with PWM events
- Used for:
  - current-mode FOC
  - observer feedback
  - protection mechanisms

---

## ⏱️ Real-Time Design

espFoC is designed around **hardware-driven timing**:

- PWM timer drives the control loop
- ADC sampling is synchronized to PWM
- High-speed ISR captures timestamps and samples
- Control computation runs in a deterministic task

This architecture minimizes jitter and ensures stable motor behavior.

---

## 🧪 Examples

The repository includes multiple examples demonstrating:
- Open-loop voltage control
- Sensored voltage mode
- Speed and position control
- Current-mode FOC
- Observer usage
- Inverter and sensor bring-up

Examples are located in the `examples/` directory.

---

## 🦀 Rust Sample (esp-idf-sys)

This repository includes an optional Rust integration under [`rust/`](rust/), built on top of **`esp-idf-sys` (STD)**.
The Rust code talks to espFoC through a small C FFI shim (`include/rust/` + `source/rust/`).

### Build & Flash (example: open-loop Vq ramp)

1) Go to the Rust folder:

```bash
cd rust
```

2) Build (pick the correct target for your MCU):

- ESP32: `xtensa-esp32-espidf`
- ESP32-S3: `xtensa-esp32s3-espidf`
- ESP32-C3: `riscv32imc-esp-espidf`

```bash
cargo build --target xtensa-esp32-espidf
```

3) Flash + monitor the example:

```bash
MCU=esp32 cargo espflash flash --target xtensa-esp32-espidf --example open_loop_voltage_ramp --monitor
```

> Adjust GPIO assignments and DC link voltage at the top of `rust/examples/open_loop_voltage_ramp.rs`.

For more details, see `rust/README.md`.

---

## 🚀 Sample Code

Below is a minimal example showing how to initialize and run espFoC and drive the
target motor in **sensored voltage mode** using:

- MCPWM inverter
- rotor sensor
- optional current sensor

```c
#include "esp_log.h"
#include "esp_err.h"

#include "espFoC/rotor_sensor_as5600.h"
#include "espFoC/inverter_3pwm_mcpwm.h"
#include "espFoC/current_sensor_adc.h"
#include "espFoC/esp_foc.h"

static const char *TAG = "esp-foc-example";

static esp_foc_inverter_t *inverter;
static esp_foc_inverter_t *inverter;
static esp_foc_isensor_t  *shunts;

static esp_foc_rotor_sensor_t *sensor;
static esp_foc_axis_t axis;
static esp_foc_motor_control_settings_t settings = {
    .motor_pole_pairs = 4,
    .velocity_control_settings.kp = 1.0f,
    .velocity_control_settings.ki = 0.0f,
    .velocity_control_settings.kd = 0.0f,
    .velocity_control_settings.integrator_limit = 100.0f,
    .velocity_control_settings.max_output_value = 4.0f, //conservative setpoint to the current controller
    .torque_control_settings[0].max_output_value = 6.0f, //Uses the max driver voltage allowed as limit
    .natural_direction = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW,
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

    if(inverter == NULL) {
        ESP_LOGE(TAG, "failed to create the inverter driver, aborting!");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
        abort();
    }

    sensor = rotor_sensor_as5600_new(
        CONFIG_FOC_ENCODER_SDA_PIN,
        CONFIG_FOC_ENCODER_SCL_PIN,
        0
    );

    if(sensor == NULL) {
        ESP_LOGE(TAG, "failed to create the inverter driver, aborting!");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
        abort();
    }

    esp_foc_isensor_adc_config_t shunt_cfg = {
        .axis_channels = {ADC_CHANNEL_7, ADC_CHANNEL_6},
        .amp_gain = 50.0f,
        .shunt_resistance = 0.01f,
        .number_of_channels = 2,
    };

    shunts = isensor_adc_new(&shunt_cfg);
    if(sensor == NULL) {
        ESP_LOGE(TAG, "failed to create the shunt sensor driver, aborting!");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
        abort();
    }
}

void app_main(void)
{
    esp_foc_control_data_t control_data;
    esp_foc_q_voltage uq = {.raw = -6.0f};

    ESP_LOGI(TAG, "Initializing the esp foc motor controller!");

    initialize_foc_drivers();

    esp_foc_initialize_axis(
        &axis,
        inverter,
        sensor,
        shunts,
        settings
    );

    esp_foc_align_axis(&axis);
    esp_foc_run(&axis);

    /* Set velocity by using state vector given by vq+vd */
    esp_foc_set_target_voltage(&axis, (esp_foc_q_voltage){.raw = 0.0}, (esp_foc_d_voltage){.raw = 0.0});

    /* ramp the velocity */
    uq.raw = 1.0f;

    while(1) {
        esp_foc_set_target_voltage(&axis, uq, (esp_foc_d_voltage){.raw = 0.0});
        uq.raw *= -1.0f;
        esp_foc_sleep_ms(200);
        esp_foc_get_control_data(&axis, &control_data);
        ESP_LOGI(TAG, "Estimated speed: %f rad/s, dt: %f s", control_data.speed.raw, control_data.dt.raw);
        esp_foc_set_target_voltage(&axis, (esp_foc_q_voltage){.raw = 0.0}, (esp_foc_d_voltage){.raw = 0.0});
        esp_foc_sleep_ms(200);
    }
}

```

## 📁 Repository Structure

```
espFoC/
├── include/        # Public headers
├── source/         # Core implementation and drivers
│   ├── drivers/
│   ├── motor_control/
│   └── strategies/
├── examples/       # Example applications
├── doc/            # Additional documentation
└── README.md

```

---

## 🚧 Project Status

espFoC is experimental but **actively developed** and tested on real hardware.

---

## 📜 License

espFoC is released under the **MIT License**.

See `LICENSE` for details.

---

## 🤝 Contributing

Issues, feature requests, and pull requests are welcome.

Maintainer: **Felipe Neves**
`ryukokki.felipe@gmail.com`

---