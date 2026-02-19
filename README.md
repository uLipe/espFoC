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

---

## ✨ Key Features

- Voltage-mode FOC (open-loop and sensored)
- Current-mode FOC (Id / Iq) using ADC shunt sensing plus rotor sensor
- Current-mode FOC (Id / Iq) using ADC shunt in sensorless mode (experimental)
- Speed and position control loops
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
- Requires current sensor, a encoder is required for position control
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
- dual or three-shunt configurations
- Synchronized with PWM events
- Used for:
  - current-mode FOC
  - observer feedback

---

## ⏱️ Real-Time Design

espFoC is designed around **hardware-driven timing**:

- PWM timer drives the control loop
- ADC sampling is synchronized to PWM
- High-speed ISR captures timestamps and samples
- Control computation runs in a deterministic high priority task

---

## 🧪 Examples

The repository includes multiple examples demonstrating:
- Sensored mode
- Sensorless mode
- Inverter and sensor bring-up

Examples are located in the `examples/` directory.

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

#include "espFoC/inverter_6pwm_mcpwm.h"
#include "espFoC/current_sensor_adc.h"
#include "espFoC/current_sensor_adc_one_shot.h"
#include "espFoC/rotor_sensor_as5600.h"
#include "espFoC/esp_foc.h"

static const char *TAG = "esp-foc-example";

static esp_foc_inverter_t *inverter;
static esp_foc_isensor_t  *shunts;
static esp_foc_rotor_sensor_t *sensor;

static esp_foc_axis_t axis;
static esp_foc_motor_control_settings_t settings = {
    /*Motor part number: QBL4208 - 64 - 013 */
    .motor_pole_pairs = 4,
    .motor_inductance = 0.0018f,
    .motor_resistance = 1.08f,
    .natural_direction = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW,
};

static void regulation_callback (esp_foc_axis_t *axis, esp_foc_d_current  *id_ref, esp_foc_q_current *iq_ref,
                                                    esp_foc_d_voltage *ud_forward, esp_foc_q_voltage *uq_forward)
{

    /* Actual motor drive is done inside of ythe user defined callback */
    float vq_base = 0.0f;
    float vd_base = 0.0f;

    float iq_base = 2.0f;
    float id_base = 0.0f;

    uq_forward->raw = vq_base;
    ud_forward->raw = vd_base;
    iq_ref->raw = iq_base;
    id_ref->raw = id_base;

}

static void initialize_foc_drivers(void)
{

    inverter = inverter_6pwm_mpcwm_new(
        CONFIG_FOC_PWM_U_PIN,
        CONFIG_FOC_PWM_UL_PIN,
        CONFIG_FOC_PWM_V_PIN,
        CONFIG_FOC_PWM_VL_PIN,
        CONFIG_FOC_PWM_W_PIN,
        CONFIG_FOC_PWM_WL_PIN,
        -47,
        12.0f,
        0
    );
    if(inverter == NULL) {
        ESP_LOGE(TAG, "failed to create the inverter driver, aborting!");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
        abort();
    }

#ifdef CONFIG_IDF_TARGET_ESP32P4
    esp_foc_isensor_adc_oneshot_config_t shunt_oneshot_cfg = {
        .axis_channels = {ADC_CHANNEL_7, ADC_CHANNEL_6},
        .units = {ADC_UNIT_1, ADC_UNIT_1},
        .amp_gain = 20.0f,
        .shunt_resistance = 0.005f,
        .number_of_channels = 2,
        .enable_analog_encoder = false,
    };

    shunts = isensor_adc_oneshot_new(&shunt_oneshot_cfg, NULL);
    if(shunts == NULL) {
        ESP_LOGE(TAG, "failed to create the shunt sensor driver, aborting!");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
        abort();
    }
#elif defined (CONFIG_IDF_TARGET_ESP32S3)
    esp_foc_isensor_adc_config_t shunt_cfg = {
        .axis_channels = {ADC_CHANNEL_1, ADC_CHANNEL_5},
        .units = {ADC_UNIT_1, ADC_UNIT_1},
        .amp_gain = 50.0f,
        .shunt_resistance = 0.01f,
        .number_of_channels = 2,
    };

    shunts = isensor_adc_new(&shunt_cfg);
    if(shunts == NULL) {
        ESP_LOGE(TAG, "failed to create the shunt sensor driver, aborting!");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
        abort();
    }

#endif
    sensor = rotor_sensor_as5600_new(
        CONFIG_FOC_ENCODER_SDA_PIN,
        CONFIG_FOC_ENCODER_SCL_PIN,
        0
    );

    if(sensor == NULL) {
        ESP_LOGE(TAG, "failed to create as5600 encoder driver");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
        abort();
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing the esp foc motor controller!");

    initialize_foc_drivers();

    esp_foc_initialize_axis(
        &axis,
        inverter,
        sensor,
        shunts,
        settings
    );

    /* Add some virtual scope channels to monitor the currents */
#ifdef CONFIG_ESP_FOC_SCOPE
    esp_foc_scope_add_channel(&axis.i_u, 1);
    esp_foc_scope_add_channel(&axis.i_v, 2);
    esp_foc_scope_add_channel(&axis.i_w, 3);
#endif

    esp_foc_align_axis(&axis);
    esp_foc_run(&axis);
    esp_foc_set_regulation_callback(&axis, regulation_callback);
}
```

## 📁 Repository Structure

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

## 🚧 Project Status

espFoC is experimental and personal project  so it is subject to breaking..

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