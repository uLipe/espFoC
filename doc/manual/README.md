# espFoC user manual

espFoC is an ESP-IDF component that spins PMSM and BLDC motors with field-oriented control
(FoC) on ESP32 chips that have MCPWM, ETM and ADC with DMA. The [top README](../../README.md)
shows what it does and how to get a first motor turning. This manual is the detailed
reference: it explains how each subsystem works, how to use its API, and when you would
pick it.

## Who this manual is for

You do not need a background in motor control. The first chapter introduces FoC with as
little math as possible, and every later chapter defines the terms it uses. If you already
know FoC, start at chapter 2.

You do need to be comfortable with C and with building an ESP-IDF project (`idf.py build`,
`idf.py menuconfig`, `idf.py flash monitor`).

## How the manual is organised

Two chapters of concepts come first. After them there is one chapter per block of the
component architecture, starting from the hardware and moving up to the control stacks:

![espFoC architecture](../diagram/architecture.png)

| Block in the diagram | Chapter |
|---|---|
| (concepts) | [1. FoC in plain words](01_foc_theory.md), [2. How espFoC runs](02_execution_model.md) |
| Inverter driver | [3. Inverter driver](03_inverter_driver.md) |
| Rotor sensor drivers | [4. Rotor sensors](04_rotor_sensors.md) |
| Phase map discovery | [5. Phase map discovery](05_phase_map_discovery.md) |
| Motor identification | [6. Motor identification](06_motor_identification.md) |
| Sensored stack | [7. Sensored stack](07_sensored_stack.md) |
| Sensorless stack | [8. Sensorless stack](08_sensorless_stack.md) |
| Building blocks | [9. Building blocks](09_building_blocks.md) |
| OSAL, Trace | [10. OS abstraction and trace](10_osal_and_trace.md) |

Most subsystem chapters have the same sections: *What it does*, *How it works*, *How to use
it*, *Configuration*, *Use cases* and *Limits and pitfalls*, so you can jump straight to the
part you need.

## Contents

1. [Field-oriented control in plain words](01_foc_theory.md)
   The motor, the inverter, Clarke and Park, PI loops, SVM, where the angle comes from, and
   cascaded torque, speed and position loops.
2. [How espFoC runs](02_execution_model.md)
   The PWM period as the clock, the three execution contexts, fixed-point math, units,
   memory and events.
3. [Inverter driver](03_inverter_driver.md)
   The three-phase bridge: PWM with dead time, current sensing started by hardware, faults
   and the control callback.
4. [Rotor sensors](04_rotor_sensors.md)
   The common sensor interface, the AS5600 magnetic encoder, Hall sensors and the
   sensorless adapter.
5. [Phase map discovery](05_phase_map_discovery.md)
   Finding automatically which wire is which, the current sense signs and the sensor zero
   and direction.
6. [Motor identification](06_motor_identification.md)
   Measuring resistance, inductance, flux, mechanical constants and sensor timing at
   power-up.
7. [Sensored stack](07_sensored_stack.md)
   Torque, velocity and position control with a rotor sensor, cogging compensation and
   guards.
8. [Sensorless stack](08_sensorless_stack.md)
   Torque and velocity control without a sensor: start-up sequence, flux observer, events
   and guards.
9. [Building blocks](09_building_blocks.md)
   The fixed-point math, transforms, PI, SVM, filters, PLL and observers the stacks are made
   of, usable on their own.
10. [OS abstraction and trace](10_osal_and_trace.md)
    The portability layer between espFoC and the RTOS, and the hot-path trace for debugging.

## Reading paths

- **New to motor control**: read chapters 1 and 2, then follow the
  [sensored torque example](../../examples/foc_sensored_torque/README.md) with chapters 3 to 7
  at hand.
- **Building a fan, pump or propeller drive without a sensor**: chapters 2, 3, 5, 6 and 8,
  and the [sensorless velocity example](../../examples/foc_sensorless_velocity/README.md).
- **Building a servo or a robot joint**: chapters 2 to 7, and the
  [servo example](../../examples/foc_sensored_servo/README.md).
- **Writing your own controller on top of the drivers**: chapters 2, 3, 4, 9 and 10.

## Conventions

- Code, identifiers, Kconfig symbols and log lines are shown as they are in the source, in
  `monospace`. Kconfig symbols are written with their `CONFIG_` prefix.
- The public API takes and returns `float` values in SI units (amps, volts, ohms, henries,
  radians, seconds) and electrical Hz for most speed references. Each function's units are
  given where it is introduced. Internally, everything on the control path is fixed point;
  see [How espFoC runs](02_execution_model.md).
- "Electrical" and "mechanical" angles and speeds differ by the number of pole pairs. The
  manual always says which one it means.
- The headers under [`include/espFoC/`](../../include/espFoC/) are the final reference.
  When this manual and a header disagree, the header wins.

## Examples

Five complete applications ship with the component. Each has its own README with a control
diagram, the board connection and the expected console output:

| Example | What it shows |
|---|---|
| [foc_sensored_torque](../../examples/foc_sensored_torque/README.md) | Commissioning plus torque control with an AS5600 |
| [foc_sensored_velocity](../../examples/foc_sensored_velocity/README.md) | Speed control with an AS5600 |
| [foc_sensored_servo](../../examples/foc_sensored_servo/README.md) | Position servo with trajectories, driven from a PC script |
| [foc_sensorless_torque](../../examples/foc_sensorless_torque/README.md) | Sensorless start-up and torque control |
| [foc_sensorless_velocity](../../examples/foc_sensorless_velocity/README.md) | Sensorless speed control |
