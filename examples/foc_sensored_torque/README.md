# foc_sensored_torque

Field-oriented torque control of a PMSM/BLDC motor with an AS5600 magnetic
encoder on the shaft.

The example starts from nothing but the motor's pole pairs and the supply
voltage. On every boot it commissions the motor:

1. **Phase discovery** finds which bridge output drives which motor phase,
   zeroes the encoder on the rotor's magnetic axis and checks which way it
   counts. The motor leads can go in any order.
2. **Motor identification** measures phase resistance, inductance, flux
   linkage, acceleration per amp, and the encoder offset and delay. The
   current loop gains are designed from these numbers.

Then it runs a torque cycle in one direction, forever or for a set number of
cycles:

| Phase        | What the controller does                          |
|--------------|---------------------------------------------------|
| accelerating | q-axis current ramps from 0 to 0.45 A in 1 s      |
| cruising     | 0.45 A is held for 2 s                            |
| decelerating | the current ramps back to 0 in 1 s                |
| stopped      | zero current for 1 s, the shaft coasts            |

In torque mode the controller regulates current, not speed. With a free shaft
the motor keeps speeding up during cruise until the supply voltage cannot push
more current, and it slows down only as fast as friction lets it.

## Control loop

![Sensored torque control loop](images/ctrl_sensored_torque.png)

This is the textbook FoC current loop. The application only sets the torque
current reference; everything else runs inside the controller.

- **Torque ramp (app)**: the example task writes `i_q*` (torque current) with
  `esp_foc_sensored_set_iq()`. `i_d* = 0`: no field weakening, all the current
  makes torque.
- **Clarke**: the ADC samples two phase currents `i_U`, `i_V` at the PWM
  counter zero (TEZ), triggered by hardware (ETM). Clarke turns the three
  phase currents (the third is `-(i_U + i_V)`) into the two-axis stator frame
  `i_αβ`.
- **Park**: rotates `i_αβ` by the electrical angle `θ_e` into the rotor frame.
  `i_d` is the current along the magnet flux, `i_q` the current that makes
  torque. In this frame both are DC at steady state, so a PI can regulate them.
- **Current PI d / q**: one PI per axis turns the current errors into voltages
  `v_d`, `v_q`. Their gains are designed from the identified resistance and
  inductance.
- **Voltage limit**: clamps the voltage vector to what the bridge can make,
  `|v_dq| ≤ V_dc/√3`, and tells the PIs so they do not wind up.
- **Inverse Park**: rotates `v_dq` back into the stator frame `v_αβ`, using the
  angle advanced by the PWM delay so the voltage lands where the rotor will be.
- **SVM**: space-vector modulation turns `v_αβ` into three center-aligned duty
  cycles for the MCPWM.
- **MCPWM + inverter**: six complementary PWM outputs with dead time drive the
  bridge, which applies `v_UVW` to the motor windings.
- **Angle path**: the supervisor task reads the AS5600 over I²C at 2.5 kHz and
  updates a PLL with each sample; the PLL gives the electrical speed `ω_e`.
  Between reads the encoder driver extrapolates its angle every PWM period
  (sensor step), so Park never uses a stale angle. Park takes
  `θ_e + offset + ω_e·lead`: the electrical angle `θ_e = pp·θ_m`, the
  identified encoder offset and a lead that cancels the encoder delay.

Everything inside the dashed **TEZ ISR** box runs in one interrupt per PWM
period (20 kHz), in Q16.16 fixed point.

## Getting started

### Board interconnection

![Sensored board interconnection](images/board_sensored.png)

| Signal                         | ESP32-C6 GPIO | Direction        |
|--------------------------------|---------------|------------------|
| PWM phase U high / low side    | 18 / 19       | ESP32 → inverter |
| PWM phase V high / low side    | 20 / 21       | ESP32 → inverter |
| PWM phase W high / low side    | 22 / 23       | ESP32 → inverter |
| Bridge enable (active low)     | 15            | ESP32 → inverter |
| Current sense U (ADC)          | 5             | inverter → ESP32 |
| Current sense V (ADC)          | 4             | inverter → ESP32 |
| Current sense W                | not wired     | (two shunts)     |
| AS5600 SDA (I²C, 400 kHz)      | 2             | bidirectional    |
| AS5600 SCL                     | 3             | ESP32 → encoder  |
| GND                            | GND           | common to all    |

- The inverter takes a 12 V DC supply. The default current sense is a 10 mΩ
  shunt with a gain of 20, and the bridge trips at 4 A.
- The motor phases go to the bridge outputs in any order; phase discovery
  finds the map.
- The AS5600 sits on the shaft end, over a diametral magnet, powered at 3.3 V.
- Pins, dead time, shunt, gain, pole pairs, DC link and the cycle are in
  `idf.py menuconfig` under **espFoC example: sensored torque**.

### Build, flash and monitor

With ESP-IDF v5.5 installed:

```bash
. $IDF_PATH/export.sh
cd examples/foc_sensored_torque
idf.py menuconfig                  # optional: pins, motor, cycle
idf.py -p /dev/ttyUSB0 flash monitor
```

The target (`esp32c6`) comes from `sdkconfig.defaults`. Replace
`/dev/ttyUSB0` with the board's serial port. Leave the monitor with `Ctrl+]`.

Keep the shaft free and keep hands off: commissioning spins the motor and
identification takes about 90 s. The cycle runs forever by default; set
**Cycles to run** (`EXAMPLE_CYCLE_COUNT`) to stop after a few and turn the
bridge off.

### Expected output

From an ESP32-C6 run with `EXAMPLE_CYCLE_COUNT=2`. The identified values
change slightly from boot to boot, and the phase map depends on how the motor
is wired.

```text
          |_|   sensored torque example

  Board
  +----------------------------+-----------------------+
  | Function                   | Pin / value           |
  +----------------------------+-----------------------+
  | PWM phase U high side      | GPIO 18               |
  ...
  | Encoder sample rate        | 2500 Hz               |
  +----------------------------+-----------------------+

  Phase discovery: finding the phase order and zeroing the encoder...

  Phase map
  +-------------+---------------+--------------+
  | Motor phase | Bridge output | Current sign |
  +-------------+---------------+--------------+
  | U           | W             |           -1 |
  | V           | U             |           -1 |
  | W           | V             |           -1 |
  +-------------+---------------+--------------+
  Attempts 1, encoder zeroed: yes, encoder direction: reversed (corrected)

  Motor identification: about 90 s, the shaft will spin...

  Motor
  +----------------------------+-----------------+------------+
  | Parameter                  | Value           | Source     |
  +----------------------------+-----------------+------------+
  | Pole pairs                 | 13              | Kconfig    |
  | DC link                    | 12.00 V         | Kconfig    |
  | Phase resistance           | 2.841 ohm       | identified |
  | Phase inductance           | 409 uH          | identified |
  | Flux linkage               | 1519 uWb        | identified |
  | Acceleration per amp       | 83422 rad/s2/A  | identified |
  | Rotor inertia              | 4.62e-06 kg m2  | identified |
  | Encoder angle offset       | -3.77 deg el    | identified |
  | Encoder delay              | 201.2 us        | identified |
  +----------------------------+-----------------+------------+

  Controller (designed from the identified motor)
  +----------------------------+-----------------+
  | Current loop Kp            | 0.0210          |
  | Current loop Ki            | 144.2           |
  | Cruise torque current      | 0.45 A          |
  +----------------------------+-----------------+

  cycle 1    accelerating       0 rpm
  cycle 1    cruising        2400 rpm
  cycle 1    decelerating    2594 rpm
  cycle 1    stopped          141 rpm
  cycle 2    accelerating       5 rpm
  cycle 2    cruising        2482 rpm
  cycle 2    decelerating    2589 rpm
  cycle 2    stopped          126 rpm
  Done: 2 cycles, bridge off.
```

Each cycle line prints the shaft speed when that phase starts. If something
goes wrong the example says so and stops:

- `Encoder setup failed (check SDA/SCL and the AS5600 supply)`: the AS5600
  does not answer on I²C.
- `Phase discovery failed` / `Motor identification failed at '<step>'`:
  commissioning could not finish; check the motor leads, the supply and that
  the shaft is free.
- `Controller tripped: abort '<reason>', fault '<reason>'. Bridge is off.`:
  a controller guard fired while running and turned the bridge off.
