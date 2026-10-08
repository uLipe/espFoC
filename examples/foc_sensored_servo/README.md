# foc_sensored_servo

A position servo: field-oriented control of a PMSM/BLDC motor with an AS5600
magnetic encoder on the shaft, commanded in shaft degrees from the console.

The example starts from nothing but the motor's pole pairs and the supply
voltage. On every boot it commissions the motor:

1. **Phase discovery** finds which bridge output drives which motor phase,
   zeroes the encoder on the rotor's magnetic axis and checks which way it
   counts. The motor leads can go in any order.
2. **Motor identification** measures phase resistance, inductance, flux
   linkage, acceleration per amp, and the encoder offset and delay. The
   controller gains are designed from these numbers.
3. **Cogging learn**: slow sweeps both ways map the torque the magnets pull at
   every shaft angle. The controller cancels it from then on; without it the
   shaft settles into the nearest magnetic detent instead of the requested
   angle.

The angle the shaft rests at just before the cogging learn becomes the
origin (0 deg). After the learn the servo moves back there, holds it and waits
for commands on the console:

| Command                          | What it does                                       |
|----------------------------------|----------------------------------------------------|
| `move <deg>`                     | go to an absolute shaft angle                      |
| `traj <deg>:<s> [<deg>:<s> ...]` | pass through waypoints, each reached after the given seconds; the same angle twice is a dwell |
| `where`                          | print the shaft angle                              |
| `stop`                           | stop the controller, bridge off                    |

Every move is a minimum-jerk profile: position, speed and acceleration are
smooth and start and end at rest. A `move` takes at least 2.5 s and is
stretched so the profile never goes faster than 90 deg/s at its peak; a
`traj` waypoint that would go faster is rejected. Each command answers with

```text
at <deg> deg, moving! ...
stopped at <deg> deg (requested <deg> deg, error <deg> deg)
```

## Control loop

![Sensored servo control loop](images/ctrl_sensored_servo.png)

Three nested loops (position, speed, current), each fed with what the
trajectory already knows, so the feedback only corrects what the profile did
not predict.

- **Minimum-jerk trajectory**: the example task samples the profile every
  20 ms and hands the controller one joint sample with
  `esp_foc_sensored_set_joint()`: the position `θ*`, the speed feedforward
  `ω_ff` and the torque current feedforward `i_q,ff = α*·pp / K`, where `α*`
  is the profile acceleration and `K` the identified acceleration per amp.
- **Position P**: the position error `θ* − θ_m` times `K_p,pos` (in 1/s) is a
  speed correction. Added to `ω_ff` it becomes the speed reference `ω*`.
- **Speed PI**: compares `ω*` with the PLL speed `ω_e` and outputs torque
  current.
- **Cogging table**: the torque current learned at every shaft angle during
  commissioning, looked up at `θ_m` and added so the magnets' pull is
  cancelled.
- **Σ i_q\***: speed PI output + `i_q,ff` + cogging current is the torque
  current reference for the current loop.
- The position, speed and cogging blocks run in a slot of the PWM interrupt,
  once per fresh encoder sample (2.5 kHz), on the same angle the current loop
  just used.
- **Clarke / Park**: the ADC samples `i_U`, `i_V` at the PWM counter zero
  (TEZ), triggered by hardware (ETM). Clarke gives the stator-frame `i_αβ`,
  Park rotates them by `θ_e` into `i_d` (along the magnet flux) and `i_q`
  (torque).
- **Current PI d / q**: turn the current errors into `v_d`, `v_q`, with gains
  designed from the identified resistance and inductance. `i_d* = 0`.
- **Voltage limit**: clamps `|v_dq| ≤ V_dc/√3` and tells the PIs, so they do
  not wind up.
- **Inverse Park / SVM / MCPWM**: rotate `v_dq` back into `v_αβ` (with the
  angle advanced by the PWM delay), turn it into three center-aligned duty
  cycles and drive the bridge with six complementary PWM outputs and dead
  time.
- **Angle path**: the supervisor task reads the AS5600 over I²C at 2.5 kHz and
  updates a PLL, which gives the electrical speed `ω_e`. It also unwraps each
  reading into the multi-turn shaft angle `θ_m` used by the position loop and
  the cogging table. Between reads the encoder driver extrapolates its angle
  every PWM period (sensor step). Park takes `θ_e + offset + ω_e·lead`: the
  electrical angle `θ_e = pp·θ_m`, the identified encoder offset and a lead
  that cancels the encoder delay.

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
- Pins, dead time, shunt, gain, pole pairs, DC link, overspeed limit
  (600 rpm), move speed and time limits are in `idf.py menuconfig` under
  **espFoC example: sensored servo**.

### Build, flash and monitor

With ESP-IDF v5.5 installed:

```bash
. $IDF_PATH/export.sh
cd examples/foc_sensored_servo
idf.py menuconfig                  # optional: pins, motor, move limits
idf.py -p /dev/ttyUSB0 flash monitor
```

The target (`esp32c6`) comes from `sdkconfig.defaults`. Replace
`/dev/ttyUSB0` with the board's serial port. Keep the shaft free and keep
hands off: commissioning spins the motor and takes 3 to 4 minutes. When the
command list shows up, type commands in the monitor (`move 90`, `where`, ...).
Leave the monitor with `Ctrl+]`. The servo keeps holding position with the
bridge on until it gets `stop`.

### Driving it with servo.py

`servo.py`, next to this file, sends the same commands and generates
trajectories as waypoint lists. It needs pyserial, which ships with ESP-IDF's
Python environment. Close `idf.py monitor` first: only one program can hold
the port.

```bash
python servo.py -p /dev/ttyUSB0 wait --reset          # reboot, print the boot log until ready
python servo.py -p /dev/ttyUSB0 move 90               # go to 90 deg
python servo.py -p /dev/ttyUSB0 traj 90:2 -45:3 0:2   # waypoints <deg>:<seconds>
python servo.py -p /dev/ttyUSB0 sine -a 45 -T 4 -n 2  # two 4 s cycles of +-45 deg
python servo.py -p /dev/ttyUSB0 steps 30 -30 0 --move-time 1.5 --dwell 0.5
python servo.py -p /dev/ttyUSB0 where                 # current angle
python servo.py -p /dev/ttyUSB0 stop                  # stop the servo, bridge off
python servo.py -p /dev/ttyUSB0 shell                 # type commands interactively
```

The port defaults to `$ESPPORT`, or `/dev/ttyUSB0` when it is not set; `python servo.py -h` lists every option.

### Expected output

From an ESP32-C6 run. The identified values change slightly from boot to
boot, and the phase map depends on how the motor is wired.

```text
          |_|   sensored servo example
  ...
  Phase discovery: finding the phase order and zeroing the encoder...

  Phase map
  +-------------+---------------+--------------+
  | Motor phase | Bridge output | Current sign |
  +-------------+---------------+--------------+
  | U           | V             |           -1 |
  | V           | U             |           -1 |
  | W           | W             |           -1 |
  +-------------+---------------+--------------+
  Attempts 1, encoder zeroed: yes, encoder direction: as mounted

  Motor identification: about 90 s, the shaft will spin...

  Motor
  +----------------------------+-----------------+------------+
  | Parameter                  | Value           | Source     |
  +----------------------------+-----------------+------------+
  | Pole pairs                 | 13              | Kconfig    |
  | DC link                    | 12.00 V         | Kconfig    |
  | Phase resistance           | 3.808 ohm       | identified |
  | Phase inductance           | 481 uH          | identified |
  | Flux linkage               | 1399 uWb        | identified |
  | Acceleration per amp       | 83145 rad/s2/A  | identified |
  | Rotor inertia              | 4.27e-06 kg m2  | identified |
  | Encoder angle offset       | -10.19 deg el   | identified |
  | Encoder delay              | 193.2 us        | identified |
  +----------------------------+-----------------+------------+

  Controller (designed from the identified motor)
  +----------------------------+-----------------+
  | Current loop Kp            | 0.0247          |
  | Current loop Ki            | 193.3           |
  | Speed noise at rest        | 5.007 rad/s el  |
  | Speed loop bandwidth       | 7.8 Hz          |
  | Speed feedback filter      | 18.0 Hz         |
  | Speed loop Kp [A/(rad/s)]  | 1.36e-03        |
  | Speed loop Ki              | 2.91e-02        |
  | Position loop Kp           | 28.27 1/s       |
  | Move peak speed            | 90 deg/s        |
  +----------------------------+-----------------+

  Cogging learn: the shaft sweeps slowly both ways, up to 2 min...

  Cogging
  +----------------------------+-----------------+
  | Passes                     | 3               |
  | Converged                  | yes             |
  | Last pass change (RMS)     | 8.1 mA          |
  | Cogging torque, peak-peak  | 360.1 mA        |
  | Angle bins mapped          | 1434 / 1440     |
  +----------------------------+-----------------+

at -0.26 deg, moving! to 0.00 deg in 2.50 s
stopped at 0.00 deg (requested 0.00 deg, error +0.00 deg)

  Servo ready. Angles in degrees from the origin. Commands:
    move <deg>
    traj <deg>:<s> [<deg>:<s> ...]
    where
    stop
```

`Speed feedback filter` prints the tuning field `speed_fc_hz`, the speed
loop's crossover `2·ζ·bandwidth`. There is no separate speed filter; the PLL
bandwidth does that job.

Then a `servo.py` session against the same boot:

```text
$ servo.py move 90
at 0.00 deg, moving! to 90.00 deg in 2.50 s
stopped at 89.74 deg (requested 90.00 deg, error -0.26 deg)
$ servo.py move -90
at 89.91 deg, moving! to -90.00 deg in 3.75 s
stopped at -89.91 deg (requested -90.00 deg, error +0.09 deg)
$ servo.py move 37.5
at -89.91 deg, moving! to 37.50 deg in 2.66 s
stopped at 38.06 deg (requested 37.50 deg, error +0.56 deg)
$ servo.py traj 90:2 -45:3 0:2
at 37.97 deg, moving! through 3 waypoints to 0.00 deg in 7.00 s
stopped at -0.26 deg (requested 0.00 deg, error -0.26 deg)
$ servo.py sine -a 45 -T 4 -n 2
at 0.00 deg, moving! through 5 waypoints to 0.00 deg in 8.00 s
stopped at -0.26 deg (requested 0.00 deg, error -0.26 deg)
$ servo.py traj 180:0.5
rejected: waypoint 1 (180.00 deg in 0.50 s) peaks at 675 deg/s, limit 90 deg/s
$ servo.py stop
Done: servo stopped, bridge off.
```

A move that does not settle within 5 s ends with `, not in position yet`. If
the controller trips, the example prints
`Controller tripped: abort '<reason>', fault '<reason>'. Bridge is off.` and
stops taking commands.
