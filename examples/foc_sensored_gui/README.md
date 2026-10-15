# foc_sensored_gui

Field-oriented control of a PMSM/BLDC motor with an AS5600 encoder, driven
from a desktop GUI. The board commissions the motor one step at a time when
you press the buttons, then follows the torque, speed or position reference
you set with a slider, while three scopes plot reference against measurement.

![The GUI driving the bench motor in speed mode](images/gui_speed.png)

The GUI is a single Python file, [`host/espfoc_gui.py`](host/espfoc_gui.py):
PySide6 with the QML inline. The scopes are drawn as Qt Quick scene-graph
geometry, so the GPU does the drawing through Qt's RHI (OpenGL, Vulkan, Metal
or Direct3D, whatever the platform offers). The file is commented top to
bottom as a starting point for your own front end.

## How the host connects to the board

```mermaid
flowchart LR
    subgraph PC["Host PC"]
        GUI["espfoc_gui.py<br/>PySide6 + QML<br/>scopes on the GPU"]
    end
    subgraph DEV["Dev board (ESP32-C6)"]
        BRIDGE["USB-UART bridge<br/>(CP210x / CH340)"]
        subgraph CHIP["ESP32-C6"]
            UART["console UART<br/>115200 8N1"]
            APP["main.c<br/>protocol, commissioning,<br/>telemetry 10 Hz"]
            FOC["espFoC sensored stack<br/>current / speed / position"]
        end
    end
    INV["3-phase inverter<br/>12 V DC link"]
    MOT["PMSM / BLDC motor<br/>+ AS5600 on the shaft"]

    GUI <-- "USB cable<br/>binary frames" --> BRIDGE
    BRIDGE <-- "TX / RX" --> UART
    UART <--> APP
    APP <--> FOC
    FOC -- "6 x PWM, enable" --> INV
    INV -- "phase currents (ADC)" --> FOC
    INV ==> MOT
    MOT -- "I2C angle" --> FOC
```

There is no extra wiring for the GUI: it uses the same USB cable and UART that
`idf.py flash monitor` uses. Only one program can hold the port, so close any
serial monitor before starting the GUI. The GUI opens the port with DTR and
RTS released, so connecting does not reset the board.

## Using it

1. **Connect**: pick the board's serial port, press *Connect*. The board
   answers with its limits (pole pairs, slider ranges).
2. **Commission**, in order; each button unlocks when the previous step is
   done, and the log under them shows what the board reports:
   1. *Phase map*: which bridge output drives which phase, current signs,
      encoder zero and direction (~4 s).
   2. *Identify*: resistance, inductance, flux, acceleration per amp, inertia,
      encoder offset and delay (~90 s, the shaft spins). The controller is
      then designed from these numbers and starts, holding position.
   3. *Cogging learn*: slow sweeps both ways map the magnets' pull at every
      shaft angle (~80 s). The controller cancels it from then on. Where the
      shaft stops becomes 0 deg.
3. **Drive**: choose *Torque* (A), *Speed* (rpm) or *Position* (deg). Set the
   slider's *min* and *max*, move the slider; *Reverse direction* flips the
   sign of the reference. Torque and speed start at 0; position starts
   where the shaft is, so changing mode never makes the shaft jump.
4. **STOP** (or *Esc*) turns the bridge off. Selecting a mode afterwards
   starts the controller again; commissioning is kept. STOP also clears an
   abort or fault (overspeed, current trip, encoder lost). Phase map and
   identification cannot be interrupted half way: STOP during them reboots
   the board, with the bridge held off across the reset.

The board clamps every reference to the ranges in `menuconfig`
(**espFoC example: sensored GUI → GUI limits**), whatever the host sends:
±2000 rpm, ±0.5 A, ±3600 deg by default. The overspeed trip (2900 rpm)
protects torque mode, where an unloaded shaft keeps speeding up.

## Getting started

### Board

Same bench as the other sensored examples:

| Signal                         | Bench GPIO    | Direction        |
|--------------------------------|---------------|------------------|
| PWM phase U high / low side    | 18 / 19       | chip → inverter  |
| PWM phase V high / low side    | 20 / 21       | chip → inverter  |
| PWM phase W high / low side    | 22 / 23       | chip → inverter  |
| Bridge enable (active low)     | 15            | chip → inverter  |
| Current sense U (ADC)          | 5             | inverter → chip  |
| Current sense V (ADC)          | 4             | inverter → chip  |
| Current sense W                | not wired     | (two shunts)     |
| AS5600 SDA (I²C, 400 kHz)      | 2             | bidirectional    |
| AS5600 SCL                     | 3             | chip → encoder   |
| GND                            | GND           | common to all    |

Pins, dead time, shunt, gain, pole pairs, DC link and the GUI limits are in
`idf.py menuconfig` under **espFoC example: sensored GUI**.

### Build and flash

With ESP-IDF v5.5 installed:

```bash
. $IDF_PATH/export.sh
cd examples/foc_sensored_gui
idf.py -p /dev/ttyUSB0 flash
```

The target comes from `sdkconfig.defaults` (`esp32c6`). For another supported
chip, run `idf.py set-target <target>` first and set its pins.

### Run the GUI

```bash
pip install PySide6 pyserial
python3 host/espfoc_gui.py --port /dev/ttyUSB0
```

`--port` is optional; without it, pick the port in the window.

## Protocol

Binary frames over the console UART, 115200 8N1:

```text
+------+------+-----+-----------------+-------+
| 0xA5 | type | len | payload[len]    | crc8  |
+------+------+-----+-----------------+-------+
                     len <= 64          over type, len, payload
```

- CRC-8: polynomial `0x07`, initial value `0`, no reflection.
- Little endian; `f32` is IEEE-754 single precision.
- A receiver skips bytes until a `0xA5` that starts a frame with a good CRC.
  This is how the GUI steps over the ROM and bootloader text at boot, which
  it shows in its log as `[boot]` lines.

### Host → board

| Type   | Name    | Payload     | Effect |
|--------|---------|-------------|--------|
| `0x01` | `PING`  | –           | board answers `INFO` |
| `0x02` | `MAP`   | –           | start phase discovery |
| `0x03` | `IDENT` | –           | start identification, then start the controller |
| `0x04` | `COG`   | –           | start cogging learn |
| `0x05` | `MODE`  | `u8` mode   | 0 torque, 1 speed, 2 position; starts the controller if it is stopped |
| `0x06` | `REF`   | `f32` value | reference in the unit of the mode: A, rpm, deg |
| `0x07` | `STOP`  | –           | bridge off (reboot during map / identify) |

Every command but `PING` is answered with an `ACK`. `MAP`, `IDENT`, `COG`
and a `MODE` that restarts the controller answer when the step ends, which
can take minutes; the others answer at once. A command out of order (for
example `IDENT` before `MAP`) is refused with `ESP_ERR_INVALID_STATE`.

### Board → host

| Type   | Name   | Payload | When |
|--------|--------|---------|------|
| `0x81` | `INFO` | `u8` pole pairs, `f32` max rpm, `f32` max iq [A], `f32` max deg | boot, `PING` |
| `0x82` | `TEL`  | see below, 35 bytes | every 100 ms |
| `0x83` | `PROG` | `u8` step (0 map, 1 identify, 2 cogging), `u8` percent (`0xFF` unknown) | during a step |
| `0x84` | `LOG`  | text, not terminated | the board's log, one line per frame (split if longer than 64) |
| `0x85` | `ACK`  | `u8` command, `i32` `esp_err_t` (0 = OK) | after a command |

`TEL` payload:

| Offset | Type  | Field |
|--------|-------|-------|
| 0      | `u8`  | steps done: bit 0 map, bit 1 identify, bit 2 cogging |
| 1      | `u8`  | busy: 0 no, 1 map, 2 identify, 3 cogging, 4 starting |
| 2      | `u8`  | controller state: 0 idle, 1 armed, 2 running, 3 learning, 4 fault |
| 3      | `u8`  | mode: 0 torque, 1 speed, 2 position |
| 4      | `u8`  | flags: bit 0 running, 1 abort, 2 fault, 3 in position, 4 cogging compensation on |
| 5      | `f32` | speed reference [rpm] |
| 9      | `f32` | speed [rpm] |
| 13     | `f32` | position reference [deg] |
| 17     | `f32` | position [deg] |
| 21     | `f32` | iq reference [A] |
| 25     | `f32` | iq [A] |
| 29     | `f32` | DC link [V] |
| 33     | –     | 2 bytes reserved |

Everything the GUI shows comes from `TEL`: it does not track the sequence
itself, so after a reconnect or a board reboot it simply shows what the board
reports.
