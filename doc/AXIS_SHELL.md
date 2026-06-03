# axis_shell example

Reference firmware: **console shell** + **ESPF scope** (17 channels). No tuner/link.

## menuconfig

```text
espFoC axis_shell
  Motor plant          → FOC-in-the-loop (default) | Hardware drivers
  Motor                → pole pairs, DC link [V]
  (FITL)               → plant R/L/J under espFoC Settings → FOC-in-the-loop
                         (defaults: QBL4208-61-04-013, delta, 24 V)
  (HW) Inverter        → MCPWM 6-PWM GPIOs + enable
  (HW) Rotor sensor    → AS5600 I2C | open-loop simulator
  (HW) Current sense   → ADC unit, U/V channels, gain ×100, shunt mΩ

espFoC Settings
  Scope stream         → enable, 17 ch, bridge (USB CDC / USJ on C6)
  espfoc_shell         → console REPL
```

## FITL bring-up (default)

```bash
cd examples/axis_shell
idf.py set-target esp32c6   # or esp32 for QEMU + GPTimer tick
idf.py build flash monitor
```

Monitor UART (shell) — `idf.py flash monitor`:

```text
espFoC> align 0
espFoC> set iq 0.5 0
espFoC> run 0
```

Scope stream (espFoC Tool) — **USB Serial/JTAG** (`/dev/ttyACM*`), **not** the monitor UART:

```bash
PYTHONPATH=tools python3 -m espfoc_tool.gui --port /dev/ttyACM0
```

QEMU smoke test (FITL + shell):

```bash
cd examples/axis_shell
idf.py set-target esp32
idf.py build
python3 ../../scripts/run_shell_qemu.py
```

## Hardware build

`menuconfig` → **Motor plant** → **Hardware drivers**, then set inverter, encoder, and shunt pins.
