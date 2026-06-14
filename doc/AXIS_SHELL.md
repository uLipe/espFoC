# axis_shell example

Reference firmware: **console shell** on the monitor UART.

## menuconfig

```text
espFoC axis_shell
  Motor plant          → FOC-in-the-loop (default) | Hardware drivers
  Motor                → pole pairs, DC link [V]
  (FITL)               → plant R/L/J under espFoC Settings → FOC-in-the-loop
                         (defaults: QBL4208-61-04-013, delta, 24 V)
  (HW) Inverter        → MCPWM 6-PWM GPIOs + enable + ADC shunt channels
  (HW) Encoder         → AS5600 I2C | open-loop simulator

espFoC Settings
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

QEMU smoke test (FITL + shell):

```bash
cd examples/axis_shell
idf.py set-target esp32
idf.py build
python3 ../../scripts/run_shell_qemu.py
```

## Hardware bring-up

Set **Motor plant → Hardware drivers** in menuconfig, fill inverter/encoder/ADC pins, then same shell flow as FITL.
