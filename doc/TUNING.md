# espFoC Tool — tuning and host protocol

This document describes how to control an espFoC axis from the host using
**espFoC Tool** (GUI) or **espfocctl** (CLI). Both use the same binary link
layer and tuner protocol implemented in firmware (`esp_foc_link`, `esp_foc_tuner`).

---

## Prerequisites

```bash
pip install -r tools/espfoc_tool/requirements.txt
export PYTHONPATH=tools   # or prefix every command with PYTHONPATH=tools
```

Reference firmware: [`examples/axis_tuning`](../examples/axis_tuning). It advertises
firmware type **`TSGX`** so auto-scan can recognise the board.

Enable in your own project:

- `CONFIG_ESP_FOC_TUNER_ENABLE=y`
- `CONFIG_ESP_FOC_BRIDGE_UART` or `CONFIG_ESP_FOC_BRIDGE_USBCDC`
- `CONFIG_ESP_FOC_SCOPE=y` (for States / Control plots)

---

## Quick start (GUI)

```bash
python3 -m espfoc_tool.gui
# optional fixed port (skips USB scan):
python3 -m espfoc_tool.gui --port /dev/ttyACM0 --baud 921600
```

1. Wait for **CONNECTED** (or plug the board — scan runs every 2 s).
2. **Control** → **Align axis** (rotor direction + encoder zero).
3. **Control** → enable **Override active** (starts `RUN`).
4. Set **iq** / **id** with spinboxes or nudge buttons.
5. **Config** → tune Kp/Ki, **Write dirty fields**, then **Store to NVS (RMW)**.
6. **E-STOP** or disable override → `stop` (inverter disabled, targets zeroed).

The GUI works **offline**: all views are navigable without a board; device
actions stay disabled until connected.

---

## Views

### Config

| Area | Purpose |
|------|---------|
| Left | Live gains, manual Kp/Ki/ILim editor, I-LPF cutoff, axis badge, firmware log |
| Right | **Live vs editor** diff; **Write dirty fields**; **Store to NVS** (firmware RMW); **Erase NVS** |

NVS **store** only writes tuning fields that changed relative to the blob
(`esp_foc_calibration_axis_tuner_store` on device). Align data in NVS is not
edited from the tool in this release.

### Current design

| Area | Purpose |
|------|---------|
| Left | Motor **R**, **L**, **bandwidth** for MPZ synthesis |
| Right | Predicted step, Bode, pole-zero, root locus |
| Actions | **Apply MPZ to RAM** writes computed Kp/Ki/ILim to the device |

Pole pairs can be pushed to the target from this view (persist with Config → Store).

### Control

| Area | Purpose |
|------|---------|
| Left | Override, id/iq spinboxes + nudge, align, **E-STOP** |
| Right | SVPWM hexagon + phase waveforms (scope ch 10–12 on `axis_tuning`) |

**E-STOP** sequence: id/iq → 0, `stop` axis (also calls `inverter.disable` via
`park_inverter_safe`).

### States

Rolling plots for all scope channels wired in `axis_tuning` (see table below).
Requires scope streaming (`scope_start` on connect in the GUI).

---

## Scope channel map (`axis_tuning`)

| Ch | Signal |
|----|--------|
| 0 | id target |
| 1 | id measured |
| 2 | iq target |
| 3 | iq measured |
| 4 | ud |
| 5 | uq |
| 6 | θ_meas mech |
| 7 | θ_est mech |
| 8 | ω_est mech |
| 9 | PLL error |
| 10 | iu |
| 11 | iv |
| 12 | iα |
| 13 | FOC hot-path µs |

Other examples may use different maps; only `axis_tuning` is the contract for espFoC Tool.

---

## espfocctl (CLI)

Interactive:

```bash
python3 -m espfoc_tool.cli.espfocctl --port /dev/ttyACM0 -i
```

One-shot:

```bash
python3 -m espfoc_tool.cli.espfocctl --port /dev/ttyACM0 align
python3 -m espfoc_tool.cli.espfocctl --port /dev/ttyACM0 estop
```

| Command | Description |
|---------|-------------|
| `connect` / `disconnect` | Link session |
| `status` | Heartbeat + axis state flags |
| `read` | Kp, Ki, Kd, Kff, ILim, Vmax |
| `write --kp … --ki …` | Write gains |
| `align` | Rotor alignment |
| `run` / `stop` | Start / stop FOC loop |
| `set-target id\|iq VALUE` | Current references (A) |
| `store` / `erase` | NVS calibration |
| `cutoff [--set HZ]` | Current LPF |
| `scope-start` / `scope-stop` | Scope stream |
| `firmware-type` | FourCC (expect `TSGX` on axis_tuning) |
| `estop` | id/iq=0 + stop |

---

## Environment variables

| Variable | Effect |
|----------|--------|
| `ESPFOC_TOOL_NO_GL=1` | Disable OpenGL plot rendering |
| `ESPFOC_TOOL_SCOPE_CSV=1` | Decode legacy CSV scope (if firmware built with legacy CSV) |
| `QT_QPA_PLATFORM=offscreen` | Headless CI / smoke tests |

---

## Host tests

```bash
QT_QPA_PLATFORM=offscreen ESPFOC_TOOL_NO_GL=1 PYTHONPATH=tools \
  python3 -m pytest tools/espfoc_tool/tests/ -v
```

`FakeTunerLoopback` is for unit tests only — not exposed in the GUI.

---

## Troubleshooting

| Symptom | Check |
|---------|--------|
| Stuck on SCANNING | Cable, driver, correct port; firmware must expose bridge + `TSGX` |
| NO LINK after connect | Heartbeat / baud (default 921600); another app holding the port |
| Align fails | Motor wiring, pole pairs, sensor; see serial log in Config |
| States flat | `CONFIG_ESP_FOC_SCOPE`, scope started, PWM loop running |
| Plots slow on VM | `ESPFOC_TOOL_NO_GL=1` or smaller window |

---

## FITL builds

Firmware built with `CONFIG_ESP_FOC_FITL` simulates plant + sensors in software.
espFoC Tool treats it like a normal target (`TSGX`); use **axis_tuning** without
FITL when validating real hardware.
