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
- `CONFIG_ESP_FOC_SCOPE=y` (for Dashboard scope plots)

---

## Quick start (GUI)

```bash
python3 -m espfoc_tool.gui
# optional fixed port (skips USB scan):
python3 -m espfoc_tool.gui --port /dev/ttyACM0 --baud 921600
```

1. Wait for **CONNECTED** in the status bar (or plug the board — scan runs every 2 s).
2. Open **Dashboard** → **Run alignment** (rotor direction + encoder zero).
3. Enable **Manual setpoints**, set **iq** / **id** (nudge buttons optional).
4. Open **Tune** → edit Kp/Ki/lim/filter → **Write** (RAM) → **Patch** (flash).
5. **E-STOP** (Dashboard → Actions) or disable manual setpoints → axis stops.

The GUI works **offline**: both views are navigable without a board; device
actions stay disabled until connected.

---

## Views

### Tune

| Area | Purpose |
|------|---------|
| Left | Live gains, manual editor, **Apply gains** / **Apply filter**, serial log |
| Center | Device vs pending diff; flash badge (stored / empty) |
| Right | Motor **R**, **L**, **bandwidth**; MPZ step/Bode/pole-zero/root locus |

**Flash actions** (center column):

| Button | Action |
|--------|--------|
| **Read** | Load Kp, Ki, lim, filter cutoff from device RAM into the editor |
| **Write** | Push only fields that differ from live RAM |
| **Patch** | Write dirty fields, then **store calibration** to NVS (firmware RMW) |

**Apply gains** under the MPZ plots writes synthesized Kp/Ki/lim from the motor model.
Pole pairs can be changed in the plot toolbar; persist with **Patch**.

NVS **store** only writes tuning fields that changed relative to the blob
(`esp_foc_calibration_axis_tuner_store` on device). Align data in NVS is not
edited from the tool in this release.

### Dashboard

| Area | Purpose |
|------|---------|
| Left — Motion | Manual setpoints, id/iq spinboxes + nudge |
| Left — Actions | **Run alignment**, **E-STOP**, **Autoset** (SVM/scope reset) |
| Right — top | SVPWM hexagon (pu) beside three-phase waveforms (scope ch 10–12) |
| Right — bottom | Rolling plots for all scope channels (`axis_tuning` map) |

**E-STOP** sequence: id/iq → 0, `stop` axis (also calls `inverter.disable` via
`park_inverter_safe`).

Scope streaming starts automatically on connect.

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
| Align fails | Motor wiring, pole pairs, sensor; see serial log on **Tune** |
| Scope flat | `CONFIG_ESP_FOC_SCOPE`, scope started, PWM loop running |
| Plots slow on VM | `ESPFOC_TOOL_NO_GL=1` or smaller window |

---

## FITL builds

Firmware built with `CONFIG_ESP_FOC_FITL` simulates plant + sensors in software.
espFoC Tool treats it like a normal target (`TSGX`); use **axis_tuning** without
FITL when validating real hardware.
