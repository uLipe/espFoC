# espFoC Tool (host)

Desktop GUI and CLI for espFoC targets over the link/tuner protocol.

**Full workflow:** [`doc/TUNING.md`](../../doc/TUNING.md)

## Install

```bash
pip install -r tools/espfoc_tool/requirements.txt
```

## GUI

```bash
PYTHONPATH=tools python3 -m espfoc_tool.gui
```

| Flag | Description |
|------|-------------|
| `--port` | Fixed serial port (omit to auto-scan USB) |
| `--baud` | Default 921600 |
| `--axis` | Axis index 0..3 |
| `--no-gl` | CPU plot rendering |
| `--scope-csv` | Legacy CSV scope decode |

Connection state (scanning, connected, link health) is shown in the **status bar**.
All device actions stay disabled until a board is connected; views remain navigable offline.

## Views

### Tune

Setup, flash, and MPZ preview in one screen:

| Column | Content |
|--------|---------|
| Left | Live gains, manual editor (Kp/Ki/lim/filter), firmware log |
| Center | Flash diff (device vs pending); **Read** / **Write** / **Patch** |
| Right | Motor model (R, L, bandwidth) + four MPZ plots |

- **Apply gains** (editor): push spinbox values to RAM.
- **Apply gains** (plots): write MPZ-designed Kp/Ki/lim to RAM.
- **Patch**: write dirty fields, then store calibration to flash (firmware RMW).

### Dashboard

Runtime control and scope:

| Column | Content |
|--------|---------|
| Left | Motion (manual id/iq, nudge), **Actions** (align, E-STOP, Autoset) |
| Right | SVPWM hexagon + three-phase waveforms (top), rolling scope channels (bottom) |

**Autoset** clears SVM trail/waveform history and resets the per-unit scale.

## CLI

```bash
PYTHONPATH=tools python3 -m espfoc_tool.cli.espfocctl --port /dev/ttyACM0 -i
```

## Layout

```
espfoc_tool/
├── client/       # EspFocClient alias (TunerClient)
├── link/         # framing, serial, scope decode
├── protocol/     # tuner requests
├── model/        # MPZ / Bode (numpy)
├── gui/
│   ├── views/    # tune_view, dashboard_view
│   ├── theme.py  # dark palette + button/surface styles
│   ├── widgets.py
│   └── ...
└── cli/espfocctl.py
```

## Tests

```bash
QT_QPA_PLATFORM=offscreen ESPFOC_TOOL_NO_GL=1 PYTHONPATH=tools \
  python3 -m pytest tools/espfoc_tool/tests/ -v
```

`FakeTunerLoopback` is unit-test only — not used by the GUI.
