# espFoC Tool (host)

Desktop control GUI and CLI for espFoC targets over the link/tuner protocol.

**Documentation:** [`doc/TUNING.md`](../../doc/TUNING.md)

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
│   ├── views/    # Config, Current wrappers
│   ├── app.py    # Qt + OpenGL bootstrap
│   └── ...
└── cli/espfocctl.py
```

## Tests

```bash
QT_QPA_PLATFORM=offscreen ESPFOC_TOOL_NO_GL=1 PYTHONPATH=tools \
  python3 -m pytest tools/espfoc_tool/tests/ -v
```

`FakeTunerLoopback` is unit-test only — not used by the GUI.
