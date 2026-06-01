# espFoC Tool (host)

Desktop control GUI and CLI for espFoC targets over the link/tuner protocol.

## Install

```bash
pip install -r tools/espfoc_tool/requirements.txt
```

## GUI

```bash
PYTHONPATH=tools python3 -m espfoc_tool.gui
```

Options: `--port` (skip USB scan), `--baud`, `--axis`, `--no-gl`, `--scope-csv`.

## CLI

```bash
PYTHONPATH=tools python3 -m espfoc_tool.cli.espfocctl --port /dev/ttyACM0 -i
```

## Layout

- `link/` — framing, serial transport, scope decode
- `protocol/` — `TunerClient` (shared by GUI and CLI)
- `client/` — stable import alias `EspFocClient`
- `model/` — MPZ / Bode analysis (numpy)
- `gui/` — espFoC Tool views
- `cli/espfocctl.py` — scripted control

## Tests

```bash
QT_QPA_PLATFORM=offscreen ESPFOC_TOOL_NO_GL=1 PYTHONPATH=tools \
  python3 tools/espfoc_tool/tests/test_gui_smoke.py
pytest tools/espfoc_tool/tests/
```

`FakeTunerLoopback` is for unit tests only (not exposed in the GUI).
