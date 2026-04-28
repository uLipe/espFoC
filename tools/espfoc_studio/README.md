# espfoc_studio

Host-side tooling for the espFoC tuner stack. Three layers that share the
same wire format as the firmware (see `source/motor_control/esp_foc_link.c`
and `esp_foc_tuner.c`):

- `espfoc_studio.link` — framing codec + transports (loopback, pyserial).
- `espfoc_studio.protocol` — synchronous `TunerClient` for read / write /
  exec round-trips (axis state, gains, motion targets, MPZ recompute).
- `espfoc_studio.model` — analytical helpers (MPZ design, step response,
  Bode, pole/zero, root locus). No Qt dependency.
- `espfoc_studio.cli.tunerctl` — argparse CLI on top of TunerClient.
- `espfoc_studio.gui` — PySide6 + pyqtgraph front-end (TunerStudio).

## Install

```bash
pip install -r tools/espfoc_studio/requirements.txt
```

`PySide6` and `pyqtgraph` are only needed for the GUI; the CLI and the
library layers work with just `pyserial` and `numpy`.

## Try the GUI against a simulated firmware

No hardware required — the `--demo` flag spawns an in-process
`DemoFirmware` that speaks the same protocol a real board would:

```bash
PYTHONPATH=tools python3 -m espfoc_studio.gui --demo
```

The window opens with:

- a **Tuning** panel on the left (live gains, manual edit, MPZ recompute,
  override toggle, current refs and voltage feed-forward targets);
- an **Analysis** tab that redraws the predicted step response, Bode
  magnitude, pole/zero map and root locus every time the motor or gain
  parameters change;
- a **Scope** tab that streams the simulated plant response so you can
  watch a step command converge in real time when the override is on.

## Talk to real hardware

Flash a firmware built with one of the bridges enabled
(`CONFIG_ESP_FOC_BRIDGE_UART` or `CONFIG_ESP_FOC_BRIDGE_USBCDC`), then:

```bash
# UART or USB-CDC both come out as a plain serial device.
PYTHONPATH=tools python3 -m espfoc_studio.gui --port /dev/ttyACM0
```

The CLI `tunerctl` works against the same bridges:

```bash
PYTHONPATH=tools python3 -m espfoc_studio.cli.tunerctl \
    --port /dev/ttyACM0 axis-state

PYTHONPATH=tools python3 -m espfoc_studio.cli.tunerctl \
    --port /dev/ttyACM0 retune --r 1.08 --l 0.0018 --bw 150
```

## Run the host-side tests

Each test file is standalone and exits with status 0 on success.

```bash
PYTHONPATH=tools python3 tools/espfoc_studio/tests/test_link_codec.py
PYTHONPATH=tools python3 tools/espfoc_studio/tests/test_tuner_protocol.py
PYTHONPATH=tools python3 tools/espfoc_studio/tests/test_analysis.py
QT_QPA_PLATFORM=offscreen PYTHONPATH=tools \
    python3 tools/espfoc_studio/tests/test_gui_smoke.py
```
