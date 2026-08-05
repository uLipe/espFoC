# espFoC Tool — passive scope viewer

Host-side viewer for the **ESPF** binary stream (`ESPF` magic, fixed channel count, no CRC, no commands).

## Wire format

See [PIVOT_SCOPE_SHELL.md](PIVOT_SCOPE_SHELL.md) and [SCOPE_CHANNELS.md](SCOPE_CHANNELS.md).

## Run

```bash
pip install -r tools/espfoc_tool/requirements.txt
PYTHONPATH=tools python3 -m espfoc_tool.gui
```

- Without `--port`: port picker — choose **USB Serial/JTAG** (`ttyACM*` on ESP32-C6), **not** the UART used by `idf.py monitor`.
- With port: `PYTHONPATH=tools python3 -m espfoc_tool.gui --port /dev/ttyACM0`

## UI (QML / Material 3)

1. Splash → port dialog → main shell (watermark, responsive plot grid, status bar).
2. **`+`** opens channel picker (color + catalog).
3. Each plot card: grid, trace, hover readout, remove. Cards reflow to fit the window; scroll when there are more plots than fit on screen.
4. **Autoset** clears capture buffers and rebases the time axis (use if the trace looks wrong or stale).
5. **⚙ Plot settings** — sample rate (1–1000 Hz), max points (10–2048); time window = points ÷ rate. Fixed horizontal scale avoids waveform compression at startup.

Catalog: `tools/espfoc_tool/catalog/axis_shell_17ch.py` (names, units, Q16 scale).

---

## Connect your application to the tool

The tool is **RX-only**. The firmware must (1) enable scope, (2) wire channels, (3) select a stream bridge, (4) run the axis so the encoder transmits.

### 1. Kconfig (app `sdkconfig.defaults`)

```ini
CONFIG_ESP_FOC_SCOPE=y
CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS=17
CONFIG_ESP_FOC_SCOPE_BUFFER_SIZE=8
CONFIG_ESP_FOC_SCOPE_DAEMON_PRIORITY=3
CONFIG_ESP_FOC_STREAM_BRIDGE_USBCDC=y   # esp32c6: USB Serial/JTAG
CONFIG_ESPFOC_SHELL=y
CONFIG_ESP_CONSOLE_UART_DEFAULT=y
```

On ESP32-S2/S3/P4 use TinyUSB CDC; on C6/C3 use `USBCDC` → USJ driver in CMake.

`CONFIG_ESP_FOC_SCOPE=y` **requires** a bridge (UART or USBCDC) — no stub fallback.

### 2. Wire channels

For **`axis_shell` (17ch)** this is automatic: `esp_foc_initialize_axis()` calls
`esp_foc_scope_initalize()` first, then `esp_foc_scope_wire_axis()` at the end.

Custom maps: `esp_foc_scope_bind_axis()` + `esp_foc_scope_add_channel()` per
channel (see `SCOPE_CHANNELS.md` for the standard order).

### 3. Shell + axis lifecycle

Console REPL uses the **same UART as** `idf monitor`. Scope uses the **bridge port** (separate USB function on C6).

```text
espFoC> align 0
espFoC> set iq 0.5 0
espFoC> run 0
```

Frames are sent only while `axis->state == RUNNING` (unless `CONFIG_ESP_FOC_SCOPE_TX_ALWAYS=y` for bring-up).

### 4. Host

```bash
# Terminal 1
idf.py flash monitor

# Terminal 2
PYTHONPATH=tools python3 -m espfoc_tool.gui --port /dev/ttyACM0
```

Status bar: `frames=N` should increase after `run 0`. The scope bridge is silent on the hot path (no periodic TX logs) so the shell REPL stays usable on the monitor UART.

### 5. Custom channel maps

1. Add `tools/espfoc_tool/catalog/your_app_Nch.py` (`ChannelDesc`, `EXPECTED_N_CH`, `BY_INDEX`).
2. Point `StreamReader(..., expected_n_ch=N)` in your GUI backend (see `tools/espfoc_tool/gui/backend.py`).
3. Keep `CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS` equal to `EXPECTED_N_CH`.

### 6. Debug (JTAG + GDB + monitor in tmux)

ESP32-C6 uses **two USB cables**: UART adapter (`ttyUSB*`) for flash/monitor, native USB-JTAG for OpenOCD/GDB.

```bash
cd examples/axis_shell
idf.py set-target esp32c6
python3 ../../scripts/esp_foc_debug.py -p /dev/ttyUSB0
```

Builds/flashes on UART, OpenOCD on USB-JTAG, tmux: **left = GDB**, **right = shell monitor**.

See [SHELL.md](SHELL.md) for REPL grammar.

---

## Reference firmware

[axis_shell](../examples/axis_shell/) — FITL default, 17ch scope, shell on UART, stream on USJ.
