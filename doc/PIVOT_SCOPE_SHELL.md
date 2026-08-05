# Pivot: scope-only stream + console shell (no gui_link / tuner)

This document replaces `TUNING.md` and all host link/tuner documentation.
There is **no** compatibility layer with protocol v2 (`0xA5` framing, CONNECT, TUNER channel).

## Goals

| Component | Role |
|-----------|------|
| **Firmware scope** | Push fixed-layout frames on UART / USB-CDC / USB-Serial-JTAG while axis is **aligned and running** |
| **espfoc_tool** | Scope viewer only: open port, decode frames, GPU plots, runtime channel picker |
| **espfoc_shell** (optional) | REPL on the **main console UART** for calibration, motion, NVS (`config.*`), axis introspection |
| **Removed** | `gui_link`, `esp_foc_tuner`, `esp_foc_link_session`, `espfocctl`, Tune/Dashboard/NVS GUI, fake loopback, link tests |

Keep: motor control, calibration/NVS APIs, logo → tool splash (`doc/images/espfoc_tool_logo.svg`).

---

## Wire format: `ESPF` scope frame v1

All multi-byte fields are **little-endian**. Each sample is one **frame**.

```
offset  size  field
------  ----  -----
0       4     magic_hdr   = 0x45 0x46 0x50 0x46  ('E','F','P','F')
4       2     seq         uint16, monotonic per sender (wrap OK)
6       2     n_ch        uint16, must equal CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS
8       4×N   samples     int32 each (q16_t raw, same as today)
8+4N    4     magic_ftr   = 0x46 0x4E 0x44 0x21  ('F','N','D','!')
```

**Frame size:** `12 + 4×N` bytes (e.g. N=14 → 68 bytes).

Properties:

- Self-synchronizing: scan RX byte stream for `ESPF`, validate `n_ch`, read payload, check `FND!`.
- No CRC in v1 (optional v2: CRC32 before footer).
- No commands, no ACK, no session.
- Host decoder is state machine only (no threads inside decoder).

Invalid `n_ch` or bad footer → drop frame, resync.

---

## Firmware architecture

### `esp_foc_scope` (keep, rewrite encoder)

- Channel registration API unchanged (`esp_foc_scope_add_channel`).
- Capture + ping-pong buffer unchanged (slow runner / `esp_foc_scope_data_push`).
- Daemon builds **ESPF** frame instead of SCOPE v1 + link wrap.
- **Stream gate:** `esp_foc_scope_stream_active(axis)` → true only when  
  `axis->state == ESP_FOC_AXIS_STATE_RUNNING` **and** axis was aligned  
  (same policy as today’s “no motion → host pumps idle” but **no bytes on wire**).

### `esp_foc_stream_bridge` (new, replaces gui_link drivers)

Public API (`include/espFoC/stream/esp_foc_stream_bridge.h`):

```c
typedef void (*esp_foc_stream_send_fn)(const uint8_t *data, size_t len);

void esp_foc_stream_bridge_init(esp_foc_stream_send_fn send);
void esp_foc_stream_bridge_send_frame(const uint8_t *data, size_t len);
```

Kconfig **choice** (no dependency on tuner):

- `ESP_FOC_STREAM_BRIDGE_UART`
- `ESP_FOC_STREAM_BRIDGE_USBCDC` (TinyUSB S2/S3/P4)
- `ESP_FOC_STREAM_BRIDGE_USB_SERIAL_JTAG` (C3/C5/C6/…)
- `ESP_FOC_STREAM_BRIDGE_NONE` (weak stub)

Each driver: install HW, RX task optional (unused for scope-only TX), implement `send`.

`esp_foc_scope_initalize()` calls `esp_foc_stream_bridge_init()` instead of `esp_foc_tuner_init_bus_callback()`.

### Delete (firmware)

```
include/espFoC/gui_link/*
source/gui_link/esp_foc_link.c
source/gui_link/esp_foc_tuner.c
source/gui_link/esp_foc_link_session.c
source/gui_link/esp_foc_bus_stub.c
include/espFoC/drivers/gui_link/*  → move/rename to stream/
source/drivers/gui_link/*         → stream/
test/test_link.c
test/test_tuner.c
test/test_tuner_reactor.c
```

### `espfoc_shell` (new optional module)

Kconfig: `CONFIG_ESPFOC_SHELL` (default y on examples, independent of scope bridge).

- REPL on **IDF console** (`stdin`/`stdout` of `CONFIG_ESP_CONSOLE_UART`).
- Registration: `espfoc_shell_register_axis(id, esp_foc_axis_t *)` (runtime OK).
- `espfoc_shell_start()` spawns reader task (line buffer, echo).

**Command grammar:** `<verb> [subcommand] [args...] <axis_id>`

| Verb | Example | Maps to |
|------|---------|---------|
| `list` | `list config 0` | Print `config.*` fields |
| `list` | `list axis` | All registered axis ids |
| `list` | `list axis 0` | Print readable axis state |
| `get` | `get config.kp 0` | Read calibration field |
| `set` | `set config.ki 120 0` | RAM + optional NVS path for `config.*` |
| `set` | `set id 0 0` / `set iq 1.5 0` | `target_i_d` / `target_i_q` |
| `set` | `set kp 0.8 0` / `set ki 50 0` | Current-loop RAM gains |
| `align` | `align 0` | `esp_foc_align` path |
| `run` | `run 0` | `esp_foc_run` |
| `stop` | `stop 0` | `esp_foc_stop` |
| `store` | `store 0` | `esp_foc_calibration_axis_tuner_store` |
| `erase` | `erase` | `esp_foc_calibration_erase` |
| `reboot` | `reboot` | `esp_restart` |

Help: `help`, `help config`, `help axis`.

Calibration NVS Kconfig: **`ESP_FOC_CALIBRATION_NVS` must not depend on tuner**.

### `axis_shell` example (replaces `axis_tuning`)

- Drop all `esp_foc_tuner_*` / `esp_foc_link_session_*`.
- `wire_scope_channels()` + `esp_foc_scope_initalize()`.
- `sdkconfig.defaults`: `ESP_FOC_SCOPE=y`, stream bridge USBCDC on C6, **no** `ESP_FOC_TUNER_ENABLE`.
- Console UART for shell; USJ for scope stream only.

---

## Host: `espfoc_tool` (rewrite)

### Process model

```
[Serial read thread]  →  ring buffer  →  [Decode thread]  →  per-channel ring buffers
                                                              ↓
                                                    [UI thread / GPU]  pyqtgraph or VisPy
```

- **No** `TunerClient`, **no** `LinkReader` demux, **no** CONNECT.
- Open port from combo box (enumerate `serial.tools.list_ports`); optional CLI `--port` skips dialog.
- Splash screen with SVG logo on startup.

### UI (minimal)

1. Main area: multi-plot grid (pyqtgraph OpenGL).
2. FAB **`+`** → slide/page **Channel picker**: human name, unit, scale (from static catalog).
3. Selected channels appended to plots; can add/remove at runtime.
4. Status bar: port, FPS, frames/s, last seq gap.

### Channel catalog

Static Python module `espfoc_tool/catalog/axis_tuning_14ch.py` mirroring today’s `axis_tuning` map:

| Index | Name | Unit | Display scale |
|-------|------|------|----------------|
| 0 | target_id | A | q16 → float |
| … | … | … | … |

Future profiles: extra catalog files, no protocol change.

### Delete (host)

```
tools/espfoc_tool/protocol/
tools/espfoc_tool/link/          (replace with stream/)
tools/espfoc_tool/cli/
tools/espfoc_tool/gui/*          (replace with scope_ui/)
tools/espfoc_tool/fake_tuner_loopback.py
tools/espfoc_tool/tests/test_*tuner*
tools/espfoc_tool/tests/test_link*
```

New tests: `test_stream_frame_codec.py`, decode fuzz, golden frames.

---

## Documentation updates

| Action | Path |
|--------|------|
| Delete | `doc/TUNING.md` |
| Add | `doc/SCOPE_TOOL.md` (user guide for tool + wire format) |
| Add | `doc/SHELL.md` (REPL reference) |
| Rewrite | `README.md`, `tools/espfoc_tool/README.md`, `changelog.txt` |
| CI | Drop tuner python tests; build axis_tuning only; add stream decode tests |

---

## Implementation phases

| Phase | Deliverable | Est. |
|-------|-------------|------|
| **P0** | This doc + `esp_foc_stream_frame.h` + host `stream/frame.py` | 1 day |
| **P1** | Firmware: ESPF encoder, stream bridges, delete gui_link/tuner, `axis_shell` builds | done |
| **P2** | Host: scope tool (port picker, decode, plots, splash, `+` channels) | done |
| **P2.1** | QML / Material 3 UI (splash, plot cards, channel picker) | done |
| **P3** | `espfoc_shell` REPL + `doc/SHELL.md` (align, get/set, run/stop) | done |
| **P4** | CI hardening, extra catalogs (`test_encoder`), docs/changelog purge | next |

**Non-negotiable:** no shims (`esp_foc_tuner_*` macros), no dual decoder in tool.

---

## Decisions locked (2025-06)

| # | Decision |
|---|----------|
| 1 | **Fixed `n_ch`** every frame (= `CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS`) |
| 2 | **`seq` u16** in header; **no CRC**; tool is **passive RX + plot only** |
| 3 | **No discovery** — channel names/units live in host catalog (`axis_shell_17ch.py`, see `SCOPE_CHANNELS.md`) |
| 4 | **Shell** on main console UART (same as `ESP_LOG` / `idf monitor`); **scope** on separate bridge (USJ on C6) |
| 5 | **`examples/axis_tuning` removed** → **`examples/axis_shell`** is the reference app |

---

## Example: `axis_shell`

- Boots axis + NVS apply (unchanged).
- `esp_foc_scope_bind_axis()` + wire 17 channels (see `SCOPE_CHANNELS.md`).
- `esp_foc_stream_bridge` on USB-CDC / USJ (scope TX only, no RX task).
- `espfoc_shell_start()` on console UART after `espfoc_shell_register_axis(0, &axis)`.
- No tuner, no link session, no `pump_scope_idle` on wire.
