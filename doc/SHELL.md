# espfoc_shell (console REPL)

Line-oriented commands on the **main UART** used by `idf monitor`. The scope stream (ESPF) uses a **separate** port — see [SCOPE_TOOL.md](SCOPE_TOOL.md).

Enable: `CONFIG_ESPFOC_SHELL=y` (default on `axis_shell`).

Console I/O goes through **OSAL** (`read`/`write` on stdin/stdout via the IDF console VFS).
The shell module has no direct UART/GPIO dependencies.

## Grammar

```
<verb> [subcommand] [args...] <axis_id>
help [config|axis]
```

**Axis id is always the last token** (except `list axis` with no id, and global `erase` / `reboot`).

| Form | Valid? |
|------|--------|
| `list axis` | yes — lists all registered axis ids |
| `list axis 0` | yes — detail for axis 0 |
| `list config 0` | yes |
| `align 0` / `run 0` / `stop 0` / `store 0` | yes |
| `set iq 0.5 0` / `get config.kp 0` | yes |
| `0 align` / `0 list axis` | **no** — axis id must be last |
| `align` / `run` without axis id | **no** |

## Commands

| Verb | Example | Action |
|------|---------|--------|
| `help` | `help` / `help config` | Usage |
| `list` | `list axis` | All registered axis ids + state |
| `list` | `list axis 0` | State, targets, rotor estimates |
| `list` | `list config 0` | Tuning fields (live RAM) |
| `get` | `get config.kp 0` | Read one config field |
| `set` | `set config.ki 120 0` | Write config field (RAM) |
| `set` | `set iq 1.0 0` | Target current [A] |
| `align` | `align 0` | `esp_foc_align_axis()` |
| `run` | `run 0` | Start FOC — scope streams when **running** |
| `stop` | `stop 0` | `esp_foc_stop()` |
| `store` | `store 0` | Save live tuning to NVS |
| `erase` | `erase` | Erase all calibration NVS |
| `reboot` | `reboot` | Restart |

### Config fields (`config.*`)

| Field | Aliases | Unit / note |
|-------|---------|-------------|
| `kp` | — | Current-loop P |
| `ki` | — | Current-loop I |
| `kd` | — | Current-loop D |
| `kff` | — | Feed-forward |
| `integrator_limit` | `ilim` | Anti-windup |
| `current_filter_fc_hz` | `lpf`, `i_lpf` | Shunt LPF cutoff [Hz] |
| `pole_pairs` | `pp` | Motor pole pairs |

Requires `CONFIG_ESP_FOC_CALIBRATION_NVS=y` for `store` / `erase` persistence.

## Typical bring-up (`axis_shell`)

```text
espFoC> list axis
espFoC> list config 0
espFoC> align 0
espFoC> set iq 0.5 0
espFoC> run 0
```

Open **espFoC Tool** on the USB Serial/JTAG / `ttyACM` scope port while the axis is **running**.

```text
espFoC> stop 0
espFoC> store 0
```

## Errors

Commands print short tokens: `ok`, `not_aligned`, `invalid_state`, `aligning`, etc. (see `esp_foc_err_t`).
