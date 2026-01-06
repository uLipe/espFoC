# espFoC Rust (esp-idf-sys)

This folder contains an **optional** Rust integration for espFoC, built on top of
`esp-idf-sys` (STD environment).

The Rust side talks to espFoC exclusively through a **small, stable C FFI**:
- `include/rust/esp_foc_ffi.h`
- `source/rust/esp_foc_ffi.c`

## Build & Flash

This follows the standard `esp-idf-sys` workflow (same as `esp-idf-template`).

1) Install prerequisites (toolchain + helpers)

- `espup` (toolchain installer)
- `espflash` or `cargo-espflash` (flashing)

2) Select the correct Rust target for your chip, e.g.:

- ESP32: `xtensa-esp32-espidf`
- ESP32-S3: `xtensa-esp32s3-espidf`
- ESP32-C3: `riscv32imc-esp-espidf`

3) Build:

```bash
cd rust
cargo build --target xtensa-esp32-espidf
```

4) Flash + monitor (example):

```bash
cd rust
MCU=esp32 cargo espflash flash --target xtensa-esp32-espidf --example open_loop_voltage_ramp --monitor
```

> Tip: add a default target in `rust/.cargo/config.toml` to avoid typing `--target` every time.

## Example

- `examples/open_loop_voltage_ramp.rs`
  - open-loop voltage mode
  - ramps `Vq` up/down (Ud = 0)
  - adjust GPIO and DC link voltage constants at the top of the file
