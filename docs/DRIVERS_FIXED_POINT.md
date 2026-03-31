# Drivers: fixed-point (IQ31) plan alongside float

> **Deprecated / historical:** espFoC 3.x targets a **single IQ31 control path**; `CONFIG_ESP_FOC_USE_FIXED_POINT` is **removed** from Kconfig. Driver code and tests no longer follow the optional dual float/IQ31 model described below. Keep this file only as background for the migration; see [`PLAN_STATUS.md`](PLAN_STATUS.md) and [`PLAN_FIXED_POINT_ONLY_REFACTOR.md`](PLAN_FIXED_POINT_ONLY_REFACTOR.md).

**Goal (2.x-era doc):** float remains the default; with `CONFIG_ESP_FOC_USE_FIXED_POINT=y`, each driver exposes **IQ31 vtable paths**, built only under that option, without removing the legacy float API.

## Principles

1. A **single** Kconfig flag (`CONFIG_ESP_FOC_USE_FIXED_POINT`) controls compilation of FP extensions in driver headers and `.c` files.
2. **IQ31 semantics** match the espFoC core: **normalized** quantities in Q1.31 in **[-1, 1)** (currents, phase voltages, per-revolution angles where applicable).
3. Quantities **not** representable in [-1, 1) (e.g. multi-turn accumulated counts, DC bus in volts): stay **`float`** or use explicit **`int32_t`/`int64_t`** on the FP API — documented per driver.
4. **Conditional implementation**: `fetch_isensors_iq31`, `set_voltages_iq31`, etc. exist on the object only when the option is enabled; otherwise the struct has no such members (clearer than `NULL` pointers).

## Interfaces (summary)

| Driver | Float API (existing) | IQ31 extension (when enabled) |
|--------|------------------------|--------------------------------|
| Current | `fetch_isensors` → `isensor_values_t` | `fetch_isensors_iq31` → `isensor_values_iq31_t` (iu/iv/iw per axis in Q1.31) |
| Inverter | `set_voltages` (vu,vv,vw float) | `set_voltages_iq31` (vu,vv,vw iq31); `get_dc_link` may stay float |
| Rotor | `read_counts` float | `read_counts_iq31` as **normalized position in one turn** [0,1) in Q1.31; accumulated: `int64_t` ticks per hardware |

Exact types are in `*_interface.h`.

## Drivers covered

- **Current:** `current_sensor_adc.c`, `current_sensor_adc_one_shot.c`
- **Inverter:** `inverter_3pwm_mcpwm.c`, `inverter_6pwm_mcpwm.c`, `inverter_3pwm_ledc.c`
- **Rotor:** `rotor_sensor_as5600.c`, `rotor_sensor_as5048.c`, `rotor_sensor_pcnt.c`

## Unit tests

- **Mocks** (`mock_drivers.c` / `.h`): extended with counters and last IQ31 values mirroring the API.
- **New tests** (`test_driver_iq31_api.c`): `TEST_CASE`s only exist with `CONFIG_ESP_FOC_USE_FIXED_POINT=y`; they call the IQ31 vtable and check scale/consistency (e.g. `set_voltages_iq31` with `IQ31_ONE`).
- **Runner:** optional job or `sdkconfig.defaults` to run the suite with FP enabled in addition to the default float build.

## Suggested implementation order

1. Types + vtables in headers (this step).
2. Mocks + minimal IQ31 API tests.
3. One pilot driver (mocks already validate; then e.g. `inverter_3pwm_mcpwm` or `current_sensor_adc_one_shot`).
4. Replicate the pattern for the rest.

## Notes

- ADC → current: the IQ31 path applies the **same physical scale** as float (offsets via `lroundf` of float offsets; gain as `iq31_t` precomputed at init), **without** calling `fetch_isensors`.
- PWM: `set_voltages_iq31` computes comparator ticks with `esp_foc_iq31_duty_ticks` (Q1.31 × period in integer ticks), **without** calling `set_voltages`.
- Rotor: separate IQ31 state (`prev_raw_iq`, `accum_i64`, etc.) — does **not** call float `read_counts` / `read_accumulated_counts`.
- `driver_iq31_local.h`: integer/Q1.31 helpers only (no wrapper around float driver functions).
