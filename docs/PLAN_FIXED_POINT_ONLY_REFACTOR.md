# espFoC: fixed-point-only refactor (canonical IQ31 API)

**Living checklist (what is done vs open):** see [`PLAN_STATUS.md`](PLAN_STATUS.md) to avoid duplicating effort.

**Release target:** **espFoC 3.0** — intentional **breaking change** vs 2.x (API, types, drivers). Downstream projects must migrate; compatibility is not preserved.

## Goals (from product intent)

- Remove **all** float-based control/driver **hot paths**; the control path is fixed-point (IQ31) with **canonical names** (no `_iq31` suffix on APIs/types that are “the” espFoC API). Where the stack still accepts **`float` at configuration or outer boundaries** (e.g. motor parameters from user code), conversion to internal fixed-point stays **inside** the library — callers do not need to implement driver normalization themselves.
- **Rewrite observers in fixed-point (IQ31)** as part of this refactor — algorithms were already validated in float; porting risk is accepted. No long-term float shim inside observers.
- **Add unit tests** for observer blocks to track **numerical stability** (regression-friendly tolerances, edge cases, and future drift detection).
- **Do not** wire sensorless from core in this refactor (sensorless strategy remains removed until a later milestone).
- **Drop** `CONFIG_ESP_FOC_USE_FIXED_POINT` in [`Kconfig`](components/espFoC/Kconfig): fixed-point is always on.
- **Leave scope** ([`esp_foc_scope.c`](source/motor_control/esp_foc_scope.c)) behavior unchanged (float `*` channels remain acceptable as a wire format).
- **OSAL**: remove `float` time API; use integer time or IQ31 as needed.
- **Tests**: delete float-first control tests; IQ31 tests as default suite with canonical names; **include observer IQ31 tests**.
- **Examples**: remove float demos; one sensored quick-start using canonical API.

## Non-goals (this pass)

- Reintroducing sensorless control strategy / core integration (explicitly later).

## Naming / header consolidation

| Current | Target |
|--------|--------|
| [`esp_foc_axis_iq31.h`](include/espFoC/esp_foc_axis_iq31.h) | Replace [`esp_foc_axis.h`](include/espFoC/esp_foc_axis.h): single `esp_foc_axis_t`, IQ31 state |
| [`esp_foc_units_iq31.h`](include/espFoC/esp_foc_units_iq31.h) | Replace [`esp_foc_units.h`](include/espFoC/esp_foc_units.h): unit wrappers with `iq31_t` |
| [`esp_foc_iq31_api.h`](include/espFoC/esp_foc_iq31_api.h) | Fold into [`esp_foc.h`](include/espFoC/esp_foc.h): `esp_foc_initialize_axis`, `esp_foc_align_axis`, `esp_foc_run`, `esp_foc_set_regulation_callback` |
| [`pid_controller_iq31.h`](include/espFoC/utils/pid_controller_iq31.h) | Replace [`pid_controller.h`](include/espFoC/utils/pid_controller.h) |
| [`ema_low_pass_filter_iq31.h`](include/espFoC/utils/ema_low_pass_filter_iq31.h) | Replace [`ema_low_pass_filter.h`](include/espFoC/utils/ema_low_pass_filter.h) |
| [`modulator_iq31.h`](include/espFoC/utils/modulator_iq31.h) + [`space_vector_modulator_iq31.h`](include/espFoC/utils/space_vector_modulator_iq31.h) | Merge into canonical modulator / SVM headers |
| [`foc_math_iq31.h`](include/espFoC/utils/foc_math_iq31.h), [`esp_foc_iq31.h`](include/espFoC/utils/esp_foc_iq31.h) | **Keep** as the fixed-point math layer |

Driver vtables: single IQ31 surface; remove float members and `CONFIG_ESP_FOC_USE_FIXED_POINT` guards.

## Driver contract (internal — not user-facing in v3.0.0 docs pass)

- **Normalization:** Inside drivers, represent physical quantities **proportionally in the range corresponding to [0, 1]** (IQ31 on the wire), consistent across the stack. This is **internal** to espFoC; **the public API does not require applications to know the scaling** — documentation will not emphasize this in the first implementation wave (see Documentation below).
- **Scope:** Unchanged — [`esp_foc_scope`](source/motor_control/esp_foc_scope.c) remains `float *` channels; applications convert IQ31 → float at the probe point if needed (accepted limitation).

## Core and sensorless

- Delete float [`esp_foc_core.c`](source/motor_control/esp_foc_core.c); merge [`esp_foc_core_iq31.c`](source/motor_control/esp_foc_core_iq31.c) into one `esp_foc_core.c`.
- Delete [`esp_foc_control_current_mode_sensorless.c`](source/motor_control/control_strategy/esp_foc_control_current_mode_sensorless.c) and core init branches; remove declarations from [`esp_foc_controls.h`](include/espFoC/esp_foc_controls.h).
- Merge sensored IQ31 → canonical [`esp_foc_control_current_mode_sensored.c`](source/motor_control/control_strategy/esp_foc_control_current_mode_sensored.c).

## Observers — IQ31 port + tests

**Scope:** [`esp_foc_pll_observer.c`](source/motor_control/observers/esp_foc_pll_observer.c), [`esp_foc_pmsm_model_observer.c`](source/motor_control/observers/esp_foc_pmsm_model_observer.c), [`esp_foc_kf_observer.c`](source/motor_control/observers/esp_foc_kf_observer.c), [`esp_foc_simu_observer.c`](source/motor_control/observers/esp_foc_simu_observer.c), plus public headers under [`include/espFoC/observer/`](include/espFoC/observer/).

**Porting approach:**

- Replace internal `float` state and parameters with IQ31 (or fixed scalars where appropriate: some coeffs may stay as `iq31_t` constants from `iq31_from_float` at init).
- **Kalman / heavy dynamics:** If IQ31 proves insufficient for covariance or intermediate products, **scale those internals to Q63** (or wider fixed intermediates) for that observer only; keep the rest of FoC on IQ31 unless a wider change is justified.
- Replace `esp_foc_sine` / float trig with [`iq31_sin`](include/espFoC/utils/esp_foc_iq31.h) / `iq31_cos` (or existing LUT paths) consistent with the rest of FoC.
- Remove dependency on float [`foc_math.h`](include/espFoC/utils/foc_math.h) from observers after port.
- [`esp_foc_sine_cosine.c`](source/motor_control/esp_foc_sine_cosine.c): delete or reduce to unused once nothing calls float sine; confirm no other callers remain.

**Unit tests (new / expanded):**

- Add `test_observer_pll_iq31.c` (or a single `test_observers_iq31.c` with subsections) covering:
  - Typical operating points vs a **frozen float reference** (optional one-time baseline) or vs **analytical small-signal** checks where feasible.
  - Edge cases: zero input, saturation, angle wrap, very small `dt` (via IQ31 `dt` representation).
  - **Stability smoke tests:** multiple steps / short “simulation runs” with bounded state norms (no NaN/inf in fixed-point — use range checks on `iq31_t` lanes).
- Document tolerances in [`test/README.md`](test/README.md) as **numerical stability** monitors (tighten over time if needed).
- Run under [`examples/unit_test_runner`](examples/unit_test_runner) like other espFoC tests.

**Note:** Core still does not allocate observers until sensorless returns; tests drive observers **directly** with mock inputs.

## OSAL

- Replace `float esp_foc_now_seconds` in [`os_interface.h`](include/espFoC/osal/os_interface.h); update [`os_interface_idf.c`](source/osal/os_interface_idf.c) and sensored strategy timestamps.

## Drivers

- IQ31-only implementations (per **Driver contract** above); update [`driver_iq31_local.h`](source/drivers/espFoC/driver_iq31_local.h) naming.

## Scope

- No change to [`esp_foc_scope.c`](source/motor_control/esp_foc_scope.c) API.

## Tests (control + pipeline)

- Remove float-first tests (`test_foc_math.c`, `test_pid.c`, `test_lp_filter.c`, `test_axis_flow.c`, …).
- Rename IQ31 tests to canonical filenames; remove `CONFIG_ESP_FOC_USE_FIXED_POINT` guards.
- [`test_foc_pipeline_iq31.c`](test/test_foc_pipeline_iq31.c): IQ31-only golden (no float baseline path).

## Kconfig

- Remove `ESP_FOC_USE_FIXED_POINT` from [`Kconfig`](components/espFoC/Kconfig) and from all sources/tests.

## Documentation (separate step **after** implementation)

- **Not** part of the main coding milestone: full **README**, examples narrative, path fixes under `docs/`, and removal/replace [`docs/DRIVERS_FIXED_POINT.md`](docs/DRIVERS_FIXED_POINT.md) in a **follow-up documentation pass** once the API has landed.
- Minimal inline comments during implementation are fine; polished user-facing docs wait for that pass.

## Examples

- Remove float examples; single sensored quick-start: [`examples/axis_sensored`](examples/axis_sensored) (canonical IQ31 API). Keep [`examples/unit_test_runner`](examples/unit_test_runner).
- Example **README / polish** can track the post-implementation documentation pass if needed.

## Rust FFI

- Update or gate [`source/rust/esp_foc_ffi.c`](source/rust/esp_foc_ffi.c) until IQ31-ready.

## Verification

- Component build + Unity/QEMU green.
- Observer tests part of the standard suite for ongoing numerical regression.

## Suggested implementation order

1. Kconfig + driver interfaces + drivers (internal [0,1] proportional IQ31 contract).
2. Utils merge (PID/LPF/modulator); delete float utils from control path.
3. Axis/units + `esp_foc.h`.
4. Core + sensored merge; delete sensorless + float duplicates.
5. OSAL time + sensored timestamps.
6. **Observers IQ31 port** (Q63 for KF if required) + **observer unit tests** + remove float sine path if unused.
7. Pipeline + integration tests.
8. Examples (canonical sensored) — minimal doc strings only.
9. **Documentation pass** (README, `docs/`, links, espFoC 3.0 migration note).

## Follow-on milestone: full Q16.16 pipeline (option B)

After the **IQ31-canonical** 3.0 baseline above, a separate milestone may replace the internal signal representation with **Q16.16** (~±32k engineering range, ~1/65536 fractional resolution) across the control pipeline — **implementation first**, **unit test refactor second**. See **[`PLAN_Q16_FULL_PIPELINE.md`](PLAN_Q16_FULL_PIPELINE.md)** and status in [`PLAN_STATUS.md`](PLAN_STATUS.md) (section 10).

---

*Planning artifact; trim after merge if desired.*
