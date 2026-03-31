# espFoC 3.0 fixed-point plan — status snapshot

**Purpose:** Single place to see what is already done vs still open, so we do not duplicate work or rip out code someone else is migrating.

**Source of truth for intent:** [`PLAN_FIXED_POINT_ONLY_REFACTOR.md`](PLAN_FIXED_POINT_ONLY_REFACTOR.md) (espFoC 3.0 baseline). **Follow-on numeric milestone:** [`PLAN_Q16_FULL_PIPELINE.md`](PLAN_Q16_FULL_PIPELINE.md) (full **Q16.16** pipeline — option B).

**Last audit:** 2026-03-25 — Q16 pipeline + observer/LPF bridge landed; see §10.

---

## Legend

| Mark | Meaning |
|------|--------|
| DONE | Matches the plan for this item; no duplicate effort needed unless regressing. |
| PARTIAL | Some work landed; read the notes before picking up. |
| OPEN | Not done or intentionally deferred; safe to schedule next. |
| DRIFT | Docs or tests disagree with code (update when touching nearby files). |

---

## Suggested order → status

### 1. Kconfig + driver interfaces + drivers (IQ31 wire contract)

- **DONE (flag):** `ESP_FOC_USE_FIXED_POINT` / `CONFIG_ESP_FOC_USE_FIXED_POINT` is **not** present in current [`Kconfig`](../Kconfig) (fixed-point is not optional via that switch anymore).
- **DONE (doc note):** [`docs/DRIVERS_FIXED_POINT.md`](DRIVERS_FIXED_POINT.md) has a **deprecated / historical** banner pointing to the 3.x single-path model.

### 2. Utils merge (PID / LPF / modulator); float off control hot path

- **DONE:** Same headers — **Q16.16** on the control hot path (PID, EMA LPF, modulator/SVM); `esp_foc_pid_update_float` and test-only float modulator helpers (`test_float_modulator_ref.h`) remain for parity tests only.

### 3. Axis / units + `esp_foc.h`

- **PARTIAL:** Only one axis header found: [`esp_foc_axis.h`](../include/espFoC/esp_foc_axis.h) (no separate `esp_foc_axis_iq31.h` in tree).
- **Verify** when changing public API: legacy typedef names `*_iq31_t` in [`esp_foc_units_iq31.h`](../include/espFoC/esp_foc_units_iq31.h) still alias **Q16** `raw` fields.

### 4. Core + sensored merge; remove sensorless + float duplicates

- **DONE (core):** No `esp_foc_control_current_mode_sensorless.c` under `source/motor_control` (sensorless strategy removed from core as planned).
- **NOTE:** Example [`examples/axis_sensorless`](../examples/axis_sensorless) may still exist — decide in **step 8** (examples) whether to delete or archive.

### 5. OSAL time + sensored timestamps

- **DONE (OSAL):** [`os_interface.h`](../include/espFoC/osal/os_interface.h) exposes `uint64_t esp_foc_now_useconds(void)` (no float `esp_foc_now_seconds` in this header).
- **DONE:** [`esp_foc_control_current_mode_sensored.c`](../source/motor_control/control_strategy/esp_foc_control_current_mode_sensored.c) low-speed loop: **no float** — angle latency via `q16_from_elapsed_us_u64()`; shaft speed via `q16_mul(Δθ, torque_controller[0].inv_dt)` (same `inv_dt` as core init for 1/(dt·N)).

### 6. Observers IQ31 + tests + float sine cleanup

- **DONE (observers):** Observer **I/O** remains **Q1.31** per [`esp_foc_observer_interface.h`](../include/espFoC/observer/esp_foc_observer_interface.h). Internal state uses IQ31 math; **PLL/KF** current/EMF filtering uses the **Q16.16 EMA** with explicit per-unit conversion via [`esp_foc_iq31_q16_bridge.h`](../include/espFoC/utils/esp_foc_iq31_q16_bridge.h) (fixes scale mismatch vs raw `int32_t` aliasing). Tests: `test_observers_iq31.c`, `test_observers_trajectory_iq31.c`.
- **OPEN:** [`esp_foc_sine`](../source/motor_control/esp_foc_sine_cosine.c) / `foc_math.h` still used by [`esp_foc_core.c`](../source/motor_control/esp_foc_core.c), float unit tests, and some **examples** (`test_drivers/...`). Plan item “remove float sine path **if unused**” → **not** satisfied until those callers are migrated or scoped as “legacy examples only”.

### 7. Pipeline + integration tests

- **DONE (pipeline test):** [`test/test_foc_pipeline_iq31.c`](../test/test_foc_pipeline_iq31.c) exercises **Q16** PID/LPF/modulator (legacy filename); determinism + bounds. Further integration tests can be added separately.

### 8. Examples (canonical sensored only)

- **DONE:** [`examples/axis_sensored`](../examples/axis_sensored) is the single sensored IQ31 quick-start; `examples/axis_sensored_iq31` was removed after merge.

### 9. Documentation pass

- **OPEN:** README, `docs/`, migration note for 3.0 — scheduled **after** API stabilizes.

### 10. Q16.16 full pipeline (option B — engineering range + ~1/65536 fractions)

- **PARTIAL:** Core **control path** (drivers, axis, sensored strategy, PID, LPF, modulator, `foc_math_q16.h`) uses **Q16.16**; [`esp_foc_units_iq31.h`](../include/espFoC/esp_foc_units_iq31.h) typedefs map to `q16_t`. Observers keep **IQ31** API/state with **bridge** at the EMA LPF (see §6). Remaining: **full** observer port to Q16 (optional), rename legacy `*_iq31` filenames/tests, user docs.
- **Order:** **(1)** production code + headers + observers — largely done; **(2)** refactor unit tests + `test/README.md` — **do not** block implementation on full test rewrites.
- **Note:** Integers (`uint32_t` / `int`) remain appropriate for **counts** (CPR, etc.); Q16.16 targets **scalable engineering values** (gains, R/L, pole-pair scaling, etc.) and **fractional** dynamics in one format.

---

## Tests: intentional duplication vs plan

The plan suggested **removing** float-first tests (`test_foc_math.c`, `test_pid.c`, `test_lp_filter.c`). **`test_lp_filter.c` is removed**; `test_foc_math.c` and `test_pid.c` remain as float regression alongside `test_*_iq31.c` until explicitly dropped.

- **Recommendation:** See [`test/README.md`](../test/README.md) for which suites are canonical vs reference-only.

---

## Quick “who should work on what next?”

| If you want to… | Pick |
|-----------------|------|
| Next large refactor chunk | Step **10** — finish test renames + optional observer Q16-only; or step **9** docs. |
| User-facing docs | Step **9** — README / migration note (after API stable). |
| Float reference tests | Optional: delete `test_foc_math.c` / `test_pid.c` when redundant; `test_lp_filter.c` already removed. |

---

*Update this file when a step closes or when the audit is refreshed.*
