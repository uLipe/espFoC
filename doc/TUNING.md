# espFoC tuning reference

Complete reference for every way gains can be produced and updated in
espFoC — complements the high-level tour in the main README. Order
reflects the typical bring-up flow: pick a motor profile at build
time, flash, then refine live through the TunerStudio GUI (or the
`tunerctl` CLI for scripting).

## Matched Pole-Zero synthesis

The tuner, the build-time autogen and the host `model.analysis` module
all share the same closed-form design, assuming a first-order PMSM
current plant `G(s) = 1 / (R + L·s)` and ZOH discretization at the
control-loop period `Ts = decimation / f_pwm`:

```
alpha = exp(-R·Ts/L)              # discrete plant pole
beta  = exp(-2·pi·bw_hz·Ts)       # desired closed-loop pole
Kp    = R·(1 − beta)/(1 − alpha)
Ki    = R·(1 − beta)/Ts
```

## Build-time autotuner

Motor profiles live in `scripts/motors/*.json`. Selecting one with
`CONFIG_ESP_FOC_MOTOR_PROFILE` runs `scripts/gen_pi_gains.py` during
the build and emits `esp_foc_autotuned_gains.h` into the **build
directory** (never the source tree — `.gitignore` and a path guard in
the script keep it that way).

The generated header is consumed by `esp_foc_initialize_axis()`
unconditionally — every axis boots with the autotuned gains.
Override at runtime with `esp_foc_axis_retune_current_pi_q16()`
or by writing through the tuner protocol; persist the result with
`esp_foc_calibration_save()` so subsequent boots come up already
tuned (see *NVS calibration overlay* below).

Example run on the reference gimbal profile:

```
$ python3 scripts/gen_pi_gains.py \
      --motor scripts/motors/default.json \
      --output build/.../gen/esp_foc_autotuned_gains.h \
      --report build/.../gen/esp_foc_autotuned_report.txt \
      --pwm-rate-hz 20000 --decimation 20
espFoC PI autotuner: ...esp_foc_autotuned_gains.h generated
                     (Kp=1.4610 V/A, Ki=659.17 V/(A*s), PM_delay=36.7 deg)
```

The script aborts the build when the result would be unsafe:

- `exit 4` — requested bandwidth above Nyquist (`bw·Ts ≥ 0.5`);
- `exit 5` — phase margin below the profile's `min_phase_margin_deg`.

A human-readable report sidecar (`...autotuned_report.txt`) records
the design context (motor params, Ts, α/β, Kp/Ki, phase-margin
estimates) for the CI log.

## Runtime tuning API

`include/espFoC/esp_foc_axis_tuning.h` exposes three primitives that
apply atomically through a short critical section; the integrator is
reset on every swap to keep the post-swap response well-defined:

```c
esp_foc_axis_retune_current_pi_q16(axis, R_q16, L_q16, bw_hz_q16);
esp_foc_axis_set_current_pi_gains_q16(axis, Kp, Ki, integrator_limit);
esp_foc_axis_get_current_pi_gains_q16(axis, &Kp, &Ki, &lim);
```

`retune` uses the same MPZ math as `gen_pi_gains.py`, implemented in
`source/motor_control/esp_foc_design_mpz.c` with pure Q16 arithmetic.

## Tuner protocol

With `CONFIG_ESP_FOC_TUNER_ENABLE` the firmware links a
transport-agnostic request handler (`esp_foc_tuner.c`) that speaks a
compact binary protocol on top of a framing codec
(`esp_foc_link.c` — sync byte, LE length, channel, seq, payload,
CRC-16/CCITT). Three channels are multiplexed over one bus:

- `TUNER` — request/response.
- `SCOPE` — CSV text from `esp_foc_scope.c` (one line per sample).
- `LOG` — reserved for future log relay.

Supported operations, mapped to `esp_foc_tuner_id_t`:

| Op    | Category         | IDs (hex)                                      |
|-------|------------------|------------------------------------------------|
| READ  | gains + state    | `0x10..0x13` (Kp, Ki, ILim, Vmax), `0x40..0x41` (state, last err) |
| WRITE | manual gains     | `0x20..0x22`                                   |
| WRITE | motion targets   | `0x50..0x53` (id, iq, ud, uq — only honored while override is ON) |
| EXEC  | commands         | `0x80` RECOMPUTE_GAINS, `0xA0` OVERRIDE_ON, `0xA1` OVERRIDE_OFF    |

Axis validation: every call checks an `axis->magic` field (present
only when the tuner is compiled in) so a stale or wild pointer is
refused immediately.

Override semantics: while `OVERRIDE_ON` is active the outer loop
still invokes the user's regulation callback but drops its writes on
the floor, honouring only the tuner shadow targets. `OVERRIDE_OFF`
restores user control bumplessly — no transient glitch at the swap.

## Transport bridges

Two bridges live under `source/drivers/`; pick one in menuconfig.
Both implement the weak callbacks declared in `esp_foc_tuner.h` and
`esp_foc_scope.c`, so tuner and scope share a single physical bus.

- `CONFIG_ESP_FOC_BRIDGE_UART` — works on every ESP32 variant
  (including the original ESP32 without native USB). Peripheral,
  baud and pin map are Kconfig entries.
- `CONFIG_ESP_FOC_BRIDGE_USBCDC` — S2/S3/P4 only. Uses the
  `espressif/esp_tinyusb` managed component; the same cable used
  for flashing carries the tuner traffic.

## tunerctl CLI

`tools/espfoc_studio/cli/tunerctl.py` is a thin argparse wrapper
around the Python `TunerClient` — handy when scripting bring-up
sequences or wiring the tuner into CI:

```bash
# axis state
python3 -m espfoc_studio.cli.tunerctl --port /dev/ttyACM0 axis-state

# read current gains
python3 -m espfoc_studio.cli.tunerctl --port /dev/ttyACM0 read

# recompute gains from motor params
python3 -m espfoc_studio.cli.tunerctl --port /dev/ttyACM0 \
      retune --r 1.08 --l 0.0018 --bw 150

# engage tuner-controlled motion
python3 -m espfoc_studio.cli.tunerctl --port /dev/ttyACM0 override on
python3 -m espfoc_studio.cli.tunerctl --port /dev/ttyACM0 set-target iq 1.5
python3 -m espfoc_studio.cli.tunerctl --port /dev/ttyACM0 override off

# alignment + calibration round-trip
python3 -m espfoc_studio.cli.tunerctl --port /dev/ttyACM0 align
python3 -m espfoc_studio.cli.tunerctl --port /dev/ttyACM0 \
      persist --r 1.08 --l 0.0018 --bw 150
python3 -m espfoc_studio.cli.tunerctl --port /dev/ttyACM0 firmware-type
```

Set `PYTHONPATH=tools` if you have not installed the package.

## Alignment with natural-direction probe

`esp_foc_align_axis()` parks the rotor at electrical 0, zeroes the
encoder, then drives a short positive `Vq` pulse (≈30 % of `V_max`
for 300 ms) and watches the raw encoder count to decide which way
the motor turned:

* delta ≥ +50 raw counts → `natural_direction = CW`
* delta ≤ −50 raw counts → `natural_direction = CCW`
* anything in between → keep the value passed in `settings`,
  log a warning. A stuck rotor (mechanical brake, open phase,
  dead encoder) lands here.

The probe is always on; the `settings.natural_direction` field
becomes a hint used only when the probe is inconclusive. Progress
shows up on the `LOG` link channel as
`alignment: started` → `alignment: complete (direction = ...)`,
which the TunerStudio Tuning panel echoes in its log viewer and
the CLI surfaces through `axis-state`.

## NVS calibration overlay

Once the loop is dialled in, persist the gains:

```c
#include "espFoC/esp_foc_calibration.h"

esp_foc_calibration_data_t cal = {
    .kp = my_kp_q16, .ki = my_ki_q16,
    .integrator_limit = my_lim_q16,
    .motor_r_ohm = q16_from_float(R),
    .motor_l_h   = q16_from_float(L),
    .bandwidth_hz = q16_from_float(BW),
};
esp_foc_calibration_save(0 /* axis_id */, &cal);
```

Or click **Save to NVS** in TunerStudio, or run
`tunerctl persist --r ... --l ... --bw ...`.

Every blob is tagged with a *profile hash*:

```
profile_hash = FNV-1a("<CONFIG_ESP_FOC_MOTOR_PROFILE>:<CONFIG_ESP_FOC_PROFILE_VERSION>")
```

`esp_foc_initialize_axis()` loads the overlay at boot only when the
stored hash matches the build's hash. Bumping
`CONFIG_ESP_FOC_PROFILE_VERSION` invalidates every persisted blob —
the guard rail that prevents loading gimbal calibration on a
scooter motor by accident.

The on-flash format is documented in
`source/motor_control/esp_foc_calibration.c`:

```
[magic=0xE5F0CC11][version=1][reserved][profile_hash:u32]
[crc32:u32][payload_len:u16][pad][payload bytes]
```

## TunerStudio target firmware

`examples/tuner_studio_target` is a service-mode firmware that does
nothing except host TunerStudio: it boots, parks the motor at zero
current, attaches the axis to the runtime tuner and waits. It also
overrides the weak `esp_foc_tuner_firmware_type()` hook to return
`'TSGX'` so the GUI can detect the target on connect and surface
the **Generate App** tab.

Pin map lives entirely in `Kconfig.projbuild`:

```
TUNER_TARGET_PWM_U_HI / V_HI / W_HI    high-side pins
TUNER_TARGET_PWM_U_LO / V_LO / W_LO    low-side pins
TUNER_TARGET_PWM_FREQ_HZ               PWM rate
TUNER_TARGET_DC_LINK_V                 link voltage
TUNER_TARGET_ENC_SDA / SCL             AS5600 I2C
TUNER_TARGET_POLE_PAIRS                motor identity
```

USB-CDC is the default transport on S2 / S3 / P4; switch to UART
under `Component config → espFoC Settings → Tuner transport bridge`
when targeting plain ESP32.

## Generate App from TunerStudio

The **Hardware** and **Generate App** tabs together turn a
TunerStudio session into a production-ready IDF project. Workflow:

1. Connect to the bring-up board running `tuner_studio_target`.
2. Use the Tuning + Analysis tabs to dial Kp/Ki/integrator_limit in.
3. Click **Save to NVS** so the bring-up board itself remembers
   the calibration (handy if you keep iterating).
4. Open the **Hardware** tab and fill in the production pin map.
5. Open the **Generate App** tab, type a project name, optionally
   override the output directory, click **Generate**.

The generator under `tools/espfoc_studio/codegen/sensored_app.py`
walks the `templates/sensored_app/*.tmpl` tree, substitutes the
live gains + Hardware values, and writes a self-contained IDF
project plus, when `nvs_partition_gen.py` is on `$IDF_PATH`, a
bit-exact `nvs_calibration.bin` you can flash into the production
NVS partition. The generated app keeps the runtime tuner enabled
so the operator can re-tune later without rebuilding firmware.

## Cross-validation Python ↔ C

`test/golden_motors.json` is the single source of truth for the
regression table. `scripts/verify_goldens.py` checks both that the
Python math agrees with the JSON and that the C mirror in
`test/test_design_mpz.c` matches byte-for-byte:

```
$ python3 scripts/verify_goldens.py
All 5 goldens agree across JSON, Python math, and C mirror.
```

Any drift in either side breaks the check immediately so the two
implementations cannot diverge silently.
