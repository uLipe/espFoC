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
whenever both `motor_resistance` and `motor_inductance` are left at
zero in the settings struct, so the application picks up the tuned
values without touching runtime code.

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
```

Set `PYTHONPATH=tools` if you have not installed the package.

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
