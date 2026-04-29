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

# read or program the per-phase current LPF cutoff (Hz)
python3 -m espfoc_studio.cli.tunerctl --port /dev/ttyACM0 cutoff
python3 -m espfoc_studio.cli.tunerctl --port /dev/ttyACM0 cutoff --set 500
```

Set `PYTHONPATH=tools` if you have not installed the package.

## Current sense filtering

Every phase current sample lands inside the isensor driver and is
pushed straight through a 2nd-order Butterworth low-pass biquad
(`include/espFoC/utils/biquad_q16.h`) before the gain conversion.
This is the single shaping stage between the ADC counts and the
Clarke / Park / PI pipeline: the EMA on i_q / i_d that 2.x kept
between Park and the PI is gone, and so is the moving-average ring
that used to sit on top of the ADC samples themselves. One filter,
one place, no buried double smoothing.

The biquad runs in Direct-Form-II Transposed in Q16 (5 multiplies,
4 adds, no internal accumulator drift). Coefficients come from a
prewarped bilinear design:

```
r  = tan(pi * fc / fs)
a0 = 1 + sqrt(2)*r + r^2
b0 = b2 = r^2 / a0
b1 = 2*r^2 / a0
a1 = 2*(r^2 - 1) / a0
a2 = (1 - sqrt(2)*r + r^2) / a0
```

DC gain is exactly unity by construction; the design places a
perfect zero at Nyquist (Butterworth LPF has H(-1) = 0). All
coefficients live in [-2, +2], so Q16.16 keeps ~14 bits of
headroom for any sane fc / fs ratio.

Boot precedence for the cutoff, identical to the gain pipeline:

1. NVS calibration overlay (when `current_filter_fc_hz != 0`).
2. `CONFIG_ESP_FOC_CURRENT_FILTER_CUTOFF_HZ` (default 300 Hz).

The runtime tuner can override at any time
(`WRITE_I_FILTER_FC_Q16` or `tunerctl cutoff --set`); persisting
through `CMD_PERSIST_NVS` stamps the live value into the
calibration blob so the next boot picks it back up.

The same biquad replaces the EMA across the rest of the library:
the velocity estimate uses `CONFIG_ESP_FOC_VELOCITY_FILTER_CUTOFF_HZ`
(default 50 Hz), and the PLL / KF observers swap their internal
EMAs for biquads with the same nominal cutoffs they used before.
Operators with hand-tuned observer cutoffs may want to revisit:
the new filter has the same -3 dB position but a sharper roll-off
(40 dB/decade vs 20 dB/decade) and slightly different phase.

## ISR hot path (Plan #2)

Set `CONFIG_ESP_FOC_ISR_HOT_PATH=y` (default) and the entire FOC
inner loop — Park, current PI, inverse Park, SVPWM, `set_voltages`
— runs inside the MCPWM timer ISR at the full PWM rate (40 kHz
default). The user `regulation_callback` keeps running on its
FreeRTOS task at `pwm_rate / ESP_FOC_LOW_SPEED_DOWNSAMPLING`
(2 kHz default), so user code rate is unchanged. The current PI's
sample rate climbs from 2 kHz to 40 kHz: a ~20x bandwidth headroom
for free, no faster ADC needed.

Three pieces work together:

* **ADC ISR** (`source/drivers/current_sensor_adc.c` and the P4
  one-shot variant): every fresh sample goes through the per-phase
  Butterworth biquad, then the driver applies offset + gain +
  Clarke and atomic-writes `i_alpha` / `i_beta` into the
  axis-supplied sinks via the new `set_publish_targets()` interface
  method. The PWM ISR can then read both currents with a single
  load each — no critical section, no fetch_isensors call.

* **PWM ISR** (`foc_hot_isr` in
  `source/motor_control/control_strategy/esp_foc_control_current_mode_sensored.c`):
  reads the published currents, uses **`rotor_elec_angle`** for Park,
  inverse Park, and SVPWM, runs the current PI (unless voltage mode),
  calls `set_voltages`, optionally pushes a scope frame via
  `esp_foc_scope_data_push_from_isr()`, then decrements the
  downsample counter and notifies the outer task on rollover.

* **Outer task** (same symbol
  `do_current_mode_sensored_low_speed_loop`): waits on the ISR's
  notification, reads the slow encoder (AS5600 over I2C, AS5048
  over SPI, or PCNT-direct), updates **`rotor_elec_angle`**, runs the velocity smoothing
  biquad, then notifies the regulator task so the user
  `regulation_callback` can refresh the targets.

An **alpha-beta angle tracker** (`include/espFoC/utils/angle_predictor_q16.h`)
exists for experiments and is covered by unit tests; it is **not** integrated
into this ISR pipeline.

When the ADC sample is older than one PWM period (drop / 50 kHz cap
on plain ESP32 with PWM > 25 kHz), the ISR reuses the latest
published value. The PI integrates one cycle of slightly stale
current — way milder degradation than the legacy 2 kHz loop already
lives with.

Set `CONFIG_ESP_FOC_ISR_HOT_PATH=n` to fall back to the legacy 2.x
task-based path. Useful while a sensorless observer config that has
not been ported to the ISR path yet is in use.

### Why not Plan #3 (offload sin/cos to the other core)?

Investigated and dropped:

* `esp_ipc_isr_call` exists in IDF v5.5 but on Xtensa (ESP32 /
  ESP32-S3) the callback must be **assembly** with no C calls
  allowed — the LUT-based `q16_sin/cos` in `iq31_sin/cos` cannot
  run there.
* The API offers only `_call` (busy-wait until the other core
  starts) and `_call_blocking` (busy-wait until it finishes); no
  asynchronous "done" callback back to the originating core, so
  the desired `PWM ISR -> dispatch -> sincos done IRQ -> hot path`
  pipeline cannot be assembled from the public IDF surface.
* `q16_sin + q16_cos` together are a few hundred cycles (LUT 8192
  + IQ31 conversions). The IPI round-trip would dwarf that.

Documented for the record so we do not revisit it without new
information.

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
`'TSGX'` for host identification.

PWM carrier / loop timing: `ESP_FOC_PWM_RATE_HZ` under **espFoC Settings → Core**
(shared with MCPWM drivers and build-time PI autotuning). Pin map is in
`main/Kconfig.projbuild`:

```
TUNER_TARGET_PWM_U_HI / V_HI / W_HI    high-side pins
TUNER_TARGET_PWM_U_LO / V_LO / W_LO    low-side pins
TUNER_TARGET_DC_LINK_V                 link voltage
TUNER_TARGET_ENC_SDA / SCL             AS5600 I2C
TUNER_TARGET_POLE_PAIRS                motor identity
```

USB-CDC is the default transport on S2 / S3 / P4; switch to UART
under `Component config → espFoC Settings → Tuner transport bridge`
when targeting plain ESP32.

## Generate App (GUI) and offline codegen

In the PySide GUI, **Hardware** + **Generate App** are available only when you
start TunerStudio with **`python -m espfoc_studio.gui --demo`** (embedded
`DemoFirmware`). A normal **`--port`** session shows tuning, analysis, scope,
and SVM tabs without codegen.

`generate_sensored_app()` in `tools/espfoc_studio/codegen/sensored_app.py`
walks the `templates/sensored_app/*.tmpl` tree, substitutes motor /
gain / hardware fields from `HardwareConfig`, and writes a self-contained IDF
project plus, when `nvs_partition_gen.py` is on `$IDF_PATH`, a
bit-exact `nvs_calibration.bin` you can flash into the production
NVS partition. The generated app keeps the runtime tuner enabled
so the operator can re-tune later without rebuilding firmware.

## Debugging: small `Uq` but SVPWM hits the rails / motor buzzes

Symptoms: you command a modest **feed-forward** voltage on the Q axis
(`WRITE_TARGET_UQ`, e.g. 0.1 “volts” in the tuner wire format) with
`Ud = 0`, expect smooth torque and rotation, but the three phase duties
look stuck near **0 % or 100 %** and the shaft **vibrates**.

If **`skip_torque_control` is always on** (open-loop `Ud`/`Uq` only), the
current PI does **not** run — ignore §1 below and focus on **DC-link
normalization (§2), electrical angle / encoder (§3), and hardware**.

### 1. When current PI *is* enabled: feed-forward vs PI output

Skip this subsection if you already run **open-loop voltage only**
(`skip_torque_control == 1`).

Unless `CONFIG_ESP_FOC_TUNER_ALWAYS_OVERRIDE_VOLTAGE_MODE` is enabled,
`axis->skip_torque_control` defaults to **0** at init — the **current
controllers are active**. In that mode `target_u_q` is **not** the only
term that drives the inverter: the Q-axis PI computes voltage from
`(target_i_q − i_q)` and **adds** `target_u_q` as feed-forward:

```c
u_q = pid(target_i_q, i_q) + target_u_q;  // sensored ISR path
```

So even with `Uq = 0.1`, **noise, wrong Clarke/Park polarity, or a large
`target_i_q`** can push the PI output to **`±max_voltage`**, which clips
in `esp_foc_limit_voltage_q16` and shows up as SVPWM **saturation**.

**Bring-up checklist:** in TunerStudio / host, enable **“Open-loop voltage
(no current PI)”** (`WRITE_SKIP_TORQUE`) and set **`target_i_q = 0`**
when you only want to probe voltage injection. Re-read `skip_torque`
from the device to confirm.

### 2. DC-link volts vs SVPWM duty (`set_voltages`)

The MCPWM drivers report **`get_dc_link_voltage()`** from the nominal rail passed to
`inverter_*_new(..., dc_link_voltage, ...)` (or from **`ESP_FOC_DC_LINK_NOMINAL_MV`**
in menuconfig when that argument is ≤ 0). **`esp_foc_initialize_axis`** uses it so
**Ud/Uq from the tuner (volts)** share the same engineering basis as **`1/Vbus`**
inside `esp_foc_svm_set`. Historically the drivers wrongly returned **`Q16_ONE`**
(1 V) while ignoring the constructor — so e.g. **0.1 V** looked like **10 %** of a
1 V bus instead of **0.1/24** of a 24 V bus.

**`esp_foc_modulate_dq_voltage`** outputs **three phase voltages** (same units as
Ud/Uq, typically volts Q16). **`set_voltages`** receives those **phase voltages**;
the MCPWM driver applies **`esp_foc_svm_phase_volts_to_duties`** internally using
**`get_dc_link_voltage()`**, then writes comparator ticks for **[0, 1]** duty.

### 3. Angle / encoder

The PWM ISR uses **`rotor_elec_angle`** from the outer loop each tick (encoder
rate is slower than PWM — expect some torque ripple at high bandwidth unless the
sensor is fast enough).

Wrong **pole pairs**, **CPR**, **alignment offset**, or a noisy angle stream
still causes **torque ripple** and wrong torque direction (for example
negative **Vq** not opposing positive **Vq** as expected): verify θe vs mechanical rotation on a
scope, **natural_direction**, UVW wiring, and alignment. Open-loop **Vq** only
tracks torque sign if θe matches the real field angle.

### 4. What to put on the scope

Wire channels for **`u_u`, `u_v`, `u_w`** (normalized duties), **`u_q`**
command if exposed, and rotor-related quantities from your `esp_foc_scope`
tap points. If **`u_q` looks tiny** but phase duties are rail-to-rail:
with **current PI off**, look at **angle / encoder**, scaling, or the
inverter path — not the PI. With **current PI on**, also suspect
**integrator wind-up** and noisy `i_q`.

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
