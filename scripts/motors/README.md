# Motor profiles

Each `*.json` file in this directory describes a motor and the design targets
the build-time autotuner (`scripts/gen_pi_gains.py`) uses to compute current-loop
PI gains via Matched Pole-Zero (MPZ) synthesis.

Select the active profile with the Kconfig option `ESP_FOC_MOTOR_PROFILE`
(for example `default`, picking `default.json`).

## Schema

```json
{
  "name": "human-readable description",
  "comment": "free-form notes (optional)",
  "motor": {
    "resistance_ohm": 1.08,        // phase resistance R, ohms
    "inductance_h":   0.0018,      // phase inductance L, henries
    "pole_pairs":     11           // for reference only (not used by autotuner)
  },
  "control": {
    "current_bw_hz":  150.0,       // closed-loop bandwidth target, hertz
    "v_max_volts":    12.0         // output saturation, volts (anti-windup)
  },
  "safety": {
    "min_phase_margin_deg": 30.0   // build aborts if PM (with 1-sample delay) is below this
  }
}
```

## Q16 precision caveat

Inductances below ~0.5 mH suffer from the 15 uH LSB of Q16.16. The autotuner
warns in the report when L is small; use manual gain entry via
`esp_foc_axis_set_current_pi_gains_q16()` for ultra-low-L motors until the
Q24/Q32 design path lands.

## Why motor profiles are not committed-by-build

The autotuner outputs a header into the **build directory**, never into the
source tree. This keeps generated artifacts out of git and lets each user/CI
job freely pick a different motor without polluting commits. The script also
refuses to write into `source/` or `include/` unless `--allow-in-tree` is
passed explicitly.
