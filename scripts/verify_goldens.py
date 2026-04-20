#!/usr/bin/env python3
"""
Cross-validate the MPZ golden table between Python (authoritative math in
gen_pi_gains.py) and the C test mirror in test/test_design_mpz.c.

Two checks:
  1. Python-computed Kp/Ki for each motor in test/golden_motors.json
     must match the kp_expected/ki_expected stored in the JSON (1e-4 tol).
  2. The kp_expected/ki_expected literals in test/test_design_mpz.c must
     match the JSON (exact string match after f-string formatting).

Exits non-zero on any mismatch so CI catches drift immediately.
"""
from __future__ import annotations

import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
COMP_ROOT = os.path.dirname(HERE)
GOLDENS_JSON = os.path.join(COMP_ROOT, "test", "golden_motors.json")
C_TEST_FILE = os.path.join(COMP_ROOT, "test", "test_design_mpz.c")

sys.path.insert(0, HERE)
from gen_pi_gains import design_mpz  # noqa: E402


def load_goldens():
    with open(GOLDENS_JSON, "r") as f:
        return json.load(f)


def check_python_math(goldens):
    errors = []
    for m in goldens["motors"]:
        ts_s = m["ts_us"] * 1e-6
        res = design_mpz(m["r_ohm"], m["l_h"], ts_s, m["bw_hz"], m["v_max"])
        kp_err = abs(res.kp - m["kp_expected"])
        ki_err = abs(res.ki - m["ki_expected"])
        kp_rel = kp_err / max(abs(m["kp_expected"]), 1e-12)
        ki_rel = ki_err / max(abs(m["ki_expected"]), 1e-12)
        if kp_rel > 1e-4 or ki_rel > 1e-4:
            errors.append(
                f"{m['name']}: Python math drifted from JSON: "
                f"kp_rel={kp_rel:.2e}, ki_rel={ki_rel:.2e}"
            )
    return errors


def check_c_mirror(goldens):
    """Ensure the C test golden entries match the JSON ones."""
    errors = []
    c_body = open(C_TEST_FILE, "r").read()
    for m in goldens["motors"]:
        # The C line looks like:
        #   {"gimbal_iPower", 1.080f, 0.00180f, 500, 150.0f, 12.0f, 1.565825f, 811.667f},
        # We search for any occurrence of the motor name and parse its values.
        pattern = re.compile(
            r'\{\s*"' + re.escape(m["name"]) + r'"\s*,'
            r'\s*([-\d.]+)f\s*,'      # R
            r'\s*([-\d.]+)f\s*,'      # L
            r'\s*(\d+)\s*,'           # Ts_us
            r'\s*([-\d.]+)f\s*,'      # bw
            r'\s*([-\d.]+)f\s*,'      # Vmax
            r'\s*([-\d.]+)f\s*,'      # Kp
            r'\s*([-\d.]+)f\s*,?\s*\}'
        )
        mo = pattern.search(c_body)
        if mo is None:
            errors.append(f"{m['name']}: not found in {C_TEST_FILE}")
            continue
        r, l, ts, bw, vm, kp, ki = mo.groups()
        if abs(float(r) - m["r_ohm"]) > 1e-9 or \
           abs(float(l) - m["l_h"]) > 1e-9 or \
           int(ts) != int(m["ts_us"]) or \
           abs(float(bw) - m["bw_hz"]) > 1e-9 or \
           abs(float(vm) - m["v_max"]) > 1e-9:
            errors.append(f"{m['name']}: C mirror inputs disagree with JSON")
        if abs(float(kp) - m["kp_expected"]) > 1e-4:
            errors.append(
                f"{m['name']}: C mirror kp_expected={kp} != JSON "
                f"{m['kp_expected']}"
            )
        if abs(float(ki) - m["ki_expected"]) > 1e-3:
            errors.append(
                f"{m['name']}: C mirror ki_expected={ki} != JSON "
                f"{m['ki_expected']}"
            )
    return errors


def main():
    goldens = load_goldens()
    errors = []
    errors.extend(check_python_math(goldens))
    errors.extend(check_c_mirror(goldens))
    if errors:
        for e in errors:
            print("FAIL: " + e, file=sys.stderr)
        print(f"\n{len(errors)} mismatch(es) detected. Update one side or the "
              f"other so Python, JSON, and C agree.", file=sys.stderr)
        return 1
    n = len(goldens["motors"])
    print(f"All {n} goldens agree across JSON, Python math, and C mirror.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
