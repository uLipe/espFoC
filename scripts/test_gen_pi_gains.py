#!/usr/bin/env python3
"""
Sanity tests for scripts/gen_pi_gains.py.

Run from the espFoC component root:
    python3 scripts/test_gen_pi_gains.py
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "gen_pi_gains.py")
DEFAULT_PROFILE = os.path.join(HERE, "motors", "default.json")


def run(args, expect_rc=0, capture=True):
    proc = subprocess.run(
        [sys.executable, SCRIPT, *args],
        capture_output=capture, text=True
    )
    if proc.returncode != expect_rc:
        print("STDOUT:", proc.stdout, file=sys.stderr)
        print("STDERR:", proc.stderr, file=sys.stderr)
        raise AssertionError(
            f"Expected rc={expect_rc}, got {proc.returncode} for args={args}"
        )
    return proc


def test_basic_run():
    with tempfile.TemporaryDirectory() as tmp:
        out_h = os.path.join(tmp, "gains.h")
        out_txt = os.path.join(tmp, "report.txt")
        run([
            "--motor", DEFAULT_PROFILE,
            "--output", out_h,
            "--report", out_txt,
            "--pwm-rate-hz", "20000",
            "--decimation", "20",
        ])
        assert os.path.isfile(out_h), "header not generated"
        assert os.path.isfile(out_txt), "report not generated"
        body = open(out_h).read()
        for sym in ("ESP_FOC_AUTOGEN_CURRENT_KP_Q16",
                    "ESP_FOC_AUTOGEN_CURRENT_KI_Q16",
                    "ESP_FOC_AUTOGEN_CURRENT_INT_LIM_Q16",
                    "ESP_FOC_AUTOGEN_ALPHA_Q16",
                    "ESP_FOC_AUTOGEN_BETA_Q16"):
            assert sym in body, f"missing {sym} in generated header"


def test_invalid_json():
    with tempfile.TemporaryDirectory() as tmp:
        bad = os.path.join(tmp, "bad.json")
        open(bad, "w").write("{not json")
        run([
            "--motor", bad,
            "--output", os.path.join(tmp, "h.h"),
            "--pwm-rate-hz", "20000",
            "--decimation", "20",
        ], expect_rc=3)


def test_missing_field():
    with tempfile.TemporaryDirectory() as tmp:
        bad = os.path.join(tmp, "incomplete.json")
        open(bad, "w").write(json.dumps({
            "motor": {"resistance_ohm": 1.0},
            "control": {"current_bw_hz": 100, "v_max_volts": 12}
        }))
        run([
            "--motor", bad,
            "--output", os.path.join(tmp, "h.h"),
            "--pwm-rate-hz", "20000",
            "--decimation", "20",
        ], expect_rc=3)


def test_path_guard_rejects_source():
    with tempfile.TemporaryDirectory() as tmp:
        # Construct a forbidden path like /tmp/.../source/foo.h
        forbidden = os.path.join(tmp, "source", "foo.h")
        os.makedirs(os.path.dirname(forbidden), exist_ok=True)
        run([
            "--motor", DEFAULT_PROFILE,
            "--output", forbidden,
            "--pwm-rate-hz", "20000",
            "--decimation", "20",
        ], expect_rc=2)


def test_path_guard_override():
    with tempfile.TemporaryDirectory() as tmp:
        forbidden = os.path.join(tmp, "source", "foo.h")
        os.makedirs(os.path.dirname(forbidden), exist_ok=True)
        run([
            "--motor", DEFAULT_PROFILE,
            "--output", forbidden,
            "--pwm-rate-hz", "20000",
            "--decimation", "20",
            "--allow-in-tree",
        ])
        assert os.path.isfile(forbidden)


def test_phase_margin_abort():
    """Push the bandwidth so high that PM (with delay) drops below threshold,
    while still staying within Nyquist (so the PM guard fires, not Nyquist)."""
    with tempfile.TemporaryDirectory() as tmp:
        prof = os.path.join(tmp, "aggressive.json")
        open(prof, "w").write(json.dumps({
            "name": "aggressive",
            "motor": {"resistance_ohm": 1.0,
                      "inductance_h": 0.001,
                      "pole_pairs": 4},
            "control": {"current_bw_hz": 1000.0, "v_max_volts": 12.0},
            "safety": {"min_phase_margin_deg": 60.0}
        }))
        run([
            "--motor", prof,
            "--output", os.path.join(tmp, "h.h"),
            "--pwm-rate-hz", "20000",
            "--decimation", "4",  # Ts=200us, Nyquist=2500Hz (bw=1000Hz fits)
        ], expect_rc=5)


def test_nyquist_violation():
    with tempfile.TemporaryDirectory() as tmp:
        # Ts = 20/20000 = 1ms, Nyquist = 500Hz; pick bw = 800Hz
        prof = os.path.join(tmp, "nyquist.json")
        open(prof, "w").write(json.dumps({
            "name": "above_nyquist",
            "motor": {"resistance_ohm": 1.0, "inductance_h": 0.001,
                      "pole_pairs": 4},
            "control": {"current_bw_hz": 800.0, "v_max_volts": 12.0}
        }))
        run([
            "--motor", prof,
            "--output", os.path.join(tmp, "h.h"),
            "--pwm-rate-hz", "20000",
            "--decimation", "20",
        ], expect_rc=4)


def main():
    tests = [
        test_basic_run,
        test_invalid_json,
        test_missing_field,
        test_path_guard_rejects_source,
        test_path_guard_override,
        test_phase_margin_abort,
        test_nyquist_violation,
    ]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"OK    {t.__name__}")
        except Exception as e:
            failed += 1
            print(f"FAIL  {t.__name__}: {e}")
    if failed:
        print(f"\n{failed} test(s) failed", file=sys.stderr)
        return 1
    print(f"\nAll {len(tests)} tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
