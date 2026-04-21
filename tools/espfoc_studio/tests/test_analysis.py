#!/usr/bin/env python3
"""Unit tests for tools/espfoc_studio/model/analysis.py.

Pure math, no Qt; safe to run in CI or headless dev boxes.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

from espfoc_studio.model import (
    MotorParams,
    PiGains,
    bode,
    closed_loop_tf,
    loop_gain_tf,
    mpz_design,
    pole_zero_map,
    root_locus,
    step_response,
)


def test_mpz_design_matches_gen_pi_gains():
    """Design helper must agree with scripts/gen_pi_gains.py on the
    reference gimbal motor."""
    motor = MotorParams(r_ohm=1.08, l_h=0.0018, ts_s=0.001, v_max=12.0)
    g = mpz_design(motor, bandwidth_hz=150.0)
    # Values match the golden iPower gimbal entry.
    assert abs(g.kp - 1.460955) < 2e-3
    assert abs(g.ki - 659.166) < 1.0


def test_step_response_reaches_reference():
    motor = MotorParams(r_ohm=1.08, l_h=0.0018, ts_s=0.0001, v_max=24.0)
    g = mpz_design(motor, bandwidth_hz=200.0)
    t, i = step_response(motor, g, n_samples=4000, reference=1.0)
    steady = np.mean(i[-50:])
    assert abs(steady - 1.0) < 0.05, f"did not settle: steady={steady}"


def test_step_response_rise_time_matches_bandwidth():
    """First-order target: 63.2% at t = 1/w_bw."""
    bw = 200.0
    motor = MotorParams(r_ohm=1.08, l_h=0.0018, ts_s=0.0001, v_max=24.0)
    g = mpz_design(motor, bandwidth_hz=bw)
    t, i = step_response(motor, g, n_samples=4000, reference=1.0)
    idx = np.searchsorted(i, 0.632)
    t_meas = t[idx]
    t_expected = 1.0 / (2.0 * math.pi * bw)
    rel = abs(t_meas - t_expected) / t_expected
    # ~13% typical residual because the firmware PI is a delayed
    # forward-Euler, introducing a 1-sample lag over the ideal target.
    assert rel < 0.20, f"rise-time off: meas={t_meas}, expected={t_expected}"


def test_bode_crosses_0db_near_bandwidth():
    """Loop-gain magnitude crosses 0 dB near the designed bandwidth."""
    bw = 150.0
    motor = MotorParams(r_ohm=1.08, l_h=0.0018, ts_s=0.001, v_max=12.0)
    g = mpz_design(motor, bandwidth_hz=bw)
    f, mag, _ = bode(motor, g, n_points=400)
    crossover = f[np.argmin(np.abs(mag))]
    assert 0.5 * bw <= crossover <= 1.5 * bw, (
        f"crossover {crossover} Hz should be near {bw} Hz"
    )


def test_closed_loop_poles_inside_unit_circle():
    motor = MotorParams(r_ohm=1.08, l_h=0.0018, ts_s=0.001, v_max=12.0)
    g = mpz_design(motor, bandwidth_hz=150.0)
    _, _, cl_poles, _ = pole_zero_map(motor, g)
    # All closed-loop poles must be strictly inside the unit circle.
    assert np.all(np.abs(cl_poles) < 1.0), (
        f"CL pole outside unit circle: {cl_poles}"
    )


def test_pi_zero_cancels_plant_pole():
    motor = MotorParams(r_ohm=1.08, l_h=0.0018, ts_s=0.001, v_max=12.0)
    g = mpz_design(motor, bandwidth_hz=150.0)
    _, ol_zeros, _, _ = pole_zero_map(motor, g)
    alpha = math.exp(-motor.r_ohm * motor.ts_s / motor.l_h)
    # One of the open-loop zeros comes from the PI and must sit at alpha.
    assert any(abs(z - alpha) < 1e-6 for z in ol_zeros), (
        f"no PI zero near alpha={alpha} in {ol_zeros}"
    )


def test_root_locus_shrinks_for_lower_kp():
    motor = MotorParams(r_ohm=1.08, l_h=0.0018, ts_s=0.001, v_max=12.0)
    g = mpz_design(motor, bandwidth_hz=150.0)
    ks, mat = root_locus(motor, g, scale_min=0.1, scale_max=2.0, n_points=50)
    # At Kp scale ~ 0.1 poles should be slower (further from 0) than at 2.0;
    # for this first-order-with-cancellation structure they approach z=1
    # as Kp decreases.
    slow = np.nanmax(np.abs(mat[0]))
    fast = np.nanmax(np.abs(mat[-1]))
    assert slow > fast, (
        f"expected slower pole for lower Kp: slow={slow}, fast={fast}"
    )


def test_loop_gain_tf_shape():
    motor = MotorParams(r_ohm=1.08, l_h=0.0018, ts_s=0.001, v_max=12.0)
    g = mpz_design(motor, bandwidth_hz=150.0)
    num, den = loop_gain_tf(motor, g)
    # 2nd-order numerator, 2nd-order denominator.
    assert len(num) == 3
    assert len(den) == 3


def test_closed_loop_tf_shape():
    motor = MotorParams(r_ohm=1.08, l_h=0.0018, ts_s=0.001, v_max=12.0)
    g = mpz_design(motor, bandwidth_hz=150.0)
    num, den = closed_loop_tf(motor, g)
    assert len(num) == 3
    assert len(den) == 3


def main() -> int:
    tests = [
        test_mpz_design_matches_gen_pi_gains,
        test_step_response_reaches_reference,
        test_step_response_rise_time_matches_bandwidth,
        test_bode_crosses_0db_near_bandwidth,
        test_closed_loop_poles_inside_unit_circle,
        test_pi_zero_cancels_plant_pole,
        test_root_locus_shrinks_for_lower_kp,
        test_loop_gain_tf_shape,
        test_closed_loop_tf_shape,
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
