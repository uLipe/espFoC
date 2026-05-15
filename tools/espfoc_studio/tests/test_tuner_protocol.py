#!/usr/bin/env python3
"""End-to-end test of the Python TunerClient against a Python "echo
firmware" wired through an in-process loopback transport.

The fake firmware mirrors enough of the C reactor to validate the
protocol envelope, the seq matching, and the read/write/exec round-trips.
The real firmware-side codec is exercised separately by the Unity tests
(test_tuner_reactor.c). When the two suites agree, the wire format is
known to be byte-for-byte compatible.
"""

from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

from espfoc_studio.fake_tuner_loopback import FakeTunerLoopback
from espfoc_studio.link import LoopbackTransport
from espfoc_studio.protocol import (
    AxisStateFlag,
    TunerClient,
    TunerError,
)


def _setup():
    host_t, fw_t = LoopbackTransport.pair()
    fw = FakeTunerLoopback(fw_t)
    fw.start()
    cli = TunerClient(host_t)
    return cli, fw


def test_read_pi_gains():
    cli, fw = _setup()
    try:
        assert abs(cli.read_kp() - fw.kp) < 1e-3
        assert abs(cli.read_ki() - fw.ki) < 1e-3
        assert abs(cli.read_int_lim() - fw.lim) < 1e-3
        assert abs(cli.read_v_max() - fw.vmax) < 1e-3
    finally:
        fw.stop()


def test_write_then_read():
    cli, fw = _setup()
    try:
        cli.write_kp(2.5)
        cli.write_ki(123.4)
        assert abs(cli.read_kp() - 2.5) < 1e-3
        assert abs(cli.read_ki() - 123.4) < 1e-2
    finally:
        fw.stop()


def test_axis_state_flags():
    cli, fw = _setup()
    try:
        st = cli.read_axis_state()
        assert AxisStateFlag.INITIALIZED in st
        assert AxisStateFlag.ALIGNED in st
        assert AxisStateFlag.TUNER_OVERRIDE not in st
    finally:
        fw.stop()


def test_override_flow_then_motion():
    cli, fw = _setup()
    try:
        cli.override_on()
        st = cli.read_axis_state()
        assert AxisStateFlag.TUNER_OVERRIDE in st
        cli.write_target_iq(1.5)
        assert abs(fw.target_iq - 1.5) < 1e-3
        cli.override_off()
    finally:
        fw.stop()


def test_motion_rejected_when_override_off():
    cli, fw = _setup()
    try:
        try:
            cli.write_target_iq(1.0)
        except TunerError:
            return
        raise AssertionError("expected TunerError when override is off")
    finally:
        fw.stop()


def test_kd_kff_write_then_read():
    cli, fw = _setup()
    try:
        cli.write_kd(0.0001220703125)
        cli.write_kff(0.98)
        assert abs(cli.read_kd() - 0.0001220703125) < 1e-12
        assert abs(cli.read_kff() - 0.98) < 1e-3
    finally:
        fw.stop()


def test_reset_board():
    cli, fw = _setup()
    try:
        cli.reset_board()
    finally:
        fw.stop()


def test_ping():
    cli, fw = _setup()
    try:
        cli.ping()
    finally:
        fw.stop()


def test_motor_pole_pairs_round_trip():
    cli, fw = _setup()
    try:
        assert cli.read_motor_pole_pairs() == 7
        cli.write_motor_pole_pairs(11)
        assert cli.read_motor_pole_pairs() == 11
        assert fw.motor_pole_pairs == 11
    finally:
        fw.stop()


def main() -> int:
    tests = [
        test_read_pi_gains,
        test_write_then_read,
        test_axis_state_flags,
        test_override_flow_then_motion,
        test_motion_rejected_when_override_off,
        test_kd_kff_write_then_read,
        test_reset_board,
        test_ping,
        test_motor_pole_pairs_round_trip,
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
