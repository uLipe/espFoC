#!/usr/bin/env python3
"""End-to-end test of TunerClient against fake firmware (protocol v2)."""

from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

from espfoc_tool.fake_tuner_loopback import FakeTunerLoopback
from espfoc_tool.link import LoopbackTransport
from espfoc_tool.protocol import (
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


def test_connect_and_read():
    cli, fw = _setup()
    try:
        info = cli.connect()
        assert info.firmware_type == 0x58475354
        assert abs(cli.read_kp() - fw.kp) < 1e-3
    finally:
        fw.stop()


def test_disconnect():
    cli, fw = _setup()
    try:
        cli.connect()
        cli.disconnect()
        assert not cli.connected
    finally:
        fw.stop()


def test_command_requires_session():
    cli, fw = _setup()
    try:
        try:
            cli.read_kp()
        except TunerError:
            return
        raise AssertionError("expected TunerError when not connected")
    finally:
        fw.stop()


def test_run_flow_then_motion():
    cli, fw = _setup()
    try:
        cli.connect()
        cli.run_axis()
        st = cli.read_axis_state()
        assert AxisStateFlag.RUNNING in st
        cli.write_target_iq(1.5)
        assert abs(fw.target_iq - 1.5) < 1e-3
        cli.stop_axis()
    finally:
        fw.stop()


def test_scope_start_stop():
    cli, fw = _setup()
    try:
        cli.connect()
        cli.scope_start()
        assert fw.scope_on
        cli.scope_stop()
        assert not fw.scope_on
    finally:
        fw.stop()


def main() -> int:
    tests = [
        test_connect_and_read,
        test_disconnect,
        test_command_requires_session,
        test_run_flow_then_motion,
        test_scope_start_stop,
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
