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
import struct
import sys
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

from espfoc_studio.link import (
    Channel,
    Decoder,
    LoopbackTransport,
    Status,
    encode,
)
from espfoc_studio.protocol import (
    AxisStateFlag,
    Op,
    ParamId,
    TunerClient,
    TunerError,
)


class FakeFirmware(threading.Thread):
    """Spins on a transport, decodes tuner-channel frames and emits
    canned responses. Mirrors the dispatch logic of esp_foc_tuner.c
    closely enough to flush out wire-level mismatches."""

    def __init__(self, transport):
        super().__init__(daemon=True)
        self._t = transport
        self._dec = Decoder()
        self._stop = threading.Event()
        # State the fake firmware "owns".
        self.kp = 1.0
        self.ki = 50.0
        self.lim = 12.0
        self.vmax = 12.0
        self.aligned = True
        self.override = False
        self.skip_torque = False
        self.target_iq = 0.0
        self.motor_pole_pairs = 7

    def stop(self):
        self._stop.set()

    def _send_response(self, seq: int, status: int, payload: bytes = b""):
        body = struct.pack("<bB", status, seq) + payload
        self._t.send_bytes(encode(Channel.TUNER, seq, body))

    def _q16(self, v: float) -> bytes:
        raw = max(-0x80000000, min(0x7FFFFFFF, round(v * 65536.0)))
        return struct.pack("<i", raw)

    def _from_q16(self, data: bytes) -> float:
        return struct.unpack("<i", data)[0] / 65536.0

    def _dispatch(self, seq: int, payload: bytes):
        if len(payload) < 4:
            self._send_response(seq, -2)  # INVALID_ARG
            return
        op = payload[0]
        pid = payload[1] | (payload[2] << 8)
        # axis = payload[3]  # ignored by this fake
        cmd = payload[4:]

        if op == int(Op.READ):
            if pid == int(ParamId.KP_Q16):
                self._send_response(seq, 0, self._q16(self.kp))
            elif pid == int(ParamId.KI_Q16):
                self._send_response(seq, 0, self._q16(self.ki))
            elif pid == int(ParamId.INT_LIM_Q16):
                self._send_response(seq, 0, self._q16(self.lim))
            elif pid == int(ParamId.V_MAX_Q16):
                self._send_response(seq, 0, self._q16(self.vmax))
            elif pid == int(ParamId.AXIS_STATE):
                flags = AxisStateFlag.INITIALIZED
                if self.aligned:
                    flags |= AxisStateFlag.ALIGNED
                if self.override:
                    flags |= AxisStateFlag.TUNER_OVERRIDE
                self._send_response(seq, 0, bytes([int(flags)]))
            elif pid == int(ParamId.AXIS_LAST_ERR):
                self._send_response(seq, 0, struct.pack("<b", 0))
            elif pid == int(ParamId.MOTOR_POLE_PAIRS):
                self._send_response(
                    seq, 0, struct.pack("<i", int(self.motor_pole_pairs)))
            elif pid == int(ParamId.SKIP_TORQUE):
                self._send_response(
                    seq, 0, bytes([1 if self.skip_torque else 0]))
            else:
                self._send_response(seq, -2)
        elif op == int(Op.WRITE):
            if pid == int(ParamId.WRITE_SKIP_TORQUE):
                if len(cmd) < 1:
                    self._send_response(seq, -2)
                    return
                self.skip_torque = cmd[0] != 0
                self._send_response(seq, 0)
                return
            if len(cmd) < 4:
                self._send_response(seq, -2)
                return
            if pid == int(ParamId.WRITE_MOTOR_POLE_PAIRS):
                p = struct.unpack("<i", cmd[:4])[0]
                if not (1 <= p <= 64):
                    self._send_response(seq, -2)
                    return
                self.motor_pole_pairs = p
                self._send_response(seq, 0)
                return
            v = self._from_q16(cmd[:4])
            if pid == int(ParamId.WRITE_KP):
                self.kp = v
                self._send_response(seq, 0)
            elif pid == int(ParamId.WRITE_KI):
                self.ki = v
                self._send_response(seq, 0)
            elif pid == int(ParamId.WRITE_INT_LIM):
                self.lim = v
                self._send_response(seq, 0)
            elif pid == int(ParamId.WRITE_TARGET_IQ):
                if not self.override:
                    self._send_response(seq, -3)  # AXIS_INVALID_STATE
                    return
                self.target_iq = v
                self._send_response(seq, 0)
            else:
                self._send_response(seq, -2)
        elif op == int(Op.EXEC):
            if pid == int(ParamId.CMD_OVERRIDE_ON):
                if not self.aligned:
                    self._send_response(seq, -1)
                    return
                self.override = True
                self._send_response(seq, 0)
            elif pid == int(ParamId.CMD_OVERRIDE_OFF):
                self.override = False
                self._send_response(seq, 0)
            elif pid == int(ParamId.CMD_RECOMPUTE_GAINS):
                if len(cmd) < 12:
                    self._send_response(seq, -2)
                    return
                # Mock MPZ recompute: just reflect that the call landed.
                self.kp = self._from_q16(cmd[0:4]) * 5.0
                self.ki = self._from_q16(cmd[4:8]) * 1000.0
                self._send_response(seq, 0)
            elif pid == int(ParamId.CMD_RESET_BOARD):
                self._send_response(seq, 0)
            else:
                self._send_response(seq, -2)
        else:
            self._send_response(seq, -2)

    def run(self):
        while not self._stop.is_set():
            data = self._t.read_bytes(64, timeout=0.05)
            if not data:
                continue
            for b in data:
                st = self._dec.push(b)
                if st == Status.OK:
                    if self._dec.channel == int(Channel.TUNER):
                        self._dispatch(self._dec.seq, bytes(self._dec.payload))
                    self._dec.reset()


def _setup():
    host_t, fw_t = LoopbackTransport.pair()
    fw = FakeFirmware(fw_t)
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


def test_recompute_gains():
    cli, fw = _setup()
    try:
        cli.recompute_gains(1.08, 0.0018, 150.0)
        assert abs(fw.kp - 1.08 * 5.0) < 1e-3
        assert abs(fw.ki - 0.0018 * 1000.0) < 1e-3
    finally:
        fw.stop()


def test_reset_board():
    cli, fw = _setup()
    try:
        cli.reset_board()
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
        test_recompute_gains,
        test_reset_board,
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
