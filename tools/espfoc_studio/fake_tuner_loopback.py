"""Minimal in-process tuner responder over :class:`LoopbackTransport`.

Used by host-side tests to exercise :class:`TunerClient` without hardware.
Not a plant simulator — only protocol round-trips."""

from __future__ import annotations

import struct
import threading

from .link import Channel, Decoder, Status, encode
from .protocol import AxisStateFlag, Op, ParamId


class FakeTunerLoopback(threading.Thread):
    """Decodes tuner-channel frames and emits canned responses."""

    def __init__(self, transport) -> None:
        super().__init__(daemon=True)
        self._t = transport
        self._dec = Decoder()
        self._stop = threading.Event()
        self.kp = 1.0
        self.ki = 50.0
        self.kd = 0.0
        self.kff = 1.0
        self.lim = 12.0
        self.vmax = 12.0
        self.aligned = True
        self.override = False
        self.target_iq = 0.0
        self.motor_pole_pairs = 7

    def stop(self) -> None:
        self._stop.set()

    def _send_response(self, seq: int, status: int, payload: bytes = b"") -> None:
        body = struct.pack("<bB", status, seq) + payload
        self._t.send_bytes(encode(Channel.TUNER, seq, body))

    def _q16(self, v: float) -> bytes:
        raw = max(-0x80000000, min(0x7FFFFFFF, round(v * 65536.0)))
        return struct.pack("<i", raw)

    def _from_q16(self, data: bytes) -> float:
        return struct.unpack("<i", data)[0] / 65536.0

    def _dispatch(self, seq: int, payload: bytes) -> None:
        if len(payload) < 4:
            self._send_response(seq, -2)
            return
        op = payload[0]
        pid = payload[1] | (payload[2] << 8)
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
            elif pid == int(ParamId.KD_Q16):
                self._send_response(seq, 0, self._q16(self.kd))
            elif pid == int(ParamId.KFF_Q16):
                self._send_response(seq, 0, self._q16(self.kff))
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
            else:
                self._send_response(seq, -2)
        elif op == int(Op.WRITE):
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
            elif pid == int(ParamId.WRITE_KD):
                self.kd = v
                self._send_response(seq, 0)
            elif pid == int(ParamId.WRITE_KFF):
                self.kff = v
                self._send_response(seq, 0)
            elif pid == int(ParamId.WRITE_INT_LIM):
                self.lim = v
                self._send_response(seq, 0)
            elif pid == int(ParamId.WRITE_TARGET_IQ):
                if not self.override:
                    self._send_response(seq, -3)
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
            elif pid == int(ParamId.CMD_RESET_BOARD):
                self._send_response(seq, 0)
            elif pid == int(ParamId.CMD_PING):
                self._send_response(seq, 0)
            else:
                self._send_response(seq, -2)
        else:
            self._send_response(seq, -2)

    def run(self) -> None:
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
