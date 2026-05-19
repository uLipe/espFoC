"""Minimal in-process tuner responder over LoopbackTransport (protocol v2)."""

from __future__ import annotations

import struct
import threading
import time

from .link import Channel, Decoder, Status, encode
from .protocol import (
    HB_MSG_ACK,
    HB_MSG_FW,
    LINK_PROTO_VER,
    Op,
    ParamId,
)


class FakeTunerLoopback(threading.Thread):
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
        self.running = False
        self.scope_on = False
        self.connected = False
        self.target_iq = 0.0
        self.motor_pole_pairs = 7
        self._hb_counter = 0
        self._hb_miss = 0
        self._hb_armed = False
        self._got_ack_this_period = False

    def stop(self) -> None:
        self._stop.set()

    def _send_response(self, seq: int, status: int, payload: bytes = b"") -> None:
        body = struct.pack("<bB", status, seq) + payload
        self._t.send_bytes(encode(Channel.TUNER, seq, body))

    def _send_hb(self) -> None:
        if not self.connected:
            return
        if self._hb_armed and not self._got_ack_this_period:
            self._hb_miss += 1
            if self._hb_miss >= 2:
                self.connected = False
                return
        self._got_ack_this_period = False
        self._hb_armed = True
        self._hb_counter = (self._hb_counter + 1) & 0xFF
        flags = 1
        if self.aligned:
            flags |= 2
        if self.running:
            flags |= 4
        body = bytes([HB_MSG_FW, self._hb_counter, flags, 0])
        self._t.send_bytes(encode(Channel.HEARTBEAT, 0, body))

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

        if op == int(Op.EXEC) and pid == int(ParamId.CMD_CONNECT):
            if self.connected:
                return
            if len(cmd) < 3 or cmd[0] != LINK_PROTO_VER:
                self._send_response(seq, -2)
                return
            self.connected = True
            self._hb_counter = 0
            self._hb_miss = 0
            self._hb_armed = False
            self._got_ack_this_period = False
            ack = bytes([
                LINK_PROTO_VER,
                0x54, 0x53, 0x47, 0x58,
                1, 8,
                0xF4, 0x01,
            ])
            self._send_response(seq, 0, ack)
            return

        if not self.connected:
            return

        if op == int(Op.EXEC) and pid == int(ParamId.CMD_DISCONNECT):
            self.connected = False
            self.scope_on = False
            self._send_response(seq, 0)
            return

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
                flags = 1
                if self.aligned:
                    flags |= 2
                if self.running:
                    flags |= 4
                self._send_response(seq, 0, bytes([flags]))
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
                if not self.running:
                    self._send_response(seq, -3)
                    return
                self.target_iq = v
                self._send_response(seq, 0)
            else:
                self._send_response(seq, -2)
        elif op == int(Op.EXEC):
            if pid == int(ParamId.CMD_RUN_AXIS):
                if not self.aligned:
                    self._send_response(seq, -1)
                    return
                self.running = True
                self._send_response(seq, 0)
            elif pid == int(ParamId.CMD_STOP_AXIS):
                self.running = False
                self._send_response(seq, 0)
            elif pid == int(ParamId.CMD_SCOPE_START):
                self.scope_on = True
                self._send_response(seq, 0)
            elif pid == int(ParamId.CMD_SCOPE_STOP):
                self.scope_on = False
                self._send_response(seq, 0)
            elif pid == int(ParamId.CMD_RESET_BOARD):
                self._send_response(seq, 0)
            else:
                self._send_response(seq, -2)

    def run(self) -> None:
        next_hb = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            if now >= next_hb:
                self._send_hb()
                next_hb = now + 0.5
            data = self._t.read_bytes(64, timeout=0.05)
            if not data:
                continue
            for b in data:
                st = self._dec.push(b)
                if st == Status.OK:
                    if self._dec.channel == int(Channel.TUNER):
                        self._dispatch(self._dec.seq, bytes(self._dec.payload))
                    elif self._dec.channel == int(Channel.HEARTBEAT):
                        pl = bytes(self._dec.payload)
                        if len(pl) >= 2 and pl[0] == HB_MSG_ACK:
                            self._got_ack_this_period = True
                            self._hb_miss = 0
                    self._dec.reset()
