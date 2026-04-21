"""Wire-level framing codec — mirror of source/motor_control/esp_foc_link.c.

Frame layout (must stay byte-for-byte identical to the firmware):

    offset  bytes  field
    -----   -----  -----
    0       1      sync = 0xA5
    1       2      payload_len (LE)
    3       1      channel
    4       1      seq
    5       N      payload
    5+N     2      crc16 (LE, CRC-16/CCITT over channel..payload)

The Python side stays sync-friendly (no asyncio) so the same code is
reusable from a CLI, a Qt thread, or a pytest harness.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional


SYNC = 0xA5
HEADER_BYTES = 5  # sync + len(2) + channel + seq
TRAILER_BYTES = 2  # crc16
MAX_PAYLOAD = 256
MAX_FRAME = HEADER_BYTES + MAX_PAYLOAD + TRAILER_BYTES


class Channel(IntEnum):
    TUNER = 0x01
    SCOPE = 0x02
    LOG = 0x03


class Status(IntEnum):
    OK = 0
    INVALID_ARG = -1
    TOO_BIG = -2
    NEED_MORE = -3
    BAD_SYNC = -4
    BAD_CRC = -5


class LinkError(Exception):
    """Raised when an encode call cannot satisfy the request."""


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT (poly 0x1021, init 0xFFFF, no reflection, no xor-out).

    Bit-by-bit implementation that matches esp_foc_link_crc16() byte-for-byte.
    """
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode(channel: int, seq: int, payload: bytes = b"") -> bytes:
    """Encode a single frame and return the wire bytes."""
    if not isinstance(payload, (bytes, bytearray)):
        raise LinkError("payload must be bytes-like")
    if len(payload) > MAX_PAYLOAD:
        raise LinkError(f"payload too big: {len(payload)} > {MAX_PAYLOAD}")
    if not (0 <= channel <= 0xFF) or not (0 <= seq <= 0xFF):
        raise LinkError("channel and seq must be u8")

    n = len(payload)
    head = bytes([
        SYNC,
        n & 0xFF, (n >> 8) & 0xFF,
        channel & 0xFF,
        seq & 0xFF,
    ])
    body = bytes(payload)
    crc = crc16_ccitt(head[3:] + body)
    trailer = bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    return head + body + trailer


_DEC_WAIT_SYNC = 0
_DEC_LEN_LO = 1
_DEC_LEN_HI = 2
_DEC_CHANNEL = 3
_DEC_SEQ = 4
_DEC_PAYLOAD = 5
_DEC_CRC_LO = 6
_DEC_CRC_HI = 7
_DEC_DONE = 8


@dataclass
class Decoder:
    """Streaming frame decoder. Mirrors esp_foc_link_decoder_t state."""

    state: int = _DEC_WAIT_SYNC
    channel: int = 0
    seq: int = 0
    payload_len: int = 0
    payload: bytearray = field(default_factory=bytearray)
    crc_recv: int = 0
    crc_calc: int = 0xFFFF

    def reset(self) -> None:
        self.state = _DEC_WAIT_SYNC
        self.channel = 0
        self.seq = 0
        self.payload_len = 0
        self.payload = bytearray()
        self.crc_recv = 0
        self.crc_calc = 0xFFFF

    @staticmethod
    def _crc_update(crc: int, byte: int) -> int:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
        return crc

    def push(self, byte: int) -> Status:
        """Feed one byte. Returns OK when a complete frame has been parsed."""
        if not 0 <= byte <= 0xFF:
            raise LinkError("byte must be 0..255")
        if self.state == _DEC_DONE:
            self.reset()

        if self.state == _DEC_WAIT_SYNC:
            if byte != SYNC:
                return Status.NEED_MORE
            self.state = _DEC_LEN_LO
            return Status.NEED_MORE
        if self.state == _DEC_LEN_LO:
            self.payload_len = byte
            self.state = _DEC_LEN_HI
            return Status.NEED_MORE
        if self.state == _DEC_LEN_HI:
            self.payload_len |= byte << 8
            if self.payload_len > MAX_PAYLOAD:
                self.reset()
                return Status.TOO_BIG
            self.state = _DEC_CHANNEL
            return Status.NEED_MORE
        if self.state == _DEC_CHANNEL:
            self.channel = byte
            self.crc_calc = self._crc_update(self.crc_calc, byte)
            self.state = _DEC_SEQ
            return Status.NEED_MORE
        if self.state == _DEC_SEQ:
            self.seq = byte
            self.crc_calc = self._crc_update(self.crc_calc, byte)
            self.state = _DEC_CRC_LO if self.payload_len == 0 else _DEC_PAYLOAD
            return Status.NEED_MORE
        if self.state == _DEC_PAYLOAD:
            self.payload.append(byte)
            self.crc_calc = self._crc_update(self.crc_calc, byte)
            if len(self.payload) >= self.payload_len:
                self.state = _DEC_CRC_LO
            return Status.NEED_MORE
        if self.state == _DEC_CRC_LO:
            self.crc_recv = byte
            self.state = _DEC_CRC_HI
            return Status.NEED_MORE
        if self.state == _DEC_CRC_HI:
            self.crc_recv |= byte << 8
            if self.crc_recv != self.crc_calc:
                self.reset()
                return Status.BAD_CRC
            self.state = _DEC_DONE
            return Status.OK

        self.reset()
        return Status.BAD_SYNC

    def feed(self, data: bytes) -> Optional["Decoder"]:
        """Convenience helper: feed many bytes, return self when a frame
        completes, or None when more bytes are needed."""
        for b in data:
            st = self.push(b)
            if st == Status.OK:
                return self
            if st < 0 and st != Status.NEED_MORE:
                raise LinkError(f"decode error: {st.name}")
        return None
