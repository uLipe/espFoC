"""ESPF scope frame v1 decoder — doc/PIVOT_SCOPE_SHELL.md."""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

HDR = bytes((0x45, 0x46, 0x50, 0x46))
FTR = bytes((0x46, 0x4E, 0x44, 0x21))


class SyncStatus(IntEnum):
    NEED_MORE = 0
    FRAME = 1


@dataclass(frozen=True)
class ScopeFrame:
    seq: int
    n_ch: int
    samples_q16: tuple[int, ...]


class StreamDecoder:
    """Byte-oriented resyncing decoder for ESPF frames."""

    def __init__(self, expected_n_ch: Optional[int] = None) -> None:
        self._buf = bytearray()
        self._expected_n_ch = expected_n_ch

    def feed(self, data: bytes) -> list[ScopeFrame]:
        if data:
            self._buf.extend(data)
        out: list[ScopeFrame] = []
        while True:
            frame = self._try_one()
            if frame is None:
                break
            out.append(frame)
        return out

    def _try_one(self) -> Optional[ScopeFrame]:
        buf = self._buf
        start = buf.find(HDR)
        if start < 0:
            if len(buf) > 3:
                del buf[:-3]
            return None
        if start > 0:
            del buf[:start]
        if len(buf) < 8:
            return None
        seq, n_ch = struct.unpack_from("<HH", buf, 4)
        if self._expected_n_ch is not None and n_ch != self._expected_n_ch:
            del buf[0:1]
            return None
        total = 12 + 4 * n_ch
        if len(buf) < total:
            return None
        if buf[total - 4:total] != FTR:
            del buf[0:1]
            return None
        samples = struct.unpack_from(f"<{n_ch}i", buf, 8)
        del buf[:total]
        return ScopeFrame(seq=seq, n_ch=n_ch, samples_q16=samples)


# Alias for doc cross-reference
ESPF_HDR = HDR
ESPF_FTR = FTR
ESPF_HEADER_BYTES = 8
