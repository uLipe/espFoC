"""Scope sample payload: binary Q16.16 packed as int32 LE (default firmware).

If the target was built with ``CONFIG_ESP_FOC_SCOPE_LEGACY_CSV=y`` (or you are
testing old code), set env ``ESP_FOC_STUDIO_SCOPE_CSV=1`` or use
``python -m espfoc_studio.gui --scope-csv`` so comma-separated float lines
are still decoded. Otherwise only SCOPE v1 is parsed (no CSV branch)."""

from __future__ import annotations

import os
import struct
from typing import List, Optional

# Match legacy firmware: enable CSV decode only when set to "1" (Kconfig
# has no direct equivalent on the host).
_ENV_CSV = "ESP_FOC_STUDIO_SCOPE_CSV"

# Wire: 0xFF + 'S','C', v1. First byte 0xFF avoids collision with CSV digits.
SCOPE_WIRE_V1 = 0x01
SCOPE_WIRE_HEADER = bytes((0xFF, 0x53, 0x43, SCOPE_WIRE_V1))
HEADER_LEN = 4
COUNT_OFS = 4
PAYLOAD_OFS = 6


def pack_scope_i32_to_payload(samples_i32: List[int]) -> bytes:
    """Build a SCOPE *channel* frame body for tests / demo. Each value is
    a signed 32-bit Q16.16 mantissa, same as ``q16_t`` in firmware."""
    n = len(samples_i32)
    out = bytearray(SCOPE_WIRE_HEADER)
    out.extend(struct.pack("<H", n & 0xFFFF))
    for s in samples_i32:
        out.extend(struct.pack("<i", s))
    return bytes(out)


def legacy_csv_decoding_enabled() -> bool:
    return os.environ.get(_ENV_CSV, "0") == "1"


def decode_scope_payload_to_floats(payload: bytes) -> Optional[List[float]]:
    """If *payload* is a binary SCOPE v1 block, return float samples (Q16.16
    to float). Otherwise return ``None``."""
    if len(payload) < PAYLOAD_OFS:
        return None
    if not payload.startswith(SCOPE_WIRE_HEADER):
        return None
    n = struct.unpack_from("<H", payload, COUNT_OFS)[0]
    if n < 1:
        return None
    need = PAYLOAD_OFS + 4 * n
    if len(payload) < need:
        return None
    out: List[float] = []
    for k in range(n):
        v = struct.unpack_from("<i", payload, PAYLOAD_OFS + 4 * k)[0]
        out.append(v / 65536.0)
    return out


def decode_scope_payload_to_floats_csv_first(payload: bytes) -> List[float]:
    """Binary SCOPE v1 first; optional CSV if :func:`legacy_csv_decoding_enabled`."""
    b = decode_scope_payload_to_floats(payload)
    if b is not None:
        return b
    if not legacy_csv_decoding_enabled():
        return []
    try:
        line = payload.decode("ascii", errors="ignore").strip()
    except (UnicodeDecodeError, ValueError):
        return []
    if not line:
        return []
    return [float(tok) for tok in line.split(",") if tok]
