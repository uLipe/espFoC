#!/usr/bin/env python3
"""Cross-validation suite for the Python link codec.

Two roles:
  * Pure Python tests for encoder/decoder semantics (round-trip,
    error paths, garbage skipping).
  * Golden vectors that MUST agree byte-for-byte with the firmware
    (test/test_link.c). Any drift here means firmware and host can
    no longer talk to each other.

Run from the component root:
    PYTHONPATH=tools python3 tools/espfoc_studio/tests/test_link_codec.py
"""

from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

from espfoc_studio.link import (
    Channel,
    Decoder,
    LinkError,
    MAX_PAYLOAD,
    Status,
    crc16_ccitt,
    encode,
)


def _expect_ok(status: Status, label: str) -> None:
    if status != Status.OK:
        raise AssertionError(f"{label}: expected OK, got {status.name}")


def test_crc_golden_vectors():
    """Must match the values asserted in test/test_link.c."""
    assert crc16_ccitt(b"") == 0xFFFF
    assert crc16_ccitt(b"\x00") == 0xE1F0
    assert crc16_ccitt(b"HELLO") == 0x49D6


def test_encode_roundtrip_small():
    pl = bytes([0xDE, 0xAD, 0xBE, 0xEF])
    frame = encode(Channel.TUNER, 0x42, pl)
    dec = Decoder()
    for b in frame[:-1]:
        assert dec.push(b) == Status.NEED_MORE
    assert dec.push(frame[-1]) == Status.OK
    assert dec.channel == int(Channel.TUNER)
    assert dec.seq == 0x42
    assert bytes(dec.payload) == pl


def test_encode_roundtrip_empty():
    frame = encode(Channel.LOG, 0)
    dec = Decoder()
    for b in frame[:-1]:
        assert dec.push(b) == Status.NEED_MORE
    assert dec.push(frame[-1]) == Status.OK
    assert dec.payload_len == 0
    assert bytes(dec.payload) == b""


def test_encode_roundtrip_max_payload():
    pl = bytes((i ^ 0x55) & 0xFF for i in range(MAX_PAYLOAD))
    frame = encode(Channel.SCOPE, 0xFF, pl)
    dec = Decoder()
    completed = dec.feed(frame)
    assert completed is dec
    assert dec.channel == int(Channel.SCOPE)
    assert dec.seq == 0xFF
    assert bytes(dec.payload) == pl


def test_encode_rejects_oversized_payload():
    try:
        encode(Channel.TUNER, 0, bytes(MAX_PAYLOAD + 1))
    except LinkError:
        return
    raise AssertionError("expected LinkError")


def test_decoder_skips_garbage_until_sync():
    pl = bytes([0x10, 0x20])
    frame = encode(Channel.TUNER, 7, pl)
    dec = Decoder()
    for noise in [0x00, 0x11, 0xFF, 0x12, 0x34]:
        assert dec.push(noise) == Status.NEED_MORE
    completed = dec.feed(frame)
    assert completed is dec
    assert dec.seq == 7
    assert bytes(dec.payload) == pl


def test_decoder_flags_crc_corruption():
    pl = bytes([0xCA, 0xFE])
    frame = bytearray(encode(Channel.SCOPE, 1, pl))
    frame[-1] ^= 0xFF  # flip CRC byte
    dec = Decoder()
    statuses = [dec.push(b) for b in frame]
    # All earlier bytes either NEED_MORE; the final one must be BAD_CRC.
    assert statuses[-1] == Status.BAD_CRC


def test_decoder_recovers_after_bad_frame():
    pl = bytes([0x01, 0x02, 0x03])
    frame = encode(Channel.TUNER, 9, pl)
    dec = Decoder()
    # corrupt header (claims oversized payload)
    assert dec.push(0xA5) == Status.NEED_MORE
    assert dec.push(0xFF) == Status.NEED_MORE
    assert dec.push(0xFF) == Status.TOO_BIG
    # decoder must be reset; valid frame must parse
    assert dec.feed(frame) is dec
    assert bytes(dec.payload) == pl


def test_cross_validation_against_firmware_pattern():
    """The same payload encoded by Python must produce identical bytes
    to what the firmware would emit. We re-derive the expected output by
    construction (no firmware needed) so the test is self-contained."""
    pl = bytes(range(8))
    seq = 0x10
    chan = Channel.TUNER
    out = encode(chan, seq, pl)
    # Header
    assert out[0] == 0xA5
    assert out[1] == len(pl) & 0xFF
    assert out[2] == (len(pl) >> 8) & 0xFF
    assert out[3] == int(chan)
    assert out[4] == seq
    # Payload
    assert out[5:5 + len(pl)] == pl
    # CRC over channel..payload (LE)
    crc = crc16_ccitt(out[3:5 + len(pl)])
    assert out[-2] == crc & 0xFF
    assert out[-1] == (crc >> 8) & 0xFF


def main() -> int:
    tests = [
        test_crc_golden_vectors,
        test_encode_roundtrip_small,
        test_encode_roundtrip_empty,
        test_encode_roundtrip_max_payload,
        test_encode_rejects_oversized_payload,
        test_decoder_skips_garbage_until_sync,
        test_decoder_flags_crc_corruption,
        test_decoder_recovers_after_bad_frame,
        test_cross_validation_against_firmware_pattern,
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
