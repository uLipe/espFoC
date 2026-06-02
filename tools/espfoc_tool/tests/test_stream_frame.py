#!/usr/bin/env python3
"""ESPF frame codec tests."""

from __future__ import annotations

import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

from espfoc_tool.stream.frame import FTR, HDR, StreamDecoder


def _pack_frame(seq: int, samples: list[int]) -> bytes:
    n = len(samples)
    body = HDR + struct.pack("<HH", seq, n)
    for s in samples:
        body += struct.pack("<i", s)
    body += FTR
    return body


def test_roundtrip():
    raw = _pack_frame(7, [65536, -32768, 0])
    dec = StreamDecoder(expected_n_ch=3)
    frames = dec.feed(raw)
    assert len(frames) == 1
    assert frames[0].seq == 7
    assert frames[0].samples_q16 == (65536, -32768, 0)


def test_resync_after_garbage():
    dec = StreamDecoder(expected_n_ch=1)
    junk = b"\xff\xff" + _pack_frame(1, [100]) + _pack_frame(2, [200])
    frames = dec.feed(junk)
    assert len(frames) == 2
    assert frames[0].seq == 1
    assert frames[1].seq == 2
