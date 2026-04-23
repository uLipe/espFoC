"""Transport I/O errors become TunerError (GUI can treat like timeouts)."""
from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

import pytest  # type: ignore

from espfoc_studio.link import LinkReader, Transport
from espfoc_studio.protocol import TunerClient, TunerError


def test_send_oserror_is_tuner_error() -> None:
    class Busted(Transport):
        def send_bytes(self, data: bytes) -> None:
            raise OSError(5, "Input/output error")

        def read_bytes(self, max_bytes: int, timeout=None) -> bytes:  # noqa: ARG002
            return b""

        def close(self) -> None:
            pass

    r = LinkReader(Busted())
    r.start()
    try:
        c = TunerClient(r, axis=0)
        with pytest.raises(TunerError) as einfo:
            c.read_kp()
        assert "link I/O" in str(einfo.value)
    finally:
        r.stop()


def test_send_serial_exception_is_tuner_error() -> None:
    try:
        from serial import SerialException
    except ImportError:
        pytest.skip("pyserial not installed")

    class Busted(Transport):
        def send_bytes(self, data: bytes) -> None:
            raise SerialException("write failed: [Errno 5] Input/output error")

        def read_bytes(self, max_bytes: int, timeout=None) -> bytes:  # noqa: ARG002
            return b""

        def close(self) -> None:
            pass

    r = LinkReader(Busted())
    r.start()
    try:
        c = TunerClient(r, axis=0)
        with pytest.raises(TunerError) as einfo:
            c.read_kp()
        assert "link I/O" in str(einfo.value)
    finally:
        r.stop()
