"""Transport abstractions for the espFoC link layer.

Each transport class exposes a synchronous send_bytes / read_bytes pair.
Higher-level code (protocol clients, GUI) picks one based on the bus the
target firmware was built against (UART, USB-CDC, ...).
"""

from __future__ import annotations

from typing import Optional


class Transport:
    """Bare interface every concrete transport implements."""

    def send_bytes(self, data: bytes) -> None:
        raise NotImplementedError

    def read_bytes(self, max_bytes: int, timeout: Optional[float] = None) -> bytes:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError

    def __enter__(self) -> "Transport":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


class LoopbackTransport(Transport):
    """In-process transport that pipes the host side back to itself.

    Useful for unit tests: pair two LoopbackTransport instances and any
    bytes written into one show up on the other's read queue. The pair
    is created via LoopbackTransport.pair().
    """

    def __init__(self) -> None:
        self._tx_to: Optional["LoopbackTransport"] = None
        self._rx_buf = bytearray()

    @classmethod
    def pair(cls) -> tuple["LoopbackTransport", "LoopbackTransport"]:
        a, b = cls(), cls()
        a._tx_to = b
        b._tx_to = a
        return a, b

    def send_bytes(self, data: bytes) -> None:
        if self._tx_to is None:
            raise RuntimeError("LoopbackTransport is unpaired")
        self._tx_to._rx_buf.extend(data)

    def read_bytes(self, max_bytes: int, timeout: Optional[float] = None) -> bytes:
        n = min(max_bytes, len(self._rx_buf))
        chunk = bytes(self._rx_buf[:n])
        del self._rx_buf[:n]
        return chunk

    def close(self) -> None:
        self._tx_to = None
        self._rx_buf.clear()
