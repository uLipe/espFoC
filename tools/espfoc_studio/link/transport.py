"""Transport abstractions for the espFoC link layer.

Each transport class exposes a synchronous send_bytes / read_bytes pair.
Higher-level code (protocol clients, GUI) picks one based on the bus the
target firmware was built against (UART, USB-CDC, ...).
"""

from __future__ import annotations

import threading
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

    Useful for unit tests and the GUI's --demo mode: pair two
    LoopbackTransport instances and any bytes written into one show up
    on the other's read queue.

    read_bytes uses a Condition so a caller waiting for data does not
    busy-loop; in practice this keeps the GUI event loop responsive
    (otherwise two threads spinning on idle reads starve every other
    Python thread including Qt's).
    """

    def __init__(self) -> None:
        self._tx_to: Optional["LoopbackTransport"] = None
        self._rx_buf = bytearray()
        self._cond = threading.Condition()
        self._closed = False

    @classmethod
    def pair(cls) -> tuple["LoopbackTransport", "LoopbackTransport"]:
        a, b = cls(), cls()
        a._tx_to = b
        b._tx_to = a
        return a, b

    def send_bytes(self, data: bytes) -> None:
        if self._tx_to is None:
            raise RuntimeError("LoopbackTransport is unpaired")
        peer = self._tx_to
        with peer._cond:
            if peer._closed:
                return
            peer._rx_buf.extend(data)
            peer._cond.notify_all()

    def read_bytes(self, max_bytes: int, timeout: Optional[float] = None) -> bytes:
        if max_bytes <= 0:
            return b""
        with self._cond:
            if not self._rx_buf and timeout is not None and timeout > 0:
                # wait for data or timeout; Condition.wait returns early
                # when notified so round-trips feel instant.
                self._cond.wait(timeout=timeout)
            n = min(max_bytes, len(self._rx_buf))
            chunk = bytes(self._rx_buf[:n])
            del self._rx_buf[:n]
            return chunk

    def close(self) -> None:
        with self._cond:
            self._closed = True
            self._rx_buf.clear()
            self._cond.notify_all()
        self._tx_to = None
