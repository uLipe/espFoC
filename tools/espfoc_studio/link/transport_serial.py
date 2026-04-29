"""Serial-port transport (works for both UART and USB-CDC).

pyserial is imported lazily so unit tests for the codec / protocol layers
do not need pyserial installed.
"""

from __future__ import annotations

from typing import Optional

from .transport import Transport


class SerialTransport(Transport):
    def __init__(self, port: str, baud: int = 921600,
                 read_timeout: float = 0.5,
                 write_timeout: float = 1.0) -> None:
        try:
            import serial  # type: ignore
        except ImportError as e:
            raise RuntimeError(
                "SerialTransport requires pyserial (pip install pyserial)"
            ) from e
        self._serial = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=read_timeout,
            write_timeout=write_timeout,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        # Some USB-CDC stacks discard initial bytes; flush garbage.
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()

    def send_bytes(self, data: bytes) -> None:
        if not data:
            return
        try:
            self._serial.write(data)
            self._serial.flush()
        except (OSError, TypeError, AttributeError, ValueError) as e:
            raise OSError(f"serial send failed: {e}") from e

    def read_bytes(self, max_bytes: int, timeout: Optional[float] = None) -> bytes:
        if timeout is not None:
            old = self._serial.timeout
            self._serial.timeout = timeout
            try:
                return self._serial.read(max_bytes)
            finally:
                self._serial.timeout = old
        return self._serial.read(max_bytes)

    def close(self) -> None:
        try:
            self._serial.close()
        except Exception:
            pass
