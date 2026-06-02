"""Serial RX thread + ESPF decode (passive scope stream)."""

from __future__ import annotations

import threading
import time
from typing import Callable, List, Optional

from .frame import StreamDecoder


class StreamReader:
    def __init__(
        self,
        port: str,
        baud: int = 921600,
        expected_n_ch: int = 17,
        on_frames: Optional[Callable[[list], None]] = None,
  ) -> None:
        self._port = port
        self._baud = baud
        self._expected_n_ch = expected_n_ch
        self._on_frames = on_frames
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._serial = None
        self._error: Optional[str] = None

    @property
    def error(self) -> Optional[str]:
        return self._error

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="espfoc-rx", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def _run(self) -> None:
        try:
            import serial
        except ImportError as e:
            self._error = f"pyserial required: {e}"
            return
        dec = StreamDecoder(expected_n_ch=self._expected_n_ch)
        t0 = time.monotonic()
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baud,
                timeout=0.05,
            )
            self._serial.reset_input_buffer()
            self._serial.dtr = True
            self._serial.rts = False
            while not self._stop.is_set():
                chunk = self._serial.read(512)
                if not chunk:
                    continue
                frames = dec.feed(chunk)
                if frames and self._on_frames is not None:
                    self._on_frames(frames)
        except Exception as e:
            self._error = str(e)
        finally:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None


def list_serial_ports() -> List[str]:
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    out: List[str] = []
    for info in list_ports.comports():
        if info.device:
            out.append(info.device)
    return sorted(out, key=_port_sort_key)


def _port_sort_key(dev: str) -> tuple:
    score = 0
    dl = dev.lower()
    if "acm" in dl:
        score -= 10
    if "jtag" in dl:
        score -= 20
    if "usb" in dl:
        score -= 5
    return (score, dl)
