"""USB discovery and serial session lifecycle for espFoC Tool."""

from __future__ import annotations

import time
from typing import Callable, Optional

from PySide6.QtCore import QObject, QTimer, Signal

from ..link import LinkReader
from ..link.transport_serial import SerialTransport
from ..protocol import TunerClient, TunerError
from ..protocol.tuner import TUNER_FIRMWARE_TYPE_TSGX

SCAN_INTERVAL_MS = 2000
CONNECT_PROBE_TIMEOUT_S = 0.35


def list_candidate_ports() -> list[str]:
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    out: list[str] = []
    for info in list_ports.comports():
        dev = info.device
        if not dev:
            continue
        desc = (info.description or "").lower()
        if "bluetooth" in desc:
            continue
        out.append(dev)
    return out


def probe_port(port: str, baud: int, axis: int) -> Optional[TunerClient]:
    """Try CONNECT on *port*; return a live client or None."""
    transport: Optional[SerialTransport] = None
    reader: Optional[LinkReader] = None
    try:
        transport = SerialTransport(port=port, baud=baud)
        reader = LinkReader(transport)
        reader.start()
        client = TunerClient(reader, axis=axis)
        info = client.connect(timeout=CONNECT_PROBE_TIMEOUT_S)
        if info.firmware_type != TUNER_FIRMWARE_TYPE_TSGX:
            client.disconnect()
            return None
        return client
    except (TunerError, OSError, Exception):
        if reader is not None:
            try:
                reader.stop()
            except Exception:
                pass
        if transport is not None:
            try:
                transport.close()
            except Exception:
                pass
        return None


class ConnectionManager(QObject):
    """Background scan when offline; holds the active :class:`TunerClient`."""

    state_changed = Signal(str)
    client_ready = Signal(object)
    client_lost = Signal()
    port_descr_changed = Signal(str)

    STATE_NO_DEVICE = "NO_DEVICE"
    STATE_SCANNING = "SCANNING"
    STATE_CONNECTING = "CONNECTING"
    STATE_CONNECTED = "CONNECTED"

    def __init__(
        self,
        baud: int = 921600,
        axis: int = 0,
        fixed_port: Optional[str] = None,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self._baud = baud
        self._axis = axis
        self._fixed_port = fixed_port
        self._client: Optional[TunerClient] = None
        self._state = self.STATE_NO_DEVICE
        self._scan_timer = QTimer(self)
        self._scan_timer.setInterval(SCAN_INTERVAL_MS)
        self._scan_timer.timeout.connect(self._on_scan_tick)
        self._busy = False

    @property
    def client(self) -> Optional[TunerClient]:
        return self._client

    @property
    def connected(self) -> bool:
        return self._client is not None

    @property
    def state(self) -> str:
        return self._state

    def start(self) -> None:
        if self._fixed_port:
            self._set_state(self.STATE_CONNECTING)
            self.port_descr_changed.emit(
                f"{self._fixed_port} @ {self._baud}")
            self._try_open(self._fixed_port)
            return
        self._set_state(self.STATE_SCANNING)
        self.port_descr_changed.emit("Scanning USB…")
        self._scan_timer.start()
        self._on_scan_tick()

    def stop(self) -> None:
        self._scan_timer.stop()
        self._release_client()

    def _set_state(self, state: str) -> None:
        if self._state == state:
            return
        self._state = state
        self.state_changed.emit(state)

    def _release_client(self) -> None:
        if self._client is None:
            return
        try:
            self._client.disconnect()
        except Exception:
            pass
        try:
            self._client.reader.stop()
        except Exception:
            pass
        self._client = None
        self.client_lost.emit()
        self._set_state(self.STATE_NO_DEVICE)

    def _try_open(self, port: str) -> bool:
        if self._busy:
            return False
        self._busy = True
        try:
            if self._client is not None:
                return True
            self._set_state(self.STATE_CONNECTING)
            client = probe_port(port, self._baud, self._axis)
            if client is None:
                return False
            self._client = client
            self._set_state(self.STATE_CONNECTED)
            self.port_descr_changed.emit(f"{port} @ {self._baud}")
            self.client_ready.emit(client)
            self._scan_timer.stop()
            return True
        finally:
            self._busy = False

    def _on_scan_tick(self) -> None:
        if self._fixed_port or self._client is not None or self._busy:
            return
        ports = list_candidate_ports()
        if not ports:
            self._set_state(self.STATE_SCANNING)
            self.port_descr_changed.emit("No USB serial device")
            return
        self._set_state(self.STATE_SCANNING)
        for port in ports:
            if self._try_open(port):
                return
        self.port_descr_changed.emit(
            f"Scanning ({len(ports)} port(s))…")

    def replace_client_reader(self, setup: Callable[[TunerClient], None]) -> None:
        """After serial reconnect, *setup* rebinds scope subscribers."""
        if self._client is not None:
            setup(self._client)
