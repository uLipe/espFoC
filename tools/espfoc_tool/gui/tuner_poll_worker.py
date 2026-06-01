"""Background tuner polling — serial round-trips off the Qt GUI thread.

The LinkReader still owns RX on its own thread; TunerClient round-trips block
the caller while holding ``_bus_lock``. All such traffic for startup/reconnect
primers and periodic polls runs on this QObject's thread so the GUI thread never
blocks on that mutex (view stays responsive; protocol is model/controller)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from PySide6.QtCore import QObject, QTimer, Signal, Slot

from ..protocol import AxisStateFlag, TunerClient, TunerError


@dataclass(frozen=True)
class TunerPollSnapshot:
    kp: float
    ki: float
    lim: float
    fc: float
    vmax: float
    state: int
    override_active: bool
    cal_present: bool
    last_poll_ok: bool


_LINK_PING_INTERVAL_MS = 5000


class TunerPollWorker(QObject):
    """Runs :meth:`poll_tick` on a dedicated thread (owned by MainWindow).

    Emits :signal:`poll_finished` back to the GUI thread (QueuedConnection).
    Link liveness uses :signal:`ping_finished` on a separate timer so poll
    failures do not imply NO LINK.
    """

    poll_finished = Signal(bool, str, object)
    """ok, error message (empty if ok), TunerPollSnapshot or None."""

    ping_finished = Signal(bool, str)
    """ok, error message (empty if ok)."""

    device_reads_ready = Signal(float, object, object)
    """loop_fs_hz, NVS shadow tuple or None, pole_pairs or None — for GUI thread only."""

    suspend_requested = Signal(bool)

    def __init__(self, client: TunerClient) -> None:
        super().__init__()
        self._client = client
        self._poll_serial = 0
        self._want_full = True
        self._last_vmax = 0.0
        self._cal_present = False
        self._prev_override_active = False
        self._timer: Optional[QTimer] = None
        self._ping_timer: Optional[QTimer] = None
        self._suspend_io: bool = False
        self.suspend_requested.connect(self.set_suspend_io)

    @Slot(bool)
    def set_suspend_io(self, suspended: bool) -> None:
        self._suspend_io = suspended
        if suspended:
            if self._ping_timer is not None:
                self._ping_timer.stop()
            if self._timer is not None:
                self._timer.stop()

    @Slot()
    def start_timer(self) -> None:
        self._timer = QTimer(self)
        self._timer.setInterval(1000)
        self._timer.timeout.connect(self._on_periodic_tick)
        self._timer.start()
        QTimer.singleShot(0, self._first_tick)

        self._ping_timer = QTimer(self)
        self._ping_timer.setInterval(_LINK_PING_INTERVAL_MS)
        self._ping_timer.timeout.connect(self._do_ping)
        self._ping_timer.start()
        QTimer.singleShot(0, self._do_ping)
        QTimer.singleShot(0, self.run_prime_initial_reads)

    def _gather_device_reads_data(
            self) -> Tuple[float, Optional[Tuple[float, float, float,
                                                 float, float, float]],
                           Optional[int]]:
        fs_hz = 0.0
        try:
            fs_hz = self._client.read_loop_fs_hz()
        except TunerError:
            pass
        shadows: Optional[Tuple[float, float, float,
                                  float, float, float]] = None
        try:
            if self._client.is_calibration_present():
                kp = self._client.read_kp()
                ki = self._client.read_ki()
                fc = self._client.read_current_filter_fc()
                shadows = (0.0, 0.0, 0.0, kp, ki, fc)
        except TunerError:
            pass
        pole: Optional[int] = None
        try:
            pp = int(self._client.read_motor_pole_pairs())
            if 1 <= pp <= 64:
                pole = pp
        except TunerError:
            pass
        return fs_hz, shadows, pole

    @Slot()
    def run_prime_initial_reads(self) -> None:
        self.device_reads_ready.emit(*self._gather_device_reads_data())

    @Slot()
    def run_post_reconnect_reads(self) -> None:
        self.device_reads_ready.emit(*self._gather_device_reads_data())

    @Slot()
    def _first_tick(self) -> None:
        self.poll_tick(True)

    @Slot()
    def _on_periodic_tick(self) -> None:
        self.poll_tick(False)

    @Slot(bool)
    def poll_tick(self, want_full_hint: bool) -> None:
        if self._suspend_io:
            return
        if want_full_hint:
            self._want_full = True
        self._run_round()

    def _run_round(self) -> None:
        if self._suspend_io:
            return
        r = self._client.reader
        if r is None or not r.is_running:
            self.poll_finished.emit(
                False, "link not running", None)
            return

        self._poll_serial += 1
        want_full = self._want_full
        if not want_full and self._poll_serial % 4 == 0:
            want_full = True

        try:
            kp = self._client.read_kp()
            ki = self._client.read_ki()
            lim = self._client.read_int_lim()
            fc = self._client.read_current_filter_fc()
            st = self._client.read_axis_state()
            override_active = bool(st & AxisStateFlag.RUNNING)
            self._prev_override_active = override_active

            if want_full:
                self._last_vmax = self._client.read_v_max()
                self._cal_present = self._client.is_calibration_present()

            snap = TunerPollSnapshot(
                kp=kp,
                ki=ki,
                lim=lim,
                fc=fc,
                vmax=self._last_vmax,
                state=int(st),
                override_active=override_active,
                cal_present=self._cal_present,
                last_poll_ok=True,
            )
            self._want_full = False
            self.poll_finished.emit(True, "", snap)

            if self._timer is not None:
                if (self._client.reader is not None
                        and self._client.reader.is_running):
                    if (st & AxisStateFlag.ALIGNED):
                        self._timer.setInterval(500)
                    else:
                        self._timer.setInterval(1000)
        except TunerError as e:
            self.poll_finished.emit(False, str(e), None)
        except Exception as e:
            self.poll_finished.emit(False, str(e), None)

    @Slot()
    def _do_ping(self) -> None:
        if self._suspend_io:
            self.ping_finished.emit(False, "io suspended")
            return
        r = self._client.reader
        if r is None or not r.is_running:
            self.ping_finished.emit(False, "link not running")
            return
        try:
            self._client.ping()
        except TunerError as e:
            self.ping_finished.emit(False, str(e))
        except Exception as e:
            self.ping_finished.emit(False, str(e))
        else:
            self.ping_finished.emit(True, "")

    @Slot()
    def finish_reconnect(self) -> None:
        self._suspend_io = False
        if self._ping_timer is not None:
            self._ping_timer.start()
        if self._timer is not None:
            self._timer.start()
        QTimer.singleShot(0, self._do_ping)
        QTimer.singleShot(0, self._first_tick)

    @Slot()
    def ping_now(self) -> None:
        self._do_ping()

    @Slot(bool)
    def set_paused(self, paused: bool) -> None:
        if self._timer is None:
            return
        if paused:
            self._timer.stop()
        else:
            self._timer.start()

    @Slot()
    def shutdown(self) -> None:
        if self._ping_timer is not None:
            self._ping_timer.stop()
            self._ping_timer.deleteLater()
            self._ping_timer = None
        if self._timer is not None:
            self._timer.stop()
            self._timer.deleteLater()
            self._timer = None
