"""espFoC Tool main window: nav rail + four views, optional USB auto-connect."""

from __future__ import annotations

import time
from typing import Optional, Tuple

from PySide6.QtCore import QMetaObject, Qt, QThread, QTimer
from PySide6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QMainWindow,
    QPushButton,
    QSplitter,
    QStackedWidget,
    QSizePolicy,
    QWidget,
)

from ..link import LinkReader
from ..protocol import TunerClient, TunerError
from .analysis_panel import AnalysisPanel
from .connection_manager import ConnectionManager
from .control_rail import ControlRail
from .nav_rail import NavRail
from .scope_stream_timing import scope_uniform_dt_s
from .states_panel import StatesPanel
from .svm_panel import SvmPanel
from .theme import make_badge_qss, make_reset_board_button_qss
from .tuning_panel import TuningPanel
from .tuner_poll_worker import TunerPollSnapshot, TunerPollWorker

_LINK_DOWN_AFTER_CONSECUTIVE_PING_FAILS = 10


class MainWindow(QMainWindow):
    def __init__(
        self,
        conn: ConnectionManager,
        title: str = "espFoC Tool",
    ) -> None:
        super().__init__()
        self._conn = conn
        self._client: Optional[TunerClient] = None
        self._serial_config: Optional[Tuple[str, int, int]] = None
        self._link_ping_seen = False
        self._ping_fail_streak = 0
        self._last_reconnect_mono = 0.0

        self.setWindowTitle(title)
        self.resize(1320, 900)
        self.setMinimumSize(1080, 720)

        central = QWidget()
        self.setCentralWidget(central)
        outer = QHBoxLayout(central)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        self._nav = NavRail()
        outer.addWidget(self._nav)

        self._stack = QStackedWidget()
        self._nav.page_selected.connect(self._stack.setCurrentIndex)
        outer.addWidget(self._stack, 1)

        self._analysis = AnalysisPanel(client=None)
        self._analysis_debounce = QTimer(self)
        self._analysis_debounce.setSingleShot(True)
        self._analysis_debounce.setInterval(150)
        self._analysis_debounce.timeout.connect(self._run_pending_analysis)
        self._analysis_pending = None

        self._tuning = TuningPanel(
            client=None, on_params_changed=self._on_params)
        self._control = ControlRail(
            client=None, connected=self._device_connected)
        self._svm = SvmPanel(reader=None)
        self._states = StatesPanel(reader=None)

        self._stack.addWidget(self._wrap_margin(self._tuning))
        self._stack.addWidget(self._wrap_margin(self._analysis))
        ctrl = QWidget()
        cs = QHBoxLayout(ctrl)
        cs.setContentsMargins(0, 0, 0, 0)
        split = QSplitter(Qt.Horizontal)
        split.addWidget(self._control)
        split.addWidget(self._svm)
        split.setStretchFactor(0, 0)
        split.setStretchFactor(1, 1)
        split.setSizes([320, 900])
        cs.addWidget(split)
        self._stack.addWidget(ctrl)
        self._stack.addWidget(self._states)

        sb = self.statusBar()
        self._link_badge = QLabel()
        self._link_badge.setSizePolicy(
            QSizePolicy.Minimum, QSizePolicy.Fixed)
        self._link_descr = QLabel()
        self._link_descr.setStyleSheet("color: #9aa0a6; font-size: 12px;")
        self._reset_btn = QPushButton("RESET BOARD")
        self._reset_btn.setStyleSheet(make_reset_board_button_qss())
        self._reset_btn.clicked.connect(self._on_reset_board_clicked)
        sb.addPermanentWidget(self._link_badge)
        sb.addPermanentWidget(self._reset_btn)
        sb.addPermanentWidget(self._link_descr)

        self._poll_thread: Optional[QThread] = None
        self._poll_worker: Optional[TunerPollWorker] = None

        self._conn.state_changed.connect(self._on_conn_state)
        self._conn.port_descr_changed.connect(self._link_descr.setText)
        self._conn.client_ready.connect(self._on_client_ready)
        self._conn.client_lost.connect(self._on_client_lost)

        self._control.long_operation.connect(self._set_poll_paused)
        self._tuning.long_operation.connect(self._set_poll_paused)

        self._on_conn_state(self._conn.state)
        self._conn.start()

    @staticmethod
    def _wrap_margin(child: QWidget) -> QWidget:
        page = QWidget()
        lay = QHBoxLayout(page)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.addWidget(child)
        return page

    def _device_connected(self) -> bool:
        return self._client is not None and self._conn.connected

    def _set_device_actions_enabled(self, on: bool) -> None:
        self._reset_btn.setEnabled(on)
        self._tuning.set_actions_enabled(on)
        self._control.set_actions_enabled(on)
        self._analysis._apply_mpz_btn.setEnabled(on)
        self._analysis._save_nvs_btn.setEnabled(on)
        self._analysis._motor_pole_pairs.setEnabled(on)
        if on:
            self._states.set_interactive(True)
            if self._client is not None:
                self._svm.attach_reader(self._client.reader)
                self._states.attach_reader(self._client.reader)
        else:
            self._states.set_interactive(False)

    def _on_conn_state(self, state: str) -> None:
        key = {
            ConnectionManager.STATE_NO_DEVICE: "NO_DEVICE",
            ConnectionManager.STATE_SCANNING: "SCANNING",
            ConnectionManager.STATE_CONNECTING: "LINK_WAIT",
            ConnectionManager.STATE_CONNECTED: "LINK_WAIT",
        }.get(state, "NO_DEVICE")
        text, qss = make_badge_qss(key)
        self._link_badge.setText(text)
        self._link_badge.setStyleSheet(qss)

    def _on_client_ready(self, client: object) -> None:
        self._client = client  # type: ignore[assignment]
        assert isinstance(self._client, TunerClient)
        if self._conn._fixed_port:
            self._serial_config = (
                self._conn._fixed_port, self._conn._baud, self._conn._axis)
        self._tuning.set_client(self._client)
        self._control.set_client(self._client)
        self._analysis._client = self._client
        self._svm.attach_reader(self._client.reader)
        self._states.attach_reader(self._client.reader)
        self._tuning.rebind_log_reader()
        self._start_poll_worker()
        self._set_device_actions_enabled(True)
        try:
            self._client.scope_start()
        except TunerError:
            pass
        text, qss = make_badge_qss("LINK_WAIT")
        self._link_badge.setText(text)
        self._link_badge.setStyleSheet(qss)

    def _on_client_lost(self) -> None:
        self._stop_poll_worker()
        self._client = None
        self._tuning.set_client(None)
        self._control.set_client(None)
        self._analysis._client = None
        self._svm.detach()
        self._states.attach_reader(None)
        self._set_device_actions_enabled(False)
        self._link_ping_seen = False
        self._ping_fail_streak = 0

    def _start_poll_worker(self) -> None:
        if self._client is None or self._poll_thread is not None:
            return
        self._poll_thread = QThread(self)
        self._poll_worker = TunerPollWorker(self._client)
        self._poll_worker.moveToThread(self._poll_thread)
        self._poll_worker.poll_finished.connect(
            self._on_poll_finished, Qt.QueuedConnection)
        self._poll_worker.ping_finished.connect(
            self._on_ping_finished, Qt.QueuedConnection)
        self._poll_worker.device_reads_ready.connect(
            self._on_device_reads_ready, Qt.QueuedConnection)
        self._tuning.poll_refresh_requested.connect(
            self._poll_worker.poll_tick, Qt.QueuedConnection)
        self._poll_thread.started.connect(self._poll_worker.start_timer)
        self._poll_thread.start()
        self._tuning.poll_refresh_requested.emit(True)

    def _stop_poll_worker(self) -> None:
        if self._poll_worker is None or self._poll_thread is None:
            return
        self._tuning.long_operation.emit(True)
        QMetaObject.invokeMethod(
            self._poll_worker, "shutdown", Qt.BlockingQueuedConnection)
        self._poll_thread.quit()
        self._poll_thread.wait(3000)
        self._poll_thread = None
        self._poll_worker = None

    def _set_poll_paused(self, paused: bool) -> None:
        if self._poll_worker is not None:
            self._poll_worker.set_paused(paused)

    def closeEvent(self, event) -> None:
        self._conn.stop()
        self._stop_poll_worker()
        super().closeEvent(event)

    def _on_reset_board_clicked(self) -> None:
        if not self._device_connected() or self._client is None:
            return
        r = QMessageBox.question(
            self, "Reset board",
            "Restart the board now? The USB link may drop briefly.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if r != QMessageBox.Yes:
            return
        try:
            self._client.reset_board()
        except TunerError as e:
            QMessageBox.warning(self, "Reset failed", str(e))

    def _on_ping_finished(self, ok: bool, err: str) -> None:
        self._link_ping_seen = True
        if ok:
            self._ping_fail_streak = 0
        else:
            self._ping_fail_streak += 1
        if self._client is None:
            return
        if not self._link_ping_seen:
            return
        if self._ping_fail_streak >= _LINK_DOWN_AFTER_CONSECUTIVE_PING_FAILS:
            self._set_link_badge("LINK_DOWN")
        else:
            self._set_link_badge("LINK_OK")

    def _set_link_badge(self, key: str) -> None:
        text, qss = make_badge_qss(key)
        self._link_badge.setText(text)
        self._link_badge.setStyleSheet(qss)

    def _on_poll_finished(
            self,
            ok: bool,
            err: str,
            snap: Optional[TunerPollSnapshot]) -> None:
        if ok and snap is not None:
            self._tuning.apply_poll_snapshot(snap)
            self._control.apply_override_state(snap.override_active)
        else:
            self._tuning.apply_poll_error(err or "poll failed")
        if self._client is not None:
            self._states.set_interactive(self._client.reader.is_running)

    def _on_device_reads_ready(
            self, fs_hz: float, shadows: object, pole: object) -> None:
        if fs_hz > 1.0:
            self._analysis.set_loop_rate_hz(fs_hz)
            self._tuning.set_loop_rate_hz(fs_hz)
            dt = scope_uniform_dt_s(fs_hz)
            self._svm.set_uniform_sample_period_s(dt)
            self._states.set_uniform_sample_period_s(dt)
        if shadows is not None:
            t = shadows
            self._tuning.apply_nvs_shadow_floats(
                t[0], t[1], t[2], t[3], t[4], t[5])
        if pole is not None:
            self._analysis.set_motor_pole_pairs_silent(int(pole))
        self._tuning._notify_params_changed()

    def _on_params(self, r: float, l: float, bw: float,
                   kp: float, ki: float) -> None:
        self._analysis_pending = (r, l, bw, kp, ki)
        self._analysis_debounce.start()

    def _run_pending_analysis(self) -> None:
        if self._analysis_pending is None:
            return
        r, l, bw, kp, ki = self._analysis_pending
        self._analysis_pending = None
        self._analysis.update_model(r, l, bw, kp, ki)
