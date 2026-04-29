"""Main window: assembles Tuning + Analysis + Scope into a single view.

Tuner serial round-trips run on :class:`TunerPollWorker`; the GUI thread only
applies snapshots and updates badges."""

from __future__ import annotations

import time
from typing import Optional, Tuple

from PySide6.QtCore import QMetaObject, QTimer, Qt, QThread
from PySide6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QMainWindow,
    QPushButton,
    QSplitter,
    QTabWidget,
    QSizePolicy,
    QWidget,
)

from ..link import LinkReader
from ..protocol import TunerClient, TunerError
from .theme import make_badge_qss, make_reset_board_button_qss
from .analysis_panel import AnalysisPanel
from .generate_app_panel import GenerateAppPanel
from .sensors_debug_panel import SensorsDebugPanel
from .scope_panel import ScopePanel
from .svm_panel import SvmPanel
from .tuning_panel import TuningPanel
from .tuner_poll_worker import TunerPollSnapshot, TunerPollWorker

# Consecutive failed pings before the badge switches from CONNECTED to NO LINK.
_LINK_DOWN_AFTER_CONSECUTIVE_PING_FAILS = 10


class MainWindow(QMainWindow):
    def __init__(self, client: TunerClient,
                 title: str = "espFoC TunerStudio",
                 link_mode: str = "hw",
                 link_descr: str = "",
                 serial_config: Optional[Tuple[str, int, int]] = None) -> None:
        super().__init__()
        self._client = client
        self._serial_config = serial_config
        self._link_mode = link_mode
        self._last_reconnect_mono: float = 0.0
        self._link_ping_seen: bool = False
        self._ping_fail_streak: int = 0
        self.setWindowTitle(title)
        # 900 px of vertical room is what fits the SVM hexagon (380)
        # plus its three-phase waveform (220) plus axis labels and tab
        # chrome without clipping. The Scope tab is more forgiving but
        # benefits from the same headroom.
        self.resize(1280, 900)
        self.setMinimumSize(1024, 720)

        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)

        splitter = QSplitter(Qt.Horizontal)
        root.addWidget(splitter, 1)

        self._analysis = AnalysisPanel(client=self._client)
        # The scope and the SVM hexagon both subscribe to the same shared
        # LinkReader. ch0/1/2 are (by convention) the three-phase SVPWM
        # voltages, which the SVM panel reads exclusively.
        self._scope = ScopePanel(reader=self._client.reader)
        self._svm = SvmPanel(reader=self._client.reader)
        self._sensors = SensorsDebugPanel(reader=self._client.reader)

        # Analysis plots are expensive (step sim + bode + root locus).
        # Debounce spinbox storms so one nudge of the mouse wheel doesn't
        # fire a dozen full recomputes back-to-back. Must be created
        # BEFORE the TuningPanel because the panel primes an initial
        # _on_params call during construction.
        self._analysis_pending = None
        self._analysis_debounce = QTimer(self)
        self._analysis_debounce.setSingleShot(True)
        self._analysis_debounce.setInterval(150)
        self._analysis_debounce.timeout.connect(self._run_pending_analysis)

        self._tuning = TuningPanel(
            self._client, on_params_changed=self._on_params)
        splitter.addWidget(self._tuning)

        tabs = QTabWidget()
        tabs.addTab(self._analysis, "Analysis")
        tabs.addTab(self._sensors, "Sensors")
        tabs.addTab(self._svm, "SVM Hexagon")
        tabs.addTab(self._scope, "Scope")

        # Generate App (embedded Hardware + codegen) is only for `--demo`.
        # Live targets use TunerStudio for tuning; app generation stays out
        # of the hot path / serial workflow.
        if self._link_mode == "demo":
            self._generate = GenerateAppPanel(
                self._client, self._current_motor_params)
            tabs.addTab(self._generate, "Generate App")
            hw_cfg = self._generate._hw.get_config()
            self._sensors.set_counts_per_rev(
                int(hw_cfg.sensor_cfg.get("counts_per_rev", 4096)))

        splitter.addWidget(tabs)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([380, 900])

        # Status bar: transport + link badge (permanent, right).
        if link_descr:
            dtext = link_descr
        elif serial_config is not None:
            dtext = f"{serial_config[0]} @ {serial_config[1]}"
        else:
            dtext = ""
        sb = self.statusBar()
        self._link_badge = QLabel()
        self._link_badge.setSizePolicy(
            QSizePolicy.Minimum, QSizePolicy.Fixed)
        self._link_descr = QLabel()
        self._link_descr.setStyleSheet("color: #9aa0a6; font-size: 12px;")
        self._link_descr.setText(dtext)
        self._link_descr.setMinimumWidth(100)
        self._reset_btn = QPushButton("RESET BOARD")
        self._reset_btn.setCursor(Qt.PointingHandCursor)
        self._reset_btn.setStyleSheet(make_reset_board_button_qss())
        self._reset_btn.setToolTip(
            "Restart the target (esp_restart). Emergency use only.")
        self._reset_btn.clicked.connect(self._on_reset_board_clicked)
        if link_mode == "demo":
            self._reset_btn.setEnabled(False)
        sb.addPermanentWidget(self._link_badge, 0)
        sb.addPermanentWidget(self._reset_btn, 0)
        sb.addPermanentWidget(self._link_descr, 0)
        if link_mode == "demo":
            self._set_link_badge("LINK_DEMO")
        else:
            self._set_link_badge("LINK_WAIT")

        self._poll_thread = QThread(self)
        self._poll_worker = TunerPollWorker(self._client, link_mode)
        self._poll_worker.moveToThread(self._poll_thread)
        self._poll_worker.poll_finished.connect(
            self._on_poll_finished, Qt.QueuedConnection)
        self._poll_worker.ping_finished.connect(
            self._on_ping_finished, Qt.QueuedConnection)
        self._poll_worker.device_reads_ready.connect(
            self._on_device_reads_ready, Qt.QueuedConnection)
        self._tuning.poll_refresh_requested.connect(
            self._poll_worker.poll_tick, Qt.QueuedConnection)
        self._tuning.long_operation.connect(
            self._poll_worker.set_paused, Qt.QueuedConnection)
        self._poll_thread.started.connect(self._poll_worker.start_timer)
        self._poll_thread.start()

        if link_mode == "hw":
            self._update_link_badge()
        self._set_sensors_interactive()

    def _on_device_reads_ready(
            self,
            fs_hz: float,
            shadows: object,
            pole: object,
    ) -> None:
        """Apply tuner readout gathered on :attr:`_poll_worker` (GUI thread only)."""
        if fs_hz > 1.0:
            self._analysis.set_loop_rate_hz(fs_hz)
            self._tuning.set_loop_rate_hz(fs_hz)
        if shadows is not None:
            t = shadows
            self._tuning.apply_nvs_shadow_floats(
                t[0], t[1], t[2], t[3], t[4], t[5])
        if pole is not None:
            self._analysis.set_motor_pole_pairs_silent(int(pole))
        self._tuning._notify_params_changed()

    def closeEvent(self, event) -> None:
        self._tuning.long_operation.emit(True)
        QMetaObject.invokeMethod(
            self._poll_worker,
            "shutdown",
            Qt.BlockingQueuedConnection,
        )
        self._poll_thread.quit()
        self._poll_thread.wait(5000)
        super().closeEvent(event)

    def _on_reset_board_clicked(self) -> None:
        r = QMessageBox.question(
            self, "Reset board",
            "Restart the board now? The USB link may drop briefly.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if r != QMessageBox.Yes:
            return
        try:
            self._client.reset_board()
        except TunerError as e:
            QMessageBox.warning(self, "Reset failed", str(e))

    def _set_sensors_interactive(self) -> None:
        # Same readiness rule as Scope / SVM: live when the link reader is
        # running. Do not gate on tuner poll success — scope frames can flow
        # while CMD_TUNER polling is still warming up or temporarily failing.
        r = self._client.reader
        ok = bool(r and r.is_running)
        self._sensors.set_interactive(ok)

    def _set_link_badge(self, key: str) -> None:
        text, qss = make_badge_qss(key)
        self._link_badge.setText(text)
        self._link_badge.setStyleSheet(qss)
        self._link_badge.setVisible(True)

    def _update_link_badge(self) -> None:
        if self._link_mode == "demo":
            self._set_link_badge("LINK_DEMO")
            return
        r = self._client.reader
        if r is None or not r.is_running:
            self._set_link_badge("LINK_DOWN")
            return
        if not self._link_ping_seen:
            self._set_link_badge("LINK_WAIT")
            return
        if self._ping_fail_streak >= _LINK_DOWN_AFTER_CONSECUTIVE_PING_FAILS:
            self._set_link_badge("LINK_DOWN")
        else:
            self._set_link_badge("LINK_OK")

    def _on_ping_finished(self, ok: bool, err: str) -> None:
        if self._link_mode != "hw":
            return
        self._link_ping_seen = True
        if ok:
            self._ping_fail_streak = 0
        else:
            self._ping_fail_streak += 1
        self._update_link_badge()
        if (not ok
                and self._serial_config is not None
                and self._poll_error_implies_dead_transport(err)
                and self._maybe_reconnect()):
            QMetaObject.invokeMethod(
                self._poll_worker,
                "ping_now",
                Qt.QueuedConnection,
            )

    def _reconnect_serial(self) -> bool:
        """Replace serial transport. Never acquire ``_bus_lock`` on the GUI thread —
        that blocked the window while the worker held the mutex during poll/ping."""
        assert self._serial_config is not None
        from ..link.transport_serial import SerialTransport
        self._tuning.last_poll_ok = False
        self._link_ping_seen = False
        self._ping_fail_streak = 0
        self._poll_worker.suspend_requested.emit(True)
        time.sleep(0.15)
        success = False
        try:
            self._tuning.detach_log_reader()
            old = self._client.reader
            try:
                old.stop()
            except Exception:
                pass
            port, baud, _axis = self._serial_config
            try:
                t = SerialTransport(port=port, baud=baud)
            except Exception:
                success = False
            else:
                r = LinkReader(t)
                r.start()
                time.sleep(0.15)
                self._client.replace_reader(r)
                self._scope.attach_reader(r)
                self._svm.attach_reader(r)
                self._sensors.attach_reader(r)
                self._tuning.rebind_log_reader()
                success = True
        finally:
            QMetaObject.invokeMethod(
                self._poll_worker,
                "run_post_reconnect_reads",
                Qt.QueuedConnection,
            )
            QMetaObject.invokeMethod(
                self._poll_worker,
                "finish_reconnect",
                Qt.QueuedConnection,
            )
        return success

    def _maybe_reconnect(self) -> bool:
        if self._serial_config is None:
            return False
        now = time.monotonic()
        if now - self._last_reconnect_mono < 1.0:
            return False
        self._last_reconnect_mono = now
        if self._reconnect_serial():
            return True
        return False

    @staticmethod
    def _poll_error_implies_dead_transport(err: str) -> bool:
        """Host-side I/O loss or dead reader — safe to reopen serial."""
        el = (err or "").lower()
        if (
                "link not running" in el
                or "reader stopped" in el
                or "link i/o:" in el):
            return True
        return any(
            s in el for s in (
                "errno 5",
                "[errno 5]",
                "input/output error",
                "bad file descriptor",
                "serial send failed",
                "timeout waiting for response",
                "device disconnected",
            ))

    def _on_poll_finished(
            self,
            ok: bool,
            err: str,
            snap: Optional[TunerPollSnapshot]) -> None:
        if ok and snap is not None:
            self._tuning.apply_poll_snapshot(snap)
        else:
            self._tuning.apply_poll_error(err or "poll failed")
        self._set_sensors_interactive()
        if (self._serial_config is not None
                and not self._tuning.last_poll_ok
                and self._poll_error_implies_dead_transport(err)
                and self._maybe_reconnect()):
            self._tuning.poll_refresh_requested.emit(True)

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

    def _current_motor_params(self) -> tuple[float, float, float]:
        return (self._tuning._r_spin.value(),
                self._tuning._l_mh_spin.value() * 1e-3,
                self._tuning._bw_spin.value())
