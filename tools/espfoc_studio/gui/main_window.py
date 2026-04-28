"""Main window: assembles Tuning + Analysis + Scope into a single view
and owns the single polling timer that keeps the live panels fresh."""

from __future__ import annotations

import time
from typing import Optional, Tuple

from PySide6.QtCore import QTimer, Qt
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
from ..protocol import AxisStateFlag, TunerClient, TunerError
from ..protocol.tuner import TUNER_FIRMWARE_TYPE_TSGX
from .theme import make_badge_qss, make_reset_board_button_qss
from .analysis_panel import AnalysisPanel
from .generate_app_panel import GenerateAppPanel
from .sensors_debug_panel import SensorsDebugPanel
from .scope_panel import ScopePanel
from .svm_panel import SvmPanel
from .tuning_panel import TuningPanel


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
        self._link_fail_streak = 0
        self._last_reconnect_mono: float = 0.0
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
        self._tuning.long_operation.connect(
            self._on_tuning_long_operation)
        splitter.addWidget(self._tuning)

        tabs = QTabWidget()
        tabs.addTab(self._analysis, "Analysis")
        tabs.addTab(self._sensors, "Sensors")
        tabs.addTab(self._scope, "Scope")
        tabs.addTab(self._svm, "SVM Hexagon")

        # Generate App is shown only when the firmware identifies itself
        # as TunerStudio target (or in --demo mode where the demo also
        # advertises TSGX). Hardware configuration lives inside this
        # panel now — it was a sibling tab before, but conceptually it
        # is one step of the same code-generation workflow.
        if self._is_tuner_studio_target(self._client):
            self._generate = GenerateAppPanel(
                self._client, self._current_motor_params)
            tabs.addTab(self._generate, "Generate App")

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

        # Live gains: the poll does several serial round-trips on the UI
        # thread. While the axis is not yet aligned, refresh at 1 s to keep
        # the window responsive; after align, 500 ms matches the old feel.
        self._timer = QTimer(self)
        self._timer.setInterval(1000)
        self._timer.timeout.connect(self._poll)
        self._timer.start()

        # Pull the actual loop sample rate from the firmware so the
        # Analysis tab discretises against the right Ts. Plan #2 (FOC
        # in PWM ISR) bumps this from ~2 kHz to 40 kHz; without it
        # the predicted step response on the Analysis tab would be
        # off by the same 20x factor.
        try:
            fs_hz = self._client.read_loop_fs_hz()
            if fs_hz > 1.0:
                self._analysis.set_loop_rate_hz(fs_hz)
                self._tuning.set_loop_rate_hz(fs_hz)
        except TunerError:
            pass

        try:
            if link_mode != "demo" and self._client.is_calibration_present():
                self._tuning.sync_motor_from_nvs_shadows()
        except TunerError:
            pass
        self._try_sync_motor_pole_pairs()
        # Prime the analysis view (including after optional NVS sync).
        self._tuning._notify_params_changed()

        if link_mode == "hw":
            self._update_link_badge()
        self._set_sensors_interactive()
        QTimer.singleShot(0, self._poll)

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
        if self._link_mode == "demo":
            self._sensors.set_interactive(False)
            return
        r = self._client.reader
        ok = bool(r and r.is_running and self._tuning.last_poll_ok)
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
        if (self._tuning.last_poll_ok
                and r is not None
                and r.is_running):
            self._link_fail_streak = 0
            self._set_link_badge("LINK_OK")
            return
        if r is not None and not r.is_running:
            self._set_link_badge("LINK_DOWN")
            return
        self._link_fail_streak += 1
        if self._link_fail_streak <= 1:
            self._set_link_badge("LINK_WAIT")
        else:
            self._set_link_badge("LINK_DOWN")

    def _reconnect_serial(self) -> bool:
        assert self._serial_config is not None
        from ..link.transport_serial import SerialTransport
        self._tuning.last_poll_ok = False
        self._link_fail_streak = 0
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
            return False
        r = LinkReader(t)
        r.start()
        self._client.replace_reader(r)
        self._scope.attach_reader(r)
        self._svm.attach_reader(r)
        self._sensors.attach_reader(r)
        self._tuning.rebind_log_reader()
        self._link_fail_streak = 0
        try:
            fs_hz = self._client.read_loop_fs_hz()
            if fs_hz > 1.0:
                self._analysis.set_loop_rate_hz(fs_hz)
                self._tuning.set_loop_rate_hz(fs_hz)
        except TunerError:
            pass
        try:
            if self._client.is_calibration_present():
                self._tuning.sync_motor_from_nvs_shadows()
        except TunerError:
            pass
        self._try_sync_motor_pole_pairs()
        self._tuning._notify_params_changed()
        return True

    def _try_sync_motor_pole_pairs(self) -> None:
        try:
            p = int(self._client.read_motor_pole_pairs())
            if 1 <= p <= 64:
                self._analysis.set_motor_pole_pairs_silent(p)
        except TunerError:
            pass

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

    def _on_tuning_long_operation(self, busy: bool) -> None:
        """Stop live tuner polls while a worker thread uses TunerClient;
        the GUI event loop and scope timer keep running."""
        if busy:
            self._timer.stop()
        else:
            self._timer.start()

    def _poll(self) -> None:
        if (self._serial_config is not None
                and not self._client.reader.is_running):
            self._tuning.last_poll_ok = False
        self._tuning.poll()
        st = self._tuning.last_axis_state
        if (self._link_mode != "demo" and st is not None
                and (st & AxisStateFlag.ALIGNED)):
            self._timer.setInterval(500)
        else:
            self._timer.setInterval(1000)
        self._update_link_badge()
        self._set_sensors_interactive()
        if (self._serial_config is not None
                and not self._tuning.last_poll_ok
                and self._maybe_reconnect()):
            self._tuning.poll()
            self._update_link_badge()

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

    @staticmethod
    def _is_tuner_studio_target(client: TunerClient) -> bool:
        try:
            return client.read_firmware_type() == TUNER_FIRMWARE_TYPE_TSGX
        except TunerError:
            return False

    def _current_motor_params(self) -> tuple[float, float, float]:
        return (self._tuning._r_spin.value(),
                self._tuning._l_mh_spin.value() * 1e-3,
                self._tuning._bw_spin.value())
