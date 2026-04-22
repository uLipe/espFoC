"""Main window: assembles Tuning + Analysis + Scope into a single view
and owns the single polling timer that keeps the live panels fresh."""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import QTimer, Qt
from PySide6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QSizePolicy,
    QSplitter,
    QTabWidget,
    QWidget,
)

from ..protocol import TunerClient, TunerError
from ..protocol.tuner import TUNER_FIRMWARE_TYPE_TSGX
from .analysis_panel import AnalysisPanel
from .generate_app_panel import GenerateAppPanel
from .scope_panel import ScopePanel
from .svm_panel import SvmPanel
from .theme import make_badge_qss
from .tuning_panel import TuningPanel


# Number of consecutive failed polls before flipping the link badge to
# NO LINK. Two polls = ~1 s with the default 500 ms timer; that absorbs
# isolated round-trip jitter on a real serial link without strobing.
_LINK_DOWN_THRESHOLD = 2


class MainWindow(QMainWindow):
    def __init__(self, client: TunerClient,
                 title: str = "espFoC TunerStudio",
                 link_mode: str = "hw",
                 link_descr: str = "") -> None:
        """link_mode is "hw" for any real transport (serial, USB-CDC, ...)
        and "demo" for the in-process Python DemoFirmware. link_descr is
        an optional human-readable detail string shown next to the
        badge in the status bar (e.g. "/dev/ttyACM0 @ 921600")."""
        super().__init__()
        self.setWindowTitle(title)
        self._link_mode = link_mode
        self._link_descr = link_descr
        self._link_fail_streak = 0
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

        self._analysis = AnalysisPanel()
        # The scope and the SVM hexagon both subscribe to the same shared
        # LinkReader. ch0/1/2 are (by convention) the three-phase SVPWM
        # voltages, which the SVM panel reads exclusively.
        self._scope = ScopePanel(reader=client.reader)
        self._svm = SvmPanel(reader=client.reader)

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

        self._tuning = TuningPanel(client, on_params_changed=self._on_params)
        splitter.addWidget(self._tuning)

        tabs = QTabWidget()
        tabs.addTab(self._analysis, "Analysis")
        tabs.addTab(self._scope, "Scope")
        tabs.addTab(self._svm, "SVM Hexagon")

        # Generate App is shown only when the firmware identifies itself
        # as TunerStudio target (or in --demo mode where the demo also
        # advertises TSGX). Hardware configuration lives inside this
        # panel now — it was a sibling tab before, but conceptually it
        # is one step of the same code-generation workflow.
        if self._is_tuner_studio_target(client):
            self._generate = GenerateAppPanel(
                client, self._current_motor_params)
            tabs.addTab(self._generate, "Generate App")

        splitter.addWidget(tabs)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([380, 900])

        # Status bar: link-state badge on the left, optional human-
        # readable descriptor (port name / demo backend) on the right.
        self._link_badge = QLabel()
        self._link_badge.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
        self._link_descr_label = QLabel(self._link_descr)
        sb = self.statusBar()
        sb.addPermanentWidget(QLabel("Link:"))
        sb.addPermanentWidget(self._link_badge)
        sb.addPermanentWidget(self._link_descr_label)
        self._set_link_badge("LINK_DEMO" if link_mode == "demo" else "LINK_WAIT")

        # Live gains readout — every 500 ms is plenty for human perception
        # and leaves headroom for the 50 FPS scope. Each poll issues five
        # short tuner round-trips; 0.5 s keeps CPU usage near the noise
        # floor on real serial links.
        self._timer = QTimer(self)
        self._timer.setInterval(500)
        self._timer.timeout.connect(self._poll)
        self._timer.start()

        # Pull the actual loop sample rate from the firmware so the
        # Analysis tab discretises against the right Ts. Plan #2 (FOC
        # in PWM ISR) bumps this from ~2 kHz to 40 kHz; without it
        # the predicted step response on the Analysis tab would be
        # off by the same 20x factor.
        try:
            fs_hz = client.read_loop_fs_hz()
            if fs_hz > 1.0:
                self._analysis.set_loop_rate_hz(fs_hz)
                self._tuning.set_loop_rate_hz(fs_hz)
        except TunerError:
            pass

        # Prime the analysis view with the initial spinbox values.
        self._tuning._notify_params_changed()

    def _poll(self) -> None:
        self._tuning.poll()
        self._update_link_badge(self._tuning.last_poll_ok)

    def _set_link_badge(self, key: str) -> None:
        label, qss = make_badge_qss(key)
        self._link_badge.setText(label)
        self._link_badge.setStyleSheet(qss)

    def _update_link_badge(self, last_ok: bool) -> None:
        # The DEMO badge stays sticky — the in-process firmware never
        # "drops" the link in any meaningful way, so flicking the badge
        # to NO LINK on a transient debounced poll would be confusing.
        if self._link_mode == "demo":
            return
        if last_ok:
            self._link_fail_streak = 0
            self._set_link_badge("LINK_OK")
            return
        self._link_fail_streak += 1
        if self._link_fail_streak >= _LINK_DOWN_THRESHOLD:
            self._set_link_badge("LINK_DOWN")

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
