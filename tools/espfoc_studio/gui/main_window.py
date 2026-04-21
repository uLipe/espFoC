"""Main window: assembles Tuning + Analysis + Scope into a single view
and owns the single polling timer that keeps the live panels fresh."""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import QTimer, Qt
from PySide6.QtWidgets import (
    QHBoxLayout,
    QMainWindow,
    QSplitter,
    QTabWidget,
    QWidget,
)

from ..protocol import TunerClient
from .analysis_panel import AnalysisPanel
from .hardware_panel import HardwarePanel
from .scope_panel import ScopePanel
from .svm_panel import SvmPanel
from .tuning_panel import TuningPanel


class MainWindow(QMainWindow):
    def __init__(self, client: TunerClient,
                 title: str = "espFoC TunerStudio") -> None:
        super().__init__()
        self.setWindowTitle(title)
        self.resize(1280, 800)

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

        self._hardware = HardwarePanel()

        tabs = QTabWidget()
        tabs.addTab(self._analysis, "Analysis")
        tabs.addTab(self._scope, "Scope")
        tabs.addTab(self._svm, "SVM Hexagon")
        tabs.addTab(self._hardware, "Hardware")
        splitter.addWidget(tabs)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([380, 900])

        # Live gains readout — every 500 ms is plenty for human perception
        # and leaves headroom for the 50 FPS scope. Each poll issues five
        # short tuner round-trips; 0.5 s keeps CPU usage near the noise
        # floor on real serial links.
        self._timer = QTimer(self)
        self._timer.setInterval(500)
        self._timer.timeout.connect(self._poll)
        self._timer.start()

        # Prime the analysis view with the initial spinbox values.
        self._tuning._notify_params_changed()

    def _poll(self) -> None:
        self._tuning.poll()

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
