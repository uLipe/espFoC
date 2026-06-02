"""Tune view: device setup, flash, motor model, MPZ analysis."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QHBoxLayout,
    QScrollArea,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from ..analysis_panel import AnalysisPanel
from ..motor_model_rail import MotorModelRail
from ..nvs_diff_panel import NvsDiffPanel
from ..tuning_panel import TuningPanel
from ..widgets import PageShell, horizontal_splitter


class TuneView(QWidget):
    def __init__(
        self,
        tuning: TuningPanel,
        analysis: AnalysisPanel,
        on_params_changed=None,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)

        self.nvs = NvsDiffPanel(
            kp_spin=tuning._kp_spin,
            ki_spin=tuning._ki_spin,
            lim_spin=tuning._lim_spin,
            fc_spin=tuning._fc_spin,
        )
        self.nvs.setMinimumWidth(260)
        self.nvs.setMaximumWidth(420)
        self.nvs.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)

        self.motor = MotorModelRail(on_params_changed=on_params_changed)

        setup_scroll = QScrollArea()
        setup_scroll.setWidgetResizable(True)
        setup_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        setup_scroll.setFrameShape(QScrollArea.NoFrame)
        setup_scroll.setWidget(tuning)
        setup_scroll.setMinimumWidth(300)
        setup_scroll.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)

        plots_col = QWidget()
        plots_lay = QVBoxLayout(plots_col)
        plots_lay.setContentsMargins(0, 0, 0, 0)
        plots_lay.setSpacing(12)
        plots_lay.addWidget(self.motor, 0)
        plots_lay.addWidget(analysis, 1)

        split = horizontal_splitter(
            setup_scroll,
            self.nvs,
            plots_col,
            stretches=(2, 1, 3),
            sizes=(400, 300, 720),
        )

        body = QWidget()
        lay = QHBoxLayout(body)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(split)

        shell = PageShell(
            "Tune",
            "Status and gains on the left, flash diff in the center, "
            "motor model and MPZ plots on the right.",
            body,
            parent=self,
        )
        outer = QHBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(shell)

        for sp in (tuning._kp_spin, tuning._ki_spin,
                   tuning._lim_spin, tuning._fc_spin):
            sp.valueChanged.connect(self.nvs.refresh_from_editor)
