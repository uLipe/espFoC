"""Current design view: motor model rail + analysis plots."""

from __future__ import annotations

from typing import Callable, Optional

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QHBoxLayout, QSplitter, QWidget

from ..analysis_panel import AnalysisPanel
from ..motor_model_rail import MotorModelRail


class CurrentView(QWidget):
    def __init__(
        self,
        analysis: AnalysisPanel,
        on_params_changed: Optional[Callable] = None,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        split = QSplitter(Qt.Horizontal)
        self.motor = MotorModelRail(on_params_changed=on_params_changed)
        split.addWidget(self.motor)
        split.addWidget(analysis)
        split.setStretchFactor(0, 0)
        split.setStretchFactor(1, 1)
        split.setSizes([220, 900])
        lay.addWidget(split)
