"""Config view: tuning controls + NVS diff panel."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QHBoxLayout, QSplitter, QWidget

from ..nvs_diff_panel import NvsDiffPanel
from ..tuning_panel import TuningPanel


class ConfigView(QWidget):
    def __init__(self, tuning: TuningPanel, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        split = QSplitter(Qt.Horizontal)
        split.addWidget(tuning)
        self.nvs = NvsDiffPanel(
            kp_spin=tuning._kp_spin,
            ki_spin=tuning._ki_spin,
            lim_spin=tuning._lim_spin,
            fc_spin=tuning._fc_spin,
        )
        split.addWidget(self.nvs)
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 2)
        split.setSizes([520, 340])
        lay.addWidget(split)
        for sp in (tuning._kp_spin, tuning._ki_spin,
                   tuning._lim_spin, tuning._fc_spin):
            sp.valueChanged.connect(self.nvs.refresh_from_editor)
