"""Motor parameters bar for MPZ preview (R, L, bandwidth)."""

from __future__ import annotations

from typing import Callable, Optional

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QFormLayout, QHBoxLayout, QSizePolicy, QWidget

from . import labels as L
from .widgets import LiveMetricGrid, SurfaceCard, spin_box


class MotorModelRail(QWidget):
    def __init__(
        self,
        on_params_changed: Optional[Callable[[float, float, float,
                                            float, float], None]] = None,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._on_params_changed = on_params_changed
        self._kp_hint = 1.0
        self._ki_hint = 100.0
        self.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

        card = SurfaceCard("Motor model")
        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(24)

        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignRight)
        form.setHorizontalSpacing(12)
        self._r_spin = spin_box(0.001, 1000.0, 4, 1.08, step=0.01, suffix=" Ω")
        self._l_mh_spin = spin_box(0.001, 10_000.0, 4, 1.8, step=0.01, suffix=" mH")
        self._bw_spin = spin_box(1.0, 5000.0, 1, 150.0, step=10.0, suffix=" Hz")
        form.addRow(L.RESISTANCE, self._r_spin)
        form.addRow(L.INDUCTANCE, self._l_mh_spin)
        form.addRow(L.CONTROL_BANDWIDTH, self._bw_spin)
        row.addLayout(form)

        self._gain_grid = LiveMetricGrid([L.PROPORTIONAL_GAIN, L.INTEGRAL_GAIN])
        row.addWidget(self._gain_grid, 1)

        card.body_layout.addLayout(row)
        outer = QHBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(card)

        for sp in (self._r_spin, self._l_mh_spin, self._bw_spin):
            sp.valueChanged.connect(self._notify)
        if self._on_params_changed is not None:
            self._notify()

    def set_kp_ki_hint(self, kp: float, ki: float) -> None:
        self._kp_hint = kp
        self._ki_hint = ki
        self._gain_grid.set_values([f"{kp:.4f} V/A", f"{ki:.1f} V/(A·s)"])
        self._notify()

    def _notify(self) -> None:
        if self._on_params_changed is None:
            return
        l_h = self._l_mh_spin.value() * 1e-3
        self._on_params_changed(
            self._r_spin.value(), l_h, self._bw_spin.value(),
            self._kp_hint, self._ki_hint)

    def apply_nvs_motor(self, r: float, l_h: float, bw: float) -> None:
        for sp in (self._r_spin, self._l_mh_spin, self._bw_spin):
            sp.blockSignals(True)
        try:
            if r > 1e-10:
                self._r_spin.setValue(r)
            if l_h > 1e-12:
                self._l_mh_spin.setValue(l_h * 1e3)
            if bw > 0.5:
                self._bw_spin.setValue(bw)
        finally:
            for sp in (self._r_spin, self._l_mh_spin, self._bw_spin):
                sp.blockSignals(False)
        self._notify()
