"""Current-design sidebar: R, L, bandwidth for MPZ plots."""

from __future__ import annotations

from typing import Callable, Optional

from PySide6.QtWidgets import (
    QFormLayout,
    QGroupBox,
    QVBoxLayout,
    QWidget,
)

from .tuning_panel import _spin


class MotorModelRail(QWidget):
    def __init__(
        self,
        on_params_changed: Optional[Callable[[float, float, float,
                                            float, float], None]] = None,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._on_params_changed = on_params_changed
        self._last_r = 1.08
        self._last_l = 0.0018
        self._last_bw = 150.0

        root = QVBoxLayout(self)
        box = QGroupBox("Motor model")
        form = QFormLayout(box)
        self._r_spin = _spin(0.001, 1000.0, 4, self._last_r,
                             step=0.01, suffix=" Ω")
        self._l_mh_spin = _spin(0.001, 10_000.0, 4,
                                self._last_l * 1000.0,
                                step=0.01, suffix=" mH")
        self._bw_spin = _spin(1.0, 5000.0, 1, self._last_bw,
                              step=10.0, suffix=" Hz")
        form.addRow("R", self._r_spin)
        form.addRow("L", self._l_mh_spin)
        form.addRow("Bandwidth", self._bw_spin)
        root.addWidget(box)
        root.addStretch(1)

        for sp in (self._r_spin, self._l_mh_spin, self._bw_spin):
            sp.valueChanged.connect(self._notify)

        if self._on_params_changed is not None:
            self._notify()

    def set_kp_ki_hint(self, kp: float, ki: float) -> None:
        self._kp_hint = kp
        self._ki_hint = ki
        self._notify()

    def _notify(self) -> None:
        if self._on_params_changed is None:
            return
        l_h = self._l_mh_spin.value() * 1e-3
        kp = getattr(self, "_kp_hint", 1.0)
        ki = getattr(self, "_ki_hint", 100.0)
        self._on_params_changed(
            self._r_spin.value(), l_h, self._bw_spin.value(), kp, ki)

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
