"""Control view sidebar: id/iq targets, align, E-stop."""

from __future__ import annotations

from typing import Callable, Optional

from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QDoubleSpinBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from ..protocol import TunerClient, TunerError
from . import labels as L
from .alignment_progress import AlignmentProgressDialog
from .buttons import action_button
from .widgets import SurfaceCard


def _spin(minimum: float, maximum: float, decimals: int, value: float,
          step: float = 0.1, suffix: str = "") -> QDoubleSpinBox:
    box = QDoubleSpinBox()
    box.setRange(minimum, maximum)
    box.setDecimals(decimals)
    box.setSingleStep(step)
    box.setValue(value)
    if suffix:
        box.setSuffix(suffix)
    box.setMinimumWidth(140)
    return box


class _AlignThread(QThread):
    success = Signal()
    failed = Signal(str)

    def __init__(self, client: TunerClient, parent=None) -> None:
        super().__init__(parent)
        self._client = client

    def run(self) -> None:  # noqa: N802
        try:
            self._client.align_axis()
        except TunerError as e:
            self.failed.emit(str(e))
        else:
            self.success.emit()


class ControlRail(QWidget):
    long_operation = Signal(bool)

    def __init__(
        self,
        client: Optional[TunerClient],
        connected: Callable[[], bool],
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._client = client
        self._connected = connected
        self._align_thread: Optional[_AlignThread] = None
        self._align_progress: Optional[AlignmentProgressDialog] = None
        self._autoset_cb: Optional[Callable[[], None]] = None

        self.setSizePolicy(
            QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Expanding)

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(12)

        motion_card = SurfaceCard("Motion")
        mform = QFormLayout()
        mform.setLabelAlignment(Qt.AlignRight)
        self._override_box = QCheckBox("Manual setpoints")
        self._override_box.toggled.connect(self._on_override_toggled)
        mform.addRow(self._override_box)

        self._id_spin = _spin(-50.0, 50.0, 4, 0.0, step=0.1, suffix=" A")
        self._iq_spin = _spin(-50.0, 50.0, 4, 0.0, step=0.1, suffix=" A")
        self._id_spin.setEnabled(False)
        self._iq_spin.setEnabled(False)
        mform.addRow(L.D_AXIS_CURRENT, self._id_spin)
        mform.addRow(L.Q_AXIS_CURRENT, self._iq_spin)

        id_row = QHBoxLayout()
        id_row.setSpacing(6)
        for label, delta in (("−", -0.1), ("0", None), ("+", 0.1)):
            b = action_button(label, "BtnNudge")
            b.clicked.connect(
                lambda _c=False, d=delta, s=self._id_spin: self._nudge(s, d))
            id_row.addWidget(b)
        id_row.addStretch(1)
        mform.addRow("d-axis", id_row)

        iq_row = QHBoxLayout()
        iq_row.setSpacing(6)
        for label, delta in (("−", -0.1), ("0", None), ("+", 0.1)):
            b = action_button(label, "BtnNudge")
            b.clicked.connect(
                lambda _c=False, d=delta, s=self._iq_spin: self._nudge(s, d))
            iq_row.addWidget(b)
        iq_row.addStretch(1)
        mform.addRow("q-axis", iq_row)

        self._id_spin.valueChanged.connect(self._on_id_changed)
        self._iq_spin.valueChanged.connect(self._on_iq_changed)
        motion_card.body_layout.addLayout(mform)
        root.addWidget(motion_card)

        safety_card = SurfaceCard("Actions")
        safety_lay = QVBoxLayout()
        safety_lay.setSpacing(10)
        self._align_btn = action_button("Run alignment", "BtnDefault")
        self._align_btn.clicked.connect(self._on_align)
        safety_lay.addWidget(self._align_btn)
        self._estop_btn = action_button("E-STOP", "BtnEstop")
        self._estop_btn.clicked.connect(self._on_estop)
        safety_lay.addWidget(self._estop_btn)
        self._autoset_btn = action_button("Autoset", "BtnDefault")
        self._autoset_btn.clicked.connect(self._on_autoset)
        safety_lay.addWidget(self._autoset_btn)
        safety_card.body_layout.addLayout(safety_lay)
        root.addWidget(safety_card)

        self._status = QLabel("")
        self._status.setStyleSheet("color: #ef5350; font-size: 11px;")
        self._status.setWordWrap(True)
        root.addWidget(self._status)
        root.addStretch(1)

        self.set_actions_enabled(False)

    def set_client(self, client: Optional[TunerClient]) -> None:
        self._client = client

    def bind_svm_autoset(self, callback: Callable[[], None]) -> None:
        self._autoset_cb = callback

    def set_actions_enabled(self, on: bool) -> None:
        for w in (
            self._override_box,
            self._id_spin,
            self._iq_spin,
            self._align_btn,
            self._estop_btn,
            self._autoset_btn,
        ):
            w.setEnabled(on)
        for btn in self.findChildren(QPushButton):
            if btn is not self._estop_btn:
                btn.setEnabled(on)

    def _on_autoset(self) -> None:
        if self._autoset_cb is not None:
            self._autoset_cb()

    def apply_override_state(self, active: bool) -> None:
        self._override_box.blockSignals(True)
        self._override_box.setChecked(active)
        self._override_box.blockSignals(False)
        self._id_spin.setEnabled(active and self._connected())
        self._iq_spin.setEnabled(active and self._connected())

    @staticmethod
    def _nudge(spin: QDoubleSpinBox, delta: Optional[float]) -> None:
        if delta is None:
            spin.setValue(0.0)
        else:
            spin.setValue(spin.value() + delta)

    def _require_client(self) -> Optional[TunerClient]:
        if not self._connected() or self._client is None:
            self._status.setText("Connect a board to run this action.")
            return None
        self._status.setText("")
        return self._client

    def _on_override_toggled(self, on: bool) -> None:
        cli = self._require_client()
        if cli is None:
            self._override_box.blockSignals(True)
            self._override_box.setChecked(False)
            self._override_box.blockSignals(False)
            return
        try:
            if on:
                cli.run_axis()
            else:
                cli.stop_axis()
        except TunerError as e:
            self._status.setText(str(e))
            self._override_box.blockSignals(True)
            self._override_box.setChecked(not on)
            self._override_box.blockSignals(False)
            return
        self._id_spin.setEnabled(on)
        self._iq_spin.setEnabled(on)

    def _on_id_changed(self, v: float) -> None:
        cli = self._require_client()
        if cli is None:
            return
        try:
            cli.write_target_id(v)
        except TunerError as e:
            self._status.setText(str(e))

    def _on_iq_changed(self, v: float) -> None:
        cli = self._require_client()
        if cli is None:
            return
        try:
            cli.write_target_iq(v)
        except TunerError as e:
            self._status.setText(str(e))

    def _on_align(self) -> None:
        cli = self._require_client()
        if cli is None:
            return
        self.long_operation.emit(True)
        self._align_progress = AlignmentProgressDialog(self)
        self._align_progress.show()
        self._align_thread = _AlignThread(cli, self)
        self._align_thread.success.connect(self._on_align_done)
        self._align_thread.failed.connect(self._on_align_failed)
        self._align_thread.finished.connect(self._align_thread.deleteLater)
        self._align_thread.start()

    def _on_align_done(self) -> None:
        if self._align_progress is not None:
            self._align_progress.accept()
            self._align_progress = None
        self.long_operation.emit(False)

    def _on_align_failed(self, msg: str) -> None:
        if self._align_progress is not None:
            self._align_progress.reject()
            self._align_progress = None
        self._status.setText(msg)
        self.long_operation.emit(False)

    def _on_estop(self) -> None:
        cli = self._require_client()
        if cli is None:
            return
        try:
            cli.write_target_id(0.0)
            cli.write_target_iq(0.0)
            cli.stop_axis()
            self._override_box.blockSignals(True)
            self._override_box.setChecked(False)
            self._override_box.blockSignals(False)
            self._id_spin.setValue(0.0)
            self._iq_spin.setValue(0.0)
            self._id_spin.setEnabled(False)
            self._iq_spin.setEnabled(False)
        except TunerError as e:
            self._status.setText(str(e))
