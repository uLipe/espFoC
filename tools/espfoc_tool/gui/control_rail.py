"""Control view sidebar: id/iq targets, align, E-stop."""

from __future__ import annotations

from typing import Callable, Optional

from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from ..protocol import TunerClient, TunerError
from .alignment_progress import AlignmentProgressDialog
from .theme import make_estop_button_qss


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

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)

        motion = QGroupBox("Motion")
        mform = QFormLayout(motion)
        self._override_box = QCheckBox("Override active")
        self._override_box.toggled.connect(self._on_override_toggled)
        mform.addRow(self._override_box)

        self._id_spin = _spin(-50.0, 50.0, 4, 0.0, step=0.1, suffix=" A")
        self._iq_spin = _spin(-50.0, 50.0, 4, 0.0, step=0.1, suffix=" A")
        self._id_spin.setEnabled(False)
        self._iq_spin.setEnabled(False)
        mform.addRow("id ref", self._id_spin)
        mform.addRow("iq ref", self._iq_spin)

        id_row = QHBoxLayout()
        for label, delta in (("id −", -0.1), ("id 0", None), ("id +", 0.1)):
            b = QPushButton(label)
            b.clicked.connect(
                lambda _c=False, d=delta, s=self._id_spin: self._nudge(s, d))
            id_row.addWidget(b)
        mform.addRow(id_row)

        iq_row = QHBoxLayout()
        for label, delta in (("iq −", -0.1), ("iq 0", None), ("iq +", 0.1)):
            b = QPushButton(label)
            b.clicked.connect(
                lambda _c=False, d=delta, s=self._iq_spin: self._nudge(s, d))
            iq_row.addWidget(b)
        mform.addRow(iq_row)

        self._id_spin.valueChanged.connect(self._on_id_changed)
        self._iq_spin.valueChanged.connect(self._on_iq_changed)
        root.addWidget(motion)

        align = QGroupBox("Alignment")
        al = QVBoxLayout(align)
        self._align_btn = QPushButton("Align axis")
        self._align_btn.clicked.connect(self._on_align)
        al.addWidget(self._align_btn)
        root.addWidget(align)

        self._estop_btn = QPushButton("E-STOP")
        self._estop_btn.setStyleSheet(make_estop_button_qss())
        self._estop_btn.setCursor(Qt.PointingHandCursor)
        self._estop_btn.clicked.connect(self._on_estop)
        root.addWidget(self._estop_btn)

        self._status = QLabel("")
        self._status.setStyleSheet("color: #ef5350;")
        root.addWidget(self._status)
        root.addStretch(1)

        self.set_actions_enabled(False)

    def set_client(self, client: Optional[TunerClient]) -> None:
        self._client = client

    def set_actions_enabled(self, on: bool) -> None:
        for w in (
            self._override_box,
            self._id_spin,
            self._iq_spin,
            self._align_btn,
            self._estop_btn,
        ):
            w.setEnabled(on)
        for btn in self.findChildren(QPushButton):
            if btn is not self._estop_btn:
                btn.setEnabled(on)

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
