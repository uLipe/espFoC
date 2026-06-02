"""Tuning panel: live gain readout, manual override, MPZ retune."""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from ..protocol import TunerClient, TunerError
from . import labels as L
from .tuner_poll_worker import TunerPollSnapshot
from .theme import monospace_font
from .buttons import action_button
from .widgets import LiveMetricGrid, SurfaceCard, spin_box


class TuningPanel(QWidget):
    """Left column of Config: status, live metrics, overrides, log."""

    _logFromReader = Signal(str)
    long_operation = Signal(bool)
    poll_refresh_requested = Signal(bool)

    def __init__(
        self,
        client: Optional[TunerClient] = None,
        *,
        scrollable: bool = True,
    ) -> None:
        super().__init__()
        self._client = client
        self.last_poll_ok: bool = False
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        body = QWidget()
        root = QVBoxLayout(body)
        root.setContentsMargins(0, 0, 8, 0)
        root.setSpacing(12)

        live_card = SurfaceCard("Live")
        self._live_grid = LiveMetricGrid([
            L.PROPORTIONAL_GAIN,
            L.INTEGRAL_GAIN,
            L.CURRENT_LIMIT,
            L.VOLTAGE_LIMIT,
            L.CURRENT_FILTER,
            L.LOOP_RATE,
        ])
        live_card.body_layout.addWidget(self._live_grid)
        root.addWidget(live_card)

        editor_card = SurfaceCard("Editor")
        mform = QFormLayout()
        mform.setLabelAlignment(Qt.AlignRight)
        self._kp_spin = spin_box(0.0, 500.0, 4, 1.46, step=0.01, suffix=" V/A")
        self._ki_spin = spin_box(0.0, 1_000_000.0, 2, 659.17, step=10.0,
                                 suffix=" V/(A·s)")
        self._lim_spin = spin_box(0.0, 200.0, 3, 12.0, step=0.1, suffix=" V")
        self._fc_spin = spin_box(10.0, 20000.0, 1, 300.0, step=10.0, suffix=" Hz")
        mform.addRow(L.PROPORTIONAL_GAIN, self._kp_spin)
        mform.addRow(L.INTEGRAL_GAIN, self._ki_spin)
        mform.addRow(L.CURRENT_LIMIT, self._lim_spin)
        mform.addRow(L.CURRENT_FILTER, self._fc_spin)
        btn_apply = action_button("Apply gains", "BtnDefault")
        btn_apply.clicked.connect(self._on_apply_manual)
        btn_fc = action_button("Apply filter", "BtnDefault")
        btn_fc.clicked.connect(self._on_apply_fc)
        btn_row = QHBoxLayout()
        btn_row.setSpacing(8)
        btn_row.addWidget(btn_apply)
        btn_row.addWidget(btn_fc)
        btn_row.addStretch(1)
        editor_card.body_layout.addLayout(mform)
        editor_card.body_layout.addLayout(btn_row)
        root.addWidget(editor_card)

        log_card = SurfaceCard("Log")
        self._log_view = QPlainTextEdit()
        self._log_view.setReadOnly(True)
        self._log_view.setMaximumBlockCount(80)
        self._log_view.setMaximumHeight(120)
        self._log_view.setFont(monospace_font(9))
        log_card.body_layout.addWidget(self._log_view)
        root.addWidget(log_card)

        self._status = QLabel("")
        self._status.setStyleSheet("color: #c62828; font-size: 11px;")
        root.addWidget(self._status)

        if scrollable:
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            scroll.setFrameShape(QScrollArea.NoFrame)
            scroll.setWidget(body)
            outer.addWidget(scroll)
        else:
            outer.addWidget(body)

        self._loop_fs_hz = 0.0
        self._logFromReader.connect(self._append_log)
        self.rebind_log_reader()

    def set_loop_rate_hz(self, fs_hz: float) -> None:
        if fs_hz > 1.0:
            self._loop_fs_hz = fs_hz
            self._live_grid.metric(5).set_value(f"{fs_hz:.1f} Hz")

    def set_client(self, client: Optional[TunerClient]) -> None:
        self._client = client
        self.rebind_log_reader()

    def rebind_log_reader(self) -> None:
        if self._client is None:
            return
        try:
            self._client.reader.register_log_callback(self._on_log_reader)
        except Exception:
            pass

    def detach_log_reader(self) -> None:
        if self._client is None:
            return
        try:
            self._client.reader.register_log_callback(None)
        except Exception:
            pass

    def request_full_tuner_poll(self) -> None:
        self.poll_refresh_requested.emit(True)

    def apply_poll_snapshot(self, snap: TunerPollSnapshot) -> None:
        self.last_poll_ok = snap.last_poll_ok
        self._status.setText("")
        fs_str = (f"{self._loop_fs_hz:.1f} Hz" if self._loop_fs_hz > 1.0
                  else "—")
        self._live_grid.set_values([
            f"{snap.kp:.4f} V/A",
            f"{snap.ki:.1f} V/(A·s)",
            f"{snap.lim:.2f} V",
            f"{snap.vmax:.2f} V",
            f"{snap.fc:.0f} Hz",
            fs_str,
        ])
    def apply_poll_error(self, msg: str) -> None:
        self._status.setText(msg)
        self.last_poll_ok = False

    def _on_apply_manual(self) -> None:
        if self._client is None:
            return
        try:
            self._client.write_kp(self._kp_spin.value())
            self._client.write_ki(self._ki_spin.value())
            self._client.write_int_lim(self._lim_spin.value())
        except TunerError as e:
            self._status.setText(str(e))
            return
        self._status.setText("")

    def _on_apply_fc(self) -> None:
        if self._client is None:
            return
        try:
            self._client.write_current_filter_fc(self._fc_spin.value())
        except TunerError as e:
            self._status.setText(str(e))
            return
        self._status.setText("")

    def set_actions_enabled(self, on: bool) -> None:
        for w in self.findChildren(QPushButton):
            w.setEnabled(on)
        for w in self.findChildren(QDoubleSpinBox):
            w.setEnabled(on)

    def _on_log_reader(self, seq: int, payload: bytes) -> None:
        try:
            self._logFromReader.emit(payload.decode("ascii", errors="replace"))
        except Exception:
            pass

    def _append_log(self, line: str) -> None:
        if line:
            self._log_view.appendPlainText(line.rstrip("\n"))

    def apply_nvs_shadow_floats(
            self,
            r: float,
            l_h: float,
            bw: float,
            kp: float,
            ki: float,
            fc: float,
    ) -> None:
        for sp in (self._kp_spin, self._ki_spin, self._fc_spin):
            sp.blockSignals(True)
        try:
            self._kp_spin.setValue(kp)
            self._ki_spin.setValue(ki)
            if fc > 0.0:
                self._fc_spin.setValue(fc)
        finally:
            for sp in (self._kp_spin, self._ki_spin, self._fc_spin):
                sp.blockSignals(False)

    def sync_motor_from_nvs_shadows(self) -> None:
        if self._client is None:
            return
        try:
            kp = self._client.read_kp()
            ki = self._client.read_ki()
            fc = self._client.read_current_filter_fc()
        except TunerError:
            return
        self.apply_nvs_shadow_floats(0.0, 0.0, 0.0, kp, ki, fc)
