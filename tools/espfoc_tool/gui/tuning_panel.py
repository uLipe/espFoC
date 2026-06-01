"""Tuning panel: live gain readout, manual override, MPZ retune."""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QCheckBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from ..protocol import AxisStateFlag, TunerClient, TunerError
from .tuner_poll_worker import TunerPollSnapshot
from .theme import make_badge_qss


def _spin(minimum: float, maximum: float,
          decimals: int, value: float,
          step: float = 0.1,
          suffix: str = "") -> QDoubleSpinBox:
    """Convenience wrapper matching Qt's (min, max) ordering and with an
    optional unit suffix. Type-in is always allowed — the step value
    only controls the ± buttons / scroll wheel."""
    if minimum > maximum:
        minimum, maximum = maximum, minimum
    box = QDoubleSpinBox()
    box.setRange(minimum, maximum)
    box.setDecimals(decimals)
    box.setSingleStep(step)
    box.setValue(value)
    if suffix:
        box.setSuffix(suffix)
    # Keep the spinner buttons; make the field wide enough so long
    # suffixes like " Ω" do not visually collide with the number.
    box.setMinimumWidth(140)
    return box


class TuningPanel(QWidget):
    """Left-hand side of the main window: controls + live readout.

    Most slots use short TunerClient round-trips. Long executables
    (align) run in a QThread; MainWindow pauses the background poll
    worker to keep bus usage serialized.

    Periodic tuner reads run on :class:`TunerPollWorker`; results arrive via
    :meth:`apply_poll_snapshot`."""

    _logFromReader = Signal(str)
    long_operation = Signal(bool)
    poll_refresh_requested = Signal(bool)

    def __init__(self, client: Optional[TunerClient] = None) -> None:
        super().__init__()
        self._client = client
        self.last_poll_ok: bool = False
        self._cal_present = False
        self.last_axis_state: Optional[AxisStateFlag] = None

        # The whole panel sits inside a QScrollArea so a small window
        # gets a vertical scroll bar instead of clipping content. Keeps
        # every existing widget the operator already knows.
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setFrameShape(QScrollArea.NoFrame)
        outer.addWidget(scroll)

        body = QWidget()
        scroll.setWidget(body)
        root = QVBoxLayout(body)

        # --- Axis state badge (single colored pill, dominant flag) ---
        state_row = QHBoxLayout()
        state_row.addWidget(QLabel("Axis status:"))
        self._state_label = QLabel("OFFLINE")
        label, qss = make_badge_qss("OFFLINE")
        self._state_label.setText(label)
        self._state_label.setStyleSheet(qss)
        self._state_label.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
        state_row.addWidget(self._state_label)
        state_row.addStretch(1)
        root.addLayout(state_row)

        # --- Live gains readout ---
        live_box = QGroupBox("Live gains")
        live_form = QFormLayout(live_box)
        self._kp_label = QLabel("-")
        self._ki_label = QLabel("-")
        self._lim_label = QLabel("-")
        self._vmax_label = QLabel("-")
        for lbl in (self._kp_label, self._ki_label,
                    self._lim_label, self._vmax_label):
            lbl.setStyleSheet("font-family: monospace;")
        self._fc_label = QLabel("-")
        self._fc_label.setStyleSheet("font-family: monospace;")
        self._loop_fs_label = QLabel("-")
        self._loop_fs_label.setStyleSheet("font-family: monospace;")
        live_form.addRow("Kp [V/A]", self._kp_label)
        live_form.addRow("Ki [V/(A·s)]", self._ki_label)
        live_form.addRow("ILim [V]", self._lim_label)
        live_form.addRow("Vmax [V]", self._vmax_label)
        live_form.addRow("I-LPF fc [Hz]", self._fc_label)
        live_form.addRow("Loop fs [Hz]", self._loop_fs_label)
        root.addWidget(live_box)

        # --- Manual gain editor ---
        manual = QGroupBox("Manual gain override")
        mform = QFormLayout(manual)
        self._kp_spin = _spin(0.0, 500.0, 4, 1.46, step=0.01, suffix=" V/A")
        self._ki_spin = _spin(0.0, 1_000_000.0, 2, 659.17, step=10.0,
                              suffix=" V/(A·s)")
        self._lim_spin = _spin(0.0, 200.0, 3, 12.0, step=0.1, suffix=" V")
        mform.addRow("Kp", self._kp_spin)
        mform.addRow("Ki", self._ki_spin)
        mform.addRow("ILim", self._lim_spin)
        btn_apply = QPushButton("Apply manual gains")
        btn_apply.clicked.connect(self._on_apply_manual)
        mform.addRow(btn_apply)
        root.addWidget(manual)

        fc_box = QGroupBox("Current filter")
        fc_form = QFormLayout(fc_box)
        self._fc_spin = _spin(10.0, 20000.0, 1, 300.0,
                              step=10.0, suffix=" Hz")
        btn_fc = QPushButton("Apply I-LPF cutoff")
        btn_fc.clicked.connect(self._on_apply_fc)
        fc_form.addRow("I-LPF fc", self._fc_spin)
        fc_form.addRow(btn_fc)
        root.addWidget(fc_box)

        self._cal_label = QLabel("calibration: -")
        self._cal_label.setStyleSheet("font-family: monospace; color: #9aa0a6;")
        root.addWidget(self._cal_label)

        # --- Log channel viewer ---
        self._log_view = QPlainTextEdit()
        self._log_view.setReadOnly(True)
        self._log_view.setMaximumBlockCount(80)
        self._log_view.setMaximumHeight(110)
        f = QFont("monospace")
        f.setPointSize(9)
        self._log_view.setFont(f)
        root.addWidget(self._log_view)
        self._logFromReader.connect(self._append_log)
        self.rebind_log_reader()

        # --- Status / errors bar ---
        self._status = QLabel("")
        self._status.setStyleSheet("color: #c62828;")
        root.addWidget(self._status)
        root.addStretch(1)

    # --- Public slots (driven by MainWindow / poll worker) -----------------

    def set_loop_rate_hz(self, fs_hz: float) -> None:
        """Receive the firmware's current PI sample rate (read once
        on connect by MainWindow). Cached so the UI can show it
        without a round-trip per refresh — the value is fixed for
        the duration of a session."""
        if fs_hz > 1.0:
            self._loop_fs_hz = fs_hz
            self._loop_fs_label.setText(f"{fs_hz:9.1f}")

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
        try:
            self._client.reader.register_log_callback(None)
        except Exception:
            pass

    def request_full_tuner_poll(self) -> None:
        """Ask the background worker for Vmax + NVS-present on the next pass."""
        self.poll_refresh_requested.emit(True)

    def apply_poll_snapshot(self, snap: TunerPollSnapshot) -> None:
        """Apply a snapshot emitted by :class:`TunerPollWorker` (GUI thread)."""
        self.last_poll_ok = snap.last_poll_ok
        self._cal_present = snap.cal_present
        self._status.setText("")
        self._kp_label.setText(f"{snap.kp:9.4f}")
        self._ki_label.setText(f"{snap.ki:9.2f}")
        self._lim_label.setText(f"{snap.lim:9.3f}")
        self._vmax_label.setText(f"{snap.vmax:9.3f}")
        self._fc_label.setText(f"{snap.fc:9.1f}")
        self.last_axis_state = AxisStateFlag(snap.state)
        badge_key = self._badge_key_for_state(self.last_axis_state)
        label, qss = make_badge_qss(badge_key)
        self._state_label.setText(label)
        self._state_label.setStyleSheet(qss)
        self._cal_label.setText("calibration: " +
                                ("\u2713 present in NVS" if snap.cal_present
                                 else "\u2717 none stored"))
    def apply_poll_error(self, msg: str) -> None:
        """Worker poll failed (transport / tuner error)."""
        self._status.setText(msg)
        self.last_poll_ok = False

    # --- Handlers ----------------------------------------------------------

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

    def _set_axis_badge_key(self, key: str) -> None:
        label, qss = make_badge_qss(key)
        self._state_label.setText(label)
        self._state_label.setStyleSheet(qss)

    # --- LOG channel viewer --------------------------------------------

    def _on_log_reader(self, seq: int, payload: bytes) -> None:
        """Reader thread context — bounce to the Qt thread."""
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
        """Apply NVS-shadow values to spinboxes (GUI thread only; no I/O)."""
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
        """Pull R/L/BW and live controls from the target (blocks on :class:`TunerClient`).

        Prefer having the poll worker read and call :meth:`apply_nvs_shadow_floats`
        from the GUI thread so the main thread never holds the bus lock."""
        try:
            kp = self._client.read_kp()
            ki = self._client.read_ki()
            fc = self._client.read_current_filter_fc()
        except TunerError:
            return
        self.apply_nvs_shadow_floats(0.0, 0.0, 0.0, kp, ki, fc)

    @staticmethod
    def _badge_key_for_state(s: AxisStateFlag) -> str:
        """Pick the dominant flag and map it to a badge palette key."""
        if s & AxisStateFlag.RUNNING:
            return "RUNNING"
        if s & AxisStateFlag.ALIGNED:
            return "ALIGNED"
        if s & AxisStateFlag.INITIALIZED:
            return "INIT"
        return "OFFLINE"
