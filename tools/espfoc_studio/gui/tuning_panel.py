"""Tuning panel: live gain readout, manual override, MPZ retune."""

from __future__ import annotations

from typing import Callable, Optional

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
    QVBoxLayout,
    QWidget,
)

from ..protocol import AxisStateFlag, TunerClient, TunerError


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

    All slots reach out to a TunerClient directly. We keep every
    round-trip short so the UI does not feel janky even on real serial."""

    _logFromReader = Signal(str)

    def __init__(self, client: TunerClient,
                 on_params_changed: Optional[Callable[[float, float, float,
                                                       float, float], None]] = None) -> None:
        super().__init__()
        self._client = client
        self._on_params_changed = on_params_changed
        self._last_motor_r = 1.08
        self._last_motor_l = 0.0018
        self._last_bw = 150.0
        self._cal_present = False

        root = QVBoxLayout(self)

        # --- Axis state badge ---
        self._state_label = QLabel("axis: disconnected")
        self._state_label.setAlignment(Qt.AlignLeft)
        self._state_label.setStyleSheet("font-family: monospace;")
        root.addWidget(self._state_label)

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

        # --- MPZ retune ---
        mpz = QGroupBox("MPZ recompute")
        mfrm = QFormLayout(mpz)
        # L is stored in Henries but displayed in milli-Henries to keep the
        # typing experience reasonable for normal motors (0.1 mH .. 100 mH).
        # Conversions happen in _on_mpz and in _notify_params_changed.
        self._r_spin = _spin(0.001, 1000.0, 4, self._last_motor_r,
                             step=0.01, suffix=" Ω")
        self._l_mh_spin = _spin(0.001, 10_000.0, 4,
                                self._last_motor_l * 1000.0,
                                step=0.01, suffix=" mH")
        self._bw_spin = _spin(1.0, 5000.0, 1, self._last_bw,
                              step=10.0, suffix=" Hz")
        # Current-sense LPF cutoff. Lives next to the recompute group
        # because it is always the second knob the operator reaches for
        # after dialling Kp / Ki.
        self._fc_spin = _spin(10.0, 20000.0, 1, 300.0,
                              step=10.0, suffix=" Hz")
        mfrm.addRow("R", self._r_spin)
        mfrm.addRow("L", self._l_mh_spin)
        mfrm.addRow("Bandwidth", self._bw_spin)
        mfrm.addRow("I-LPF fc", self._fc_spin)
        btn_mpz = QPushButton("Recompute gains")
        btn_mpz.clicked.connect(self._on_mpz)
        btn_fc = QPushButton("Apply current LPF cutoff")
        btn_fc.clicked.connect(self._on_apply_fc)
        mfrm.addRow(btn_mpz)
        mfrm.addRow(btn_fc)
        root.addWidget(mpz)
        # Push any initial model to the Analysis view.
        if self._on_params_changed is not None:
            self._on_params_changed(self._last_motor_r, self._last_motor_l,
                                    self._last_bw,
                                    self._kp_spin.value(),
                                    self._ki_spin.value())
        for spin in (self._r_spin, self._l_mh_spin, self._bw_spin,
                     self._kp_spin, self._ki_spin):
            spin.valueChanged.connect(self._notify_params_changed)

        # --- Override + motion targets ---
        ovr = QGroupBox("Tuner override / motion")
        ovr_layout = QVBoxLayout(ovr)
        self._override_box = QCheckBox("Override active")
        self._override_box.toggled.connect(self._on_override_toggled)
        ovr_layout.addWidget(self._override_box)
        target_form = QFormLayout()
        self._iq_spin = _spin(-50.0, 50.0, 4, 0.0, step=0.1, suffix=" A")
        self._id_spin = _spin(-50.0, 50.0, 4, 0.0, step=0.1, suffix=" A")
        self._iq_spin.setEnabled(False)
        self._id_spin.setEnabled(False)
        self._iq_spin.valueChanged.connect(self._on_iq_changed)
        self._id_spin.valueChanged.connect(self._on_id_changed)
        target_form.addRow("iq ref [A]", self._iq_spin)
        target_form.addRow("id ref [A]", self._id_spin)
        ovr_layout.addLayout(target_form)
        root.addWidget(ovr)

        # --- Alignment + calibration ---
        align = QGroupBox("Alignment & calibration")
        align_layout = QVBoxLayout(align)
        btn_align = QPushButton("Align axis")
        btn_align.clicked.connect(self._on_align)
        align_layout.addWidget(btn_align)
        self._cal_label = QLabel("calibration: -")
        self._cal_label.setStyleSheet("font-family: monospace; color: #9aa0a6;")
        align_layout.addWidget(self._cal_label)
        cal_btns = QHBoxLayout()
        for label, slot in (("Save to NVS", self._on_persist),
                            ("Load NVS",    self._on_load),
                            ("Erase",       self._on_erase)):
            b = QPushButton(label)
            b.clicked.connect(slot)
            cal_btns.addWidget(b)
        align_layout.addLayout(cal_btns)
        root.addWidget(align)

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
        try:
            self._client.reader.register_log_callback(self._on_log_reader)
        except Exception:
            pass

        # --- Status / errors bar ---
        self._status = QLabel("")
        self._status.setStyleSheet("color: #c62828;")
        root.addWidget(self._status)
        root.addStretch(1)

    # --- Public slots (driven by MainWindow's timer) -----------------------

    def set_loop_rate_hz(self, fs_hz: float) -> None:
        """Receive the firmware's current PI sample rate (read once
        on connect by MainWindow). Cached so poll() can show it
        without a round-trip per refresh — the value is fixed for
        the duration of a session."""
        if fs_hz > 1.0:
            self._loop_fs_hz = fs_hz
            self._loop_fs_label.setText(f"{fs_hz:9.1f}")

    def poll(self) -> None:
        """Refresh the live readout. Called periodically by MainWindow."""
        try:
            kp = self._client.read_kp()
            ki = self._client.read_ki()
            lim = self._client.read_int_lim()
            vmax = self._client.read_v_max()
            fc = self._client.read_current_filter_fc()
            state = self._client.read_axis_state()
            present = self._client.is_calibration_present()
        except TunerError as e:
            self._status.setText(str(e))
            return
        self._status.setText("")
        self._kp_label.setText(f"{kp:9.4f}")
        self._ki_label.setText(f"{ki:9.2f}")
        self._lim_label.setText(f"{lim:9.3f}")
        self._vmax_label.setText(f"{vmax:9.3f}")
        self._fc_label.setText(f"{fc:9.1f}")
        self._state_label.setText(f"axis: {self._format_state(state)}")
        self._cal_present = present
        self._cal_label.setText("calibration: " +
                                ("\u2713 present in NVS" if present
                                 else "\u2717 none stored"))
        # Keep the override checkbox in sync without re-emitting signals.
        override_active = bool(state & AxisStateFlag.TUNER_OVERRIDE)
        self._override_box.blockSignals(True)
        self._override_box.setChecked(override_active)
        self._override_box.blockSignals(False)
        self._iq_spin.setEnabled(override_active)
        self._id_spin.setEnabled(override_active)

    # --- Handlers ----------------------------------------------------------

    def _on_apply_manual(self) -> None:
        try:
            self._client.write_kp(self._kp_spin.value())
            self._client.write_ki(self._ki_spin.value())
            self._client.write_int_lim(self._lim_spin.value())
        except TunerError as e:
            self._status.setText(str(e))
            return
        self._status.setText("")
        self._notify_params_changed()

    def _on_apply_fc(self) -> None:
        try:
            self._client.write_current_filter_fc(self._fc_spin.value())
        except TunerError as e:
            self._status.setText(str(e))
            return
        self._status.setText("")

    def _on_mpz(self) -> None:
        motor_l_h = self._l_mh_spin.value() * 1e-3  # mH -> H
        try:
            self._client.recompute_gains(
                motor_r=self._r_spin.value(),
                motor_l=motor_l_h,
                bw_hz=self._bw_spin.value())
        except TunerError as e:
            self._status.setText(str(e))
            return
        self._status.setText("")
        self._last_motor_r = self._r_spin.value()
        self._last_motor_l = motor_l_h
        self._last_bw = self._bw_spin.value()
        # Wait a tick for the firmware to update, then refresh readouts
        # so the manual spinboxes line up with the new design.
        self.poll()
        try:
            self._kp_spin.blockSignals(True)
            self._ki_spin.blockSignals(True)
            self._kp_spin.setValue(self._client.read_kp())
            self._ki_spin.setValue(self._client.read_ki())
        except TunerError:
            pass
        finally:
            self._kp_spin.blockSignals(False)
            self._ki_spin.blockSignals(False)
        self._notify_params_changed()

    def _on_override_toggled(self, checked: bool) -> None:
        try:
            if checked:
                self._client.override_on()
            else:
                self._client.override_off()
        except TunerError as e:
            self._status.setText(str(e))
            self._override_box.blockSignals(True)
            self._override_box.setChecked(not checked)
            self._override_box.blockSignals(False)
            return
        self._status.setText("")
        self._iq_spin.setEnabled(checked)
        self._id_spin.setEnabled(checked)

    def _on_iq_changed(self, value: float) -> None:
        try:
            self._client.write_target_iq(value)
        except TunerError as e:
            self._status.setText(str(e))

    def _on_id_changed(self, value: float) -> None:
        try:
            self._client.write_target_id(value)
        except TunerError as e:
            self._status.setText(str(e))

    def _on_align(self) -> None:
        self._append_log("> alignment requested")
        try:
            self._client.align_axis()
        except TunerError as e:
            self._status.setText(str(e))
            return
        self._status.setText("")

    def _on_persist(self) -> None:
        try:
            self._client.persist_calibration(
                motor_r=self._r_spin.value(),
                motor_l=self._l_mh_spin.value() * 1e-3,
                bandwidth_hz=self._bw_spin.value())
        except TunerError as e:
            self._status.setText(str(e))
            return
        self._status.setText("")

    def _on_load(self) -> None:
        try:
            self._client.load_calibration()
        except TunerError as e:
            self._status.setText(str(e))
            return
        self.poll()

    def _on_erase(self) -> None:
        try:
            self._client.erase_calibration()
        except TunerError as e:
            self._status.setText(str(e))
            return
        self.poll()

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

    def _notify_params_changed(self) -> None:
        if self._on_params_changed is None:
            return
        motor_l_h = self._l_mh_spin.value() * 1e-3  # mH -> H for the model
        self._on_params_changed(self._r_spin.value(), motor_l_h,
                                self._bw_spin.value(),
                                self._kp_spin.value(), self._ki_spin.value())

    @staticmethod
    def _format_state(s: AxisStateFlag) -> str:
        pretty = []
        for flag in (AxisStateFlag.INITIALIZED, AxisStateFlag.ALIGNED,
                     AxisStateFlag.RUNNING, AxisStateFlag.TUNER_OVERRIDE):
            pretty.append(f"{'+' if s & flag else '-'}{flag.name}")
        return " ".join(pretty)
