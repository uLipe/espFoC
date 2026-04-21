"""Hardware configuration tab.

Pure data-entry form: the user picks a target chip, an inverter
topology with its PWM pins, a rotor sensor with its bus pins, an
ADC current-sense configuration and a motor profile. The values feed
the Generate App tab so the produced application carries the right
pin map and Kconfig defaults baked in.

The panel does not read from the firmware in this revision — the
operator types in what they wired or what they intend to wire on the
production board. A follow-up PR will add a "Read from firmware"
button that pulls the live target's config back through the tuner
protocol.

Persistence is intentionally ephemeral: closing the GUI loses the
form, the next launch starts fresh on the default target.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QSpinBox,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)


# --- Target inventory --------------------------------------------------

# Hand-picked GPIO ranges that work for general PWM / GPIO output. Excludes
# input-only and obviously-reserved pins (strapping pins are still listed
# so the operator stays in charge of the trade-off).
_TARGET_PINS: Dict[str, List[int]] = {
    "esp32":   list(range(0, 24)) + [25, 26, 27, 32, 33],
    "esp32s3": list(range(0, 22)) + list(range(35, 46)) + [47, 48],
    "esp32p4": list(range(0, 55)),
}

_TARGET_ADC_CHANNELS: Dict[str, int] = {
    # Maximum channel index per ADC unit on the target. ADC1 is enough
    # for our three-phase shunt sensing.
    "esp32":   8,
    "esp32s3": 10,
    "esp32p4": 8,
}


@dataclass
class HardwareConfig:
    target: str = "esp32s3"
    pwm_type: str = "6pwm_mcpwm"
    pwm_freq_hz: int = 20000
    dc_link_v: float = 12.0
    pin_u_hi: int = 5
    pin_v_hi: int = 6
    pin_w_hi: int = 7
    pin_u_lo: int = 15
    pin_v_lo: int = 16
    pin_w_lo: int = 17
    sensor_type: str = "as5600_i2c"
    sensor_cfg: Dict[str, int] = field(
        default_factory=lambda: {"sda": 4, "scl": 5, "i2c_port": 0,
                                 "mosi": 11, "miso": 13, "clk": 12, "cs": 10,
                                 "spi_host": 2,
                                 "pcnt_a": 18, "pcnt_b": 19,
                                 "counts_per_rev": 4096})
    isensor_mode: str = "continuous"
    adc_unit: int = 1
    adc_ch_u: int = 0
    adc_ch_v: int = 1
    adc_ch_w: int = 2
    amp_gain: float = 50.0
    shunt_ohm: float = 0.01
    pole_pairs: int = 7
    motor_profile: str = "default"


def _list_motor_profiles() -> List[str]:
    """Find scripts/motors/*.json relative to the package root."""
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.normpath(os.path.join(here, "..", "..", ".."))
    motors_dir = os.path.join(root, "scripts", "motors")
    profiles: List[str] = []
    try:
        for name in os.listdir(motors_dir):
            if name.endswith(".json"):
                profiles.append(name[:-5])
    except OSError:
        pass
    profiles.sort()
    if not profiles:
        profiles = ["default"]
    return profiles


def _populate_pin_combo(combo: QComboBox, target: str,
                       preferred: Optional[int] = None) -> None:
    """Refill a QComboBox with valid GPIOs for the given target. Tries
    to keep the previously selected pin if it is still valid."""
    pins = _TARGET_PINS.get(target, _TARGET_PINS["esp32"])
    current = preferred if preferred is not None else combo.currentData()
    combo.blockSignals(True)
    combo.clear()
    for p in pins:
        combo.addItem(f"GPIO {p}", p)
    if current is not None and current in pins:
        combo.setCurrentIndex(pins.index(current))
    combo.blockSignals(False)


def _populate_adc_combo(combo: QComboBox, target: str,
                       preferred: Optional[int] = None) -> None:
    n = _TARGET_ADC_CHANNELS.get(target, 8)
    current = preferred if preferred is not None else combo.currentData()
    combo.blockSignals(True)
    combo.clear()
    for i in range(n):
        combo.addItem(f"ADC_CHANNEL_{i}", i)
    if current is not None and current < n:
        combo.setCurrentIndex(current)
    combo.blockSignals(False)


class HardwarePanel(QWidget):
    """Form view that exposes a HardwareConfig via get_config()."""

    configChanged = Signal()

    def __init__(self, initial: Optional[HardwareConfig] = None) -> None:
        super().__init__()
        self._cfg = initial or HardwareConfig()

        root = QVBoxLayout(self)

        # --- Target chip ---
        self._target_combo = QComboBox()
        for t in ("esp32", "esp32s3", "esp32p4"):
            self._target_combo.addItem(t, t)
        self._target_combo.setCurrentIndex(
            ("esp32", "esp32s3", "esp32p4").index(self._cfg.target))
        self._target_combo.currentTextChanged.connect(self._on_target_changed)
        target_box = QGroupBox("Target chip")
        tl = QFormLayout(target_box)
        tl.addRow("Target", self._target_combo)
        root.addWidget(target_box)

        # --- PWM / inverter ---
        pwm_box = QGroupBox("Inverter PWM")
        pf = QFormLayout(pwm_box)
        self._pwm_type = QComboBox()
        for k, label in (("3pwm_mcpwm", "3-PWM MCPWM"),
                         ("6pwm_mcpwm", "6-PWM MCPWM (HW dead-time)")):
            self._pwm_type.addItem(label, k)
        idx = 0 if self._cfg.pwm_type == "3pwm_mcpwm" else 1
        self._pwm_type.setCurrentIndex(idx)
        self._pwm_type.currentTextChanged.connect(self._on_pwm_type_changed)
        pf.addRow("Topology", self._pwm_type)

        self._pwm_freq = QSpinBox()
        self._pwm_freq.setRange(1000, 200000)
        self._pwm_freq.setSingleStep(1000)
        self._pwm_freq.setValue(self._cfg.pwm_freq_hz)
        self._pwm_freq.setSuffix(" Hz")
        self._pwm_freq.valueChanged.connect(lambda *_: self.configChanged.emit())
        pf.addRow("PWM rate", self._pwm_freq)

        self._dc_link = QDoubleSpinBox()
        self._dc_link.setRange(1.0, 200.0)
        self._dc_link.setDecimals(2)
        self._dc_link.setValue(self._cfg.dc_link_v)
        self._dc_link.setSuffix(" V")
        self._dc_link.valueChanged.connect(lambda *_: self.configChanged.emit())
        pf.addRow("DC link", self._dc_link)

        self._pin_u_hi = QComboBox()
        self._pin_v_hi = QComboBox()
        self._pin_w_hi = QComboBox()
        self._pin_u_lo = QComboBox()
        self._pin_v_lo = QComboBox()
        self._pin_w_lo = QComboBox()
        for c, p in ((self._pin_u_hi, self._cfg.pin_u_hi),
                     (self._pin_v_hi, self._cfg.pin_v_hi),
                     (self._pin_w_hi, self._cfg.pin_w_hi),
                     (self._pin_u_lo, self._cfg.pin_u_lo),
                     (self._pin_v_lo, self._cfg.pin_v_lo),
                     (self._pin_w_lo, self._cfg.pin_w_lo)):
            _populate_pin_combo(c, self._cfg.target, p)
            c.currentIndexChanged.connect(lambda *_: self.configChanged.emit())

        pf.addRow("U high pin", self._pin_u_hi)
        pf.addRow("V high pin", self._pin_v_hi)
        pf.addRow("W high pin", self._pin_w_hi)
        pf.addRow("U low pin",  self._pin_u_lo)
        pf.addRow("V low pin",  self._pin_v_lo)
        pf.addRow("W low pin",  self._pin_w_lo)
        root.addWidget(pwm_box)

        # --- Rotor sensor ---
        rs_box = QGroupBox("Rotor sensor")
        rf = QFormLayout(rs_box)
        self._sensor_type = QComboBox()
        for key, label in (("as5600_i2c", "AS5600 (I2C)"),
                           ("as5048_spi", "AS5048A (SPI)"),
                           ("pcnt",       "Quadrature (PCNT)"),
                           ("open_loop",  "Open-loop (no sensor)")):
            self._sensor_type.addItem(label, key)
        self._sensor_type.setCurrentIndex(
            ("as5600_i2c", "as5048_spi", "pcnt", "open_loop").index(
                self._cfg.sensor_type))
        self._sensor_type.currentTextChanged.connect(self._on_sensor_changed)
        rf.addRow("Type", self._sensor_type)

        self._sensor_stack = QStackedWidget()
        self._sensor_stack.addWidget(self._build_i2c_form())
        self._sensor_stack.addWidget(self._build_spi_form())
        self._sensor_stack.addWidget(self._build_pcnt_form())
        self._sensor_stack.addWidget(QWidget())  # open_loop = empty
        rf.addRow(self._sensor_stack)
        self._sensor_stack.setCurrentIndex(self._sensor_type.currentIndex())
        root.addWidget(rs_box)

        # --- Current sensor ---
        is_box = QGroupBox("Current sensor (ADC shunt)")
        isf = QFormLayout(is_box)
        self._isensor_mode = QComboBox()
        for k, l in (("continuous", "Continuous (PWM-synchronised)"),
                     ("oneshot",    "One-shot"),
                     ("none",       "None (voltage mode)")):
            self._isensor_mode.addItem(l, k)
        self._isensor_mode.setCurrentIndex(
            ("continuous", "oneshot", "none").index(self._cfg.isensor_mode))
        self._isensor_mode.currentTextChanged.connect(
            lambda *_: self.configChanged.emit())
        isf.addRow("Mode", self._isensor_mode)

        self._adc_unit = QSpinBox()
        self._adc_unit.setRange(1, 2)
        self._adc_unit.setValue(self._cfg.adc_unit)
        self._adc_unit.valueChanged.connect(lambda *_: self.configChanged.emit())
        isf.addRow("ADC unit", self._adc_unit)

        self._adc_ch_u = QComboBox()
        self._adc_ch_v = QComboBox()
        self._adc_ch_w = QComboBox()
        for c, v in ((self._adc_ch_u, self._cfg.adc_ch_u),
                     (self._adc_ch_v, self._cfg.adc_ch_v),
                     (self._adc_ch_w, self._cfg.adc_ch_w)):
            _populate_adc_combo(c, self._cfg.target, v)
            c.currentIndexChanged.connect(lambda *_: self.configChanged.emit())
        isf.addRow("Channel U", self._adc_ch_u)
        isf.addRow("Channel V", self._adc_ch_v)
        isf.addRow("Channel W", self._adc_ch_w)

        self._amp_gain = QDoubleSpinBox()
        self._amp_gain.setRange(1.0, 1000.0)
        self._amp_gain.setDecimals(1)
        self._amp_gain.setValue(self._cfg.amp_gain)
        self._amp_gain.setSuffix(" V/V")
        self._amp_gain.valueChanged.connect(lambda *_: self.configChanged.emit())
        isf.addRow("Amp gain", self._amp_gain)

        self._shunt = QDoubleSpinBox()
        self._shunt.setRange(0.0001, 1.0)
        self._shunt.setDecimals(4)
        self._shunt.setSingleStep(0.001)
        self._shunt.setValue(self._cfg.shunt_ohm)
        self._shunt.setSuffix(" \u03A9")
        self._shunt.valueChanged.connect(lambda *_: self.configChanged.emit())
        isf.addRow("Shunt", self._shunt)
        root.addWidget(is_box)

        # --- Motor identity ---
        m_box = QGroupBox("Motor")
        mf = QFormLayout(m_box)
        self._pole_pairs = QSpinBox()
        self._pole_pairs.setRange(1, 64)
        self._pole_pairs.setValue(self._cfg.pole_pairs)
        self._pole_pairs.valueChanged.connect(lambda *_: self.configChanged.emit())
        mf.addRow("Pole pairs", self._pole_pairs)

        self._profile = QComboBox()
        for p in _list_motor_profiles():
            self._profile.addItem(p, p)
        idx = self._profile.findData(self._cfg.motor_profile)
        if idx >= 0:
            self._profile.setCurrentIndex(idx)
        self._profile.currentTextChanged.connect(
            lambda *_: self.configChanged.emit())
        mf.addRow("Profile", self._profile)
        root.addWidget(m_box)

        root.addStretch(1)
        self._on_pwm_type_changed()  # set 6-PWM low-pin enable state

    # --- Sensor sub-forms ----------------------------------------------

    def _build_i2c_form(self) -> QWidget:
        w = QWidget()
        f = QFormLayout(w)
        self._sda = QComboBox()
        self._scl = QComboBox()
        self._i2c_port = QSpinBox()
        self._i2c_port.setRange(0, 1)
        self._i2c_port.setValue(self._cfg.sensor_cfg.get("i2c_port", 0))
        for c, key in ((self._sda, "sda"), (self._scl, "scl")):
            _populate_pin_combo(c, self._cfg.target,
                                self._cfg.sensor_cfg.get(key))
            c.currentIndexChanged.connect(lambda *_: self.configChanged.emit())
        self._i2c_port.valueChanged.connect(lambda *_: self.configChanged.emit())
        f.addRow("SDA", self._sda)
        f.addRow("SCL", self._scl)
        f.addRow("I2C port", self._i2c_port)
        return w

    def _build_spi_form(self) -> QWidget:
        w = QWidget()
        f = QFormLayout(w)
        self._spi_mosi = QComboBox()
        self._spi_miso = QComboBox()
        self._spi_clk = QComboBox()
        self._spi_cs = QComboBox()
        self._spi_host = QSpinBox()
        self._spi_host.setRange(1, 3)
        self._spi_host.setValue(self._cfg.sensor_cfg.get("spi_host", 2))
        for c, key in ((self._spi_mosi, "mosi"),
                       (self._spi_miso, "miso"),
                       (self._spi_clk,  "clk"),
                       (self._spi_cs,   "cs")):
            _populate_pin_combo(c, self._cfg.target,
                                self._cfg.sensor_cfg.get(key))
            c.currentIndexChanged.connect(lambda *_: self.configChanged.emit())
        self._spi_host.valueChanged.connect(lambda *_: self.configChanged.emit())
        f.addRow("MOSI", self._spi_mosi)
        f.addRow("MISO", self._spi_miso)
        f.addRow("CLK",  self._spi_clk)
        f.addRow("CS",   self._spi_cs)
        f.addRow("SPI host", self._spi_host)
        return w

    def _build_pcnt_form(self) -> QWidget:
        w = QWidget()
        f = QFormLayout(w)
        self._pcnt_a = QComboBox()
        self._pcnt_b = QComboBox()
        self._cpr = QSpinBox()
        self._cpr.setRange(16, 100000)
        self._cpr.setValue(self._cfg.sensor_cfg.get("counts_per_rev", 4096))
        for c, key in ((self._pcnt_a, "pcnt_a"), (self._pcnt_b, "pcnt_b")):
            _populate_pin_combo(c, self._cfg.target,
                                self._cfg.sensor_cfg.get(key))
            c.currentIndexChanged.connect(lambda *_: self.configChanged.emit())
        self._cpr.valueChanged.connect(lambda *_: self.configChanged.emit())
        f.addRow("A pin", self._pcnt_a)
        f.addRow("B pin", self._pcnt_b)
        f.addRow("Counts per rev", self._cpr)
        return w

    # --- Reactivity ----------------------------------------------------

    def _on_target_changed(self, *_) -> None:
        target = self._target_combo.currentData()
        for c in (self._pin_u_hi, self._pin_v_hi, self._pin_w_hi,
                  self._pin_u_lo, self._pin_v_lo, self._pin_w_lo,
                  self._sda, self._scl,
                  self._spi_mosi, self._spi_miso, self._spi_clk, self._spi_cs,
                  self._pcnt_a, self._pcnt_b):
            _populate_pin_combo(c, target)
        for c in (self._adc_ch_u, self._adc_ch_v, self._adc_ch_w):
            _populate_adc_combo(c, target)
        self.configChanged.emit()

    def _on_pwm_type_changed(self, *_) -> None:
        is_6pwm = (self._pwm_type.currentData() == "6pwm_mcpwm")
        for c in (self._pin_u_lo, self._pin_v_lo, self._pin_w_lo):
            c.setEnabled(is_6pwm)
        self.configChanged.emit()

    def _on_sensor_changed(self, *_) -> None:
        self._sensor_stack.setCurrentIndex(self._sensor_type.currentIndex())
        self.configChanged.emit()

    # --- Public --------------------------------------------------------

    def get_config(self) -> HardwareConfig:
        cfg = HardwareConfig(
            target=self._target_combo.currentData(),
            pwm_type=self._pwm_type.currentData(),
            pwm_freq_hz=self._pwm_freq.value(),
            dc_link_v=self._dc_link.value(),
            pin_u_hi=self._pin_u_hi.currentData(),
            pin_v_hi=self._pin_v_hi.currentData(),
            pin_w_hi=self._pin_w_hi.currentData(),
            pin_u_lo=self._pin_u_lo.currentData(),
            pin_v_lo=self._pin_v_lo.currentData(),
            pin_w_lo=self._pin_w_lo.currentData(),
            sensor_type=self._sensor_type.currentData(),
            isensor_mode=self._isensor_mode.currentData(),
            adc_unit=self._adc_unit.value(),
            adc_ch_u=self._adc_ch_u.currentData(),
            adc_ch_v=self._adc_ch_v.currentData(),
            adc_ch_w=self._adc_ch_w.currentData(),
            amp_gain=self._amp_gain.value(),
            shunt_ohm=self._shunt.value(),
            pole_pairs=self._pole_pairs.value(),
            motor_profile=self._profile.currentData(),
        )
        cfg.sensor_cfg = {
            "sda": self._sda.currentData(),
            "scl": self._scl.currentData(),
            "i2c_port": self._i2c_port.value(),
            "mosi": self._spi_mosi.currentData(),
            "miso": self._spi_miso.currentData(),
            "clk":  self._spi_clk.currentData(),
            "cs":   self._spi_cs.currentData(),
            "spi_host": self._spi_host.value(),
            "pcnt_a": self._pcnt_a.currentData(),
            "pcnt_b": self._pcnt_b.currentData(),
            "counts_per_rev": self._cpr.value(),
        }
        return cfg
