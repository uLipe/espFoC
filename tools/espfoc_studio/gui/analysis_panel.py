"""Analysis panel: step response, Bode, pole-zero map, root locus."""

from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QGridLayout, QVBoxLayout, QWidget

from ..model import (
    MotorParams,
    PiGains,
    bode,
    pole_zero_map,
    root_locus,
    step_response,
)
from .crosshair import attach_crosshair


class AnalysisPanel(QWidget):
    """Four plots driven off (R, L, bandwidth, Kp, Ki) emitted by the
    tuning panel. Updates are cheap — all computations are O(thousands)
    in numpy so they complete within a single event-loop tick."""

    def __init__(self) -> None:
        super().__init__()
        pg.setConfigOptions(antialias=True)

        root = QVBoxLayout(self)
        grid = QGridLayout()
        root.addLayout(grid, 1)

        self._step_plot = pg.PlotWidget(title="Predicted step response")
        self._step_plot.setLabel('left', "i_q [A]")
        self._step_plot.setLabel('bottom', "time", units='s')
        self._step_plot.showGrid(x=True, y=True, alpha=0.3)
        self._step_curve = self._step_plot.plot(pen=pg.mkPen('#2196f3', width=2))
        self._step_ref = self._step_plot.plot(pen=pg.mkPen('#888', style=Qt.DashLine))
        self._step_crosshair = attach_crosshair(
            self._step_plot,
            fmt=lambda x, y: f"t = {x*1000:.2f} ms\ni = {y:+.4f} A")

        self._bode_mag = pg.PlotWidget(title="Open-loop gain |L(jω)|")
        self._bode_mag.setLabel('left', "Magnitude", units='dB')
        self._bode_mag.setLabel('bottom', "frequency", units='Hz')
        self._bode_mag.setLogMode(x=True, y=False)
        self._bode_mag.showGrid(x=True, y=True, alpha=0.3)
        self._bode_mag_curve = self._bode_mag.plot(pen=pg.mkPen('#00897b', width=2))
        self._bode_crosshair = attach_crosshair(
            self._bode_mag,
            fmt=lambda x, y: f"f = {10.0**x:.1f} Hz\n|L| = {y:+.2f} dB")

        self._pz_plot = pg.PlotWidget(title="Pole / zero map")
        self._pz_plot.setAspectLocked(True)
        self._pz_plot.setLabel('left', "Im(z)")
        self._pz_plot.setLabel('bottom', "Re(z)")
        self._pz_plot.showGrid(x=True, y=True, alpha=0.3)
        self._pz_crosshair = attach_crosshair(
            self._pz_plot,
            fmt=lambda x, y: f"Re = {x:+.3f}\nIm = {y:+.3f}")
        # Unit circle overlay.
        theta = np.linspace(0, 2 * np.pi, 361)
        self._pz_plot.plot(np.cos(theta), np.sin(theta),
                           pen=pg.mkPen('#bdbdbd', width=1))
        self._pz_ol_pole_item = pg.ScatterPlotItem(
            size=14, symbol='x', pen=pg.mkPen('#d81b60'), brush=None)
        self._pz_ol_zero_item = pg.ScatterPlotItem(
            size=12, symbol='o', pen=pg.mkPen('#1e88e5'), brush=None)
        self._pz_cl_pole_item = pg.ScatterPlotItem(
            size=14, symbol='x', pen=pg.mkPen('#000000'),
            brush=None)
        self._pz_plot.addItem(self._pz_ol_pole_item)
        self._pz_plot.addItem(self._pz_ol_zero_item)
        self._pz_plot.addItem(self._pz_cl_pole_item)

        self._rl_plot = pg.PlotWidget(title="Root locus (Kp scale)")
        self._rl_plot.setAspectLocked(True)
        self._rl_plot.setLabel('left', "Im(z)")
        self._rl_plot.setLabel('bottom', "Re(z)")
        self._rl_plot.showGrid(x=True, y=True, alpha=0.3)
        self._rl_crosshair = attach_crosshair(
            self._rl_plot,
            fmt=lambda x, y: f"Re = {x:+.3f}\nIm = {y:+.3f}")
        self._rl_plot.plot(np.cos(theta), np.sin(theta),
                           pen=pg.mkPen('#bdbdbd', width=1))
        self._rl_points = pg.ScatterPlotItem(size=6, pen=None,
                                             brush=pg.mkBrush('#e65100'))
        self._rl_plot.addItem(self._rl_points)

        grid.addWidget(self._step_plot, 0, 0)
        grid.addWidget(self._bode_mag,  0, 1)
        grid.addWidget(self._pz_plot,   1, 0)
        grid.addWidget(self._rl_plot,   1, 1)

    def update_model(self, motor_r: float, motor_l: float, bw_hz: float,
                     kp: float, ki: float) -> None:
        """Re-render everything for the given operating point."""
        try:
            motor = MotorParams(r_ohm=motor_r, l_h=motor_l,
                                ts_s=self._ts_s(), v_max=12.0)
            gains = PiGains(kp=kp, ki=ki, int_lim=motor.v_max)
        except ValueError:
            return

        # Step response.
        try:
            t, i = step_response(motor, gains, n_samples=400)
            self._step_curve.setData(t, i)
            self._step_ref.setData(t, np.ones_like(t))
        except Exception:  # numerical blowup
            pass

        # Bode.
        try:
            f, mag, _phase = bode(motor, gains)
            self._bode_mag_curve.setData(f, mag)
        except Exception:
            pass

        # Pole / zero.
        try:
            ol_p, ol_z, cl_p, _cl_z = pole_zero_map(motor, gains)
            self._pz_ol_pole_item.setData(pos=np.column_stack(
                (ol_p.real, ol_p.imag)))
            self._pz_ol_zero_item.setData(pos=np.column_stack(
                (ol_z.real, ol_z.imag)))
            self._pz_cl_pole_item.setData(pos=np.column_stack(
                (cl_p.real, cl_p.imag)))
        except Exception:
            pass

        # Root locus.
        try:
            _ks, mat = root_locus(motor, gains, scale_min=0.1,
                                  scale_max=2.0, n_points=80)
            flat = mat.flatten()
            valid = ~np.isnan(flat.real)
            self._rl_points.setData(pos=np.column_stack(
                (flat[valid].real, flat[valid].imag)))
        except Exception:
            pass

    def set_loop_rate_hz(self, fs_hz: float) -> None:
        """Override the assumed loop sample rate. Called by the
        MainWindow after reading PARAM_LOOP_FS_HZ from the firmware
        on connect, so the discrete plant the analysis tab simulates
        (step response, Bode, root locus) matches the rate the actual
        current PI is firing at — Plan #2's ISR hot path runs at
        40 kHz, the legacy task path runs at ~2 kHz, the predicted
        response is wildly different between the two."""
        if fs_hz > 1.0:
            self._loop_fs_hz = fs_hz

    def _ts_s(self) -> float:
        """Sample period the analysis math discretises against. Set
        from the firmware via set_loop_rate_hz() at connect; falls
        back to 1 ms if the firmware never reported one."""
        fs = getattr(self, "_loop_fs_hz", None) or 1000.0
        return 1.0 / fs
