#!/usr/bin/env python3
"""Plot downsampling tests."""

from __future__ import annotations

import numpy as np

from espfoc_tool.gui.plot_util import (
    decimate_minmax_xy,
    fixed_time_axis,
    plot_window_seconds,
    target_display_points,
)


def test_minmax_keeps_peaks():
    x = np.linspace(0.0, 1.0, 2000)
    y = np.sin(2.0 * np.pi * 10.0 * x)
    xd, yd = decimate_minmax_xy(x, y, 80)
    assert xd.shape[0] <= 80
    assert yd.max() > 0.9
    assert yd.min() < -0.9


def test_minmax_preserves_time_order():
    x = np.arange(100, dtype=np.float64)
    y = np.random.default_rng(0).random(100)
    xd, yd = decimate_minmax_xy(x, y, 40)
    assert np.all(np.diff(xd) >= 0.0)


def test_fixed_time_axis_before_window_full():
    t_min, t_max = fixed_time_axis(0.5, 4.8)
    assert t_min == 0.0
    assert t_max == 4.8


def test_fixed_time_axis_rolling():
    t_min, t_max = fixed_time_axis(10.0, 4.8)
    assert abs(t_max - 10.0) < 1e-9
    assert abs(t_min - (10.0 - 4.8)) < 1e-9


def test_plot_window_from_settings():
    assert abs(plot_window_seconds(100, 480) - 4.8) < 1e-9


def test_target_points_capped():
    assert target_display_points(4.8, 100, 480) == 480
    assert target_display_points(4.8, 100, 200) == 200
