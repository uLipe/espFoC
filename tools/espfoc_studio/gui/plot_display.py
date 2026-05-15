"""Display-path downsampling for pyqtgraph. Full-rate data stays in panel
buffers; only arrays passed to PlotDataItem.setData are reduced here."""

from __future__ import annotations

from typing import List, Literal

import numpy as np
import pyqtgraph as pg

Method = Literal["uniform", "peak"]


def configure_dynamic_curve(curve: pg.PlotDataItem) -> None:
    curve.setClipToView(True)


def configure_rolling_time_xaxis(plot: pg.PlotWidget) -> None:
    """Drop ViewBox default padding so traces use the full plot width."""
    plot.getPlotItem().getViewBox().setDefaultPadding(padding=0.0)


def rolling_plot_x_upper(x: np.ndarray, cap_s: float) -> float:
    """Match X extent to the plotted time span, capped at *cap_s* (rolling window)."""
    cap = float(cap_s)
    if cap <= 0.0:
        return 1e-6
    if x.size < 2:
        return cap
    span = float(x[-1] - x[0])
    if span <= 0.0:
        return cap
    return min(span, cap)


def _uniform_index_count(n: int, max_points: int) -> np.ndarray:
    return np.unique(
        np.clip(
            np.round(np.linspace(0, n - 1, max_points)).astype(np.int64),
            0,
            n - 1,
        ))


def decimate_xy_for_display(
    x: np.ndarray,
    y: np.ndarray,
    max_points: int,
    *,
    method: Method = "peak",
) -> tuple[np.ndarray, np.ndarray]:
    n = int(x.shape[0])
    if n == 0:
        return x, y
    mp = int(max_points)
    if mp < 2 or n <= mp:
        return x, y
    if method == "uniform":
        idx = _uniform_index_count(n, mp)
    else:
        idx = _peak_indices_single(y, n, mp)
    return x[idx], y[idx]


def _peak_indices_single(y: np.ndarray, n: int, max_points: int) -> np.ndarray:
    n_bins = max(1, max_points // 2)
    edges = np.linspace(0, n, n_bins + 1)
    idx_set: set[int] = {0, n - 1}
    for bi in range(n_bins):
        lo = int(edges[bi])
        hi = min(n, int(edges[bi + 1]))
        if hi <= lo:
            continue
        sl = y[lo:hi]
        i0 = lo + int(np.argmin(sl))
        i1 = lo + int(np.argmax(sl))
        idx_set.add(i0)
        idx_set.add(i1)
    idx = np.sort(np.fromiter(idx_set, dtype=np.int64))
    if idx.size <= max_points:
        return idx
    sub = _uniform_index_count(idx.size, max_points)
    return idx[sub]


def decimation_indices_peak_union(
    y_arrays: List[np.ndarray],
    max_points: int,
) -> np.ndarray:
    """Shared index list so overlaid channels stay time-aligned."""
    if not y_arrays:
        return np.array([], dtype=np.int64)
    n = int(y_arrays[0].shape[0])
    mp = int(max_points)
    if n == 0:
        return np.array([], dtype=np.int64)
    if mp < 2 or n <= mp:
        return np.arange(n, dtype=np.int64)
    for y in y_arrays:
        if int(y.shape[0]) != n:
            raise ValueError("decimation_indices_peak_union: length mismatch")
    n_bins = max(1, mp // 2)
    edges = np.linspace(0, n, n_bins + 1)
    idx_set: set[int] = {0, n - 1}
    for bi in range(n_bins):
        lo = int(edges[bi])
        hi = min(n, int(edges[bi + 1]))
        if hi <= lo:
            continue
        for y in y_arrays:
            sl = y[lo:hi]
            i0 = lo + int(np.argmin(sl))
            i1 = lo + int(np.argmax(sl))
            idx_set.add(i0)
            idx_set.add(i1)
    idx = np.sort(np.fromiter(idx_set, dtype=np.int64))
    if idx.size <= mp:
        return idx
    sub = _uniform_index_count(idx.size, mp)
    return idx[sub]
