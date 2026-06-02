"""Display downsampling for scope traces."""

from __future__ import annotations

import numpy as np

PLOT_POINTS_ABSOLUTE_MAX = 2048
PLOT_SAMPLE_RATE_MIN_HZ = 1
PLOT_SAMPLE_RATE_MAX_HZ = 1000
PLOT_POINTS_MIN = 10


def clamp_plot_settings(sample_rate_hz: int, max_points: int) -> tuple[int, int]:
    sr = max(PLOT_SAMPLE_RATE_MIN_HZ,
             min(PLOT_SAMPLE_RATE_MAX_HZ, int(sample_rate_hz)))
    mp = max(PLOT_POINTS_MIN, min(PLOT_POINTS_ABSOLUTE_MAX, int(max_points)))
    return sr, mp


def plot_window_seconds(sample_rate_hz: int, max_points: int) -> float:
    sr, mp = clamp_plot_settings(sample_rate_hz, max_points)
    return mp / sr


def fixed_time_axis(t_max: float, window_s: float) -> tuple[float, float]:
    """Fixed horizontal span — trace grows in from the right, no early compression."""
    if window_s <= 0.0:
        return t_max, t_max + 1.0
    if t_max <= window_s:
        return 0.0, window_s
    return t_max - window_s, t_max


def target_display_points(window_s: float, sample_rate_hz: int, max_points: int) -> int:
    by_rate = max(PLOT_POINTS_MIN, int(window_s * sample_rate_hz) + 1)
    return min(max_points, by_rate)


def decimate_xy(x: np.ndarray, y: np.ndarray, max_points: int) -> tuple[np.ndarray, np.ndarray]:
    n = int(x.shape[0])
    if n <= max_points or max_points < 2:
        return x, y
    idx = np.unique(
        np.clip(
            np.round(np.linspace(0, n - 1, max_points)).astype(np.int64),
            0,
            n - 1,
        ))
    return x[idx], y[idx]


def decimate_minmax_xy(x: np.ndarray, y: np.ndarray,
                       max_points: int) -> tuple[np.ndarray, np.ndarray]:
    """Bucket min/max decimation — keeps peaks visible at low point counts."""
    n = int(x.shape[0])
    if n <= max_points or max_points < 4:
        return x, y

    n_buckets = max(1, max_points // 2)
    bucket_size = n / n_buckets
    out_x: list[float] = []
    out_y: list[float] = []

    for b in range(n_buckets):
        i0 = int(b * bucket_size)
        i1 = n if b == n_buckets - 1 else int((b + 1) * bucket_size)
        if i1 <= i0:
            continue
        chunk = y[i0:i1]
        i_min = i0 + int(np.argmin(chunk))
        i_max = i0 + int(np.argmax(chunk))
        if i_min <= i_max:
            for idx in (i_min, i_max):
                out_x.append(float(x[idx]))
                out_y.append(float(y[idx]))
        else:
            for idx in (i_max, i_min):
                out_x.append(float(x[idx]))
                out_y.append(float(y[idx]))

    return np.asarray(out_x, dtype=np.float64), np.asarray(out_y, dtype=np.float64)
