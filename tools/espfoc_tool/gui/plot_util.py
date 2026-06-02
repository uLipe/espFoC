"""Display downsampling for pyqtgraph."""

from __future__ import annotations

import numpy as np


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
