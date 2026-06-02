"""Fixed-size per-channel sample rings (decode thread → UI thread)."""

from __future__ import annotations

import threading
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple


class ChannelRing:
    __slots__ = ("_t", "_y", "_cap")

    def __init__(self, capacity: int = 50_000) -> None:
        self._cap = max(1024, capacity)
        self._t: Deque[float] = deque(maxlen=self._cap)
        self._y: Deque[float] = deque(maxlen=self._cap)

    def append(self, t_s: float, value: float) -> None:
        self._t.append(t_s)
        self._y.append(value)

    def snapshot(self) -> Tuple[List[float], List[float]]:
        return list(self._t), list(self._y)

    def __len__(self) -> int:
        return len(self._t)


class ChannelStore:
    def __init__(self, capacity: int = 50_000) -> None:
        self._lock = threading.Lock()
        self._rings: Dict[int, ChannelRing] = {}
        self._capacity = capacity
        self._frames = 0
        self._last_seq: Optional[int] = None
        self._seq_gaps = 0

    def note_frame(self, seq: int, t_s: float, samples: tuple[int, ...],
                   indices: List[int]) -> None:
        with self._lock:
            if self._last_seq is not None:
                delta = (seq - self._last_seq) & 0xFFFF
                if delta != 1:
                    self._seq_gaps += 1
            self._last_seq = seq
            self._frames += 1
            for i in indices:
                if i >= len(samples):
                    continue
                ring = self._rings.get(i)
                if ring is None:
                    ring = ChannelRing(self._capacity)
                    self._rings[i] = ring
                ring.append(t_s, float(samples[i]))

    def active_indices(self) -> List[int]:
        with self._lock:
            return sorted(self._rings.keys())

    def snapshot(self, index: int) -> Tuple[List[float], List[float]]:
        with self._lock:
            ring = self._rings.get(index)
            if ring is None:
                return [], []
            return ring.snapshot()

    def stats(self) -> Tuple[int, int]:
        with self._lock:
            return self._frames, self._seq_gaps
