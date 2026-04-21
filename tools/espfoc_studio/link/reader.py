"""Shared reader for the espFoC link protocol.

A single daemon thread consumes bytes from one Transport, feeds them
through the streaming decoder and demuxes complete frames by channel:

  * TUNER frames go to a per-seq Queue so TunerClient round-trips stay
    synchronous and match requests to responses deterministically;
  * SCOPE frames go to a user-registered callback (typically the GUI's
    scope panel);
  * LOG frames go to a user-registered callback (unused today, wired
    up for the future tooling).

The reader is designed to be shared: one physical bus → one reader →
many consumers. This is the only model that makes sense once the scope
stream starts flowing in parallel with tuner requests.
"""

from __future__ import annotations

import threading
from queue import Empty, Queue
from typing import Callable, Dict, Optional

from .codec import Channel, Decoder, Status
from .transport import Transport


ScopeCallback = Callable[[int, int, bytes], None]  # (channel, seq, payload)
LogCallback = Callable[[int, bytes], None]          # (seq, payload)


class LinkReader:
    """Background reader + demuxer for a single Transport."""

    def __init__(self, transport: Transport) -> None:
        self.transport = transport
        self._dec = Decoder()
        self._thread: Optional[threading.Thread] = None
        self._stop_flag = threading.Event()

        self._lock = threading.Lock()
        self._tuner_waiters: Dict[int, Queue] = {}
        self._scope_cbs: list[ScopeCallback] = []
        self._log_cb: Optional[LogCallback] = None

    # --- Lifecycle -------------------------------------------------------

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop_flag.clear()
        self._thread = threading.Thread(target=self._run,
                                        daemon=True,
                                        name="espfoc-link-reader")
        self._thread.start()

    def stop(self) -> None:
        self._stop_flag.set()
        try:
            self.transport.close()
        except Exception:
            pass
        if self._thread is not None:
            self._thread.join(timeout=0.5)
            self._thread = None
        # Wake any waiters so they raise a timeout instead of hanging.
        with self._lock:
            for q in self._tuner_waiters.values():
                try:
                    q.put_nowait(None)
                except Exception:
                    pass

    # --- Consumer registration ------------------------------------------

    def register_scope_callback(self, cb: ScopeCallback) -> None:
        """Append a scope handler; all subscribers are broadcast to."""
        if cb is None:
            return
        with self._lock:
            if cb not in self._scope_cbs:
                self._scope_cbs.append(cb)

    def unregister_scope_callback(self, cb: ScopeCallback) -> None:
        with self._lock:
            try:
                self._scope_cbs.remove(cb)
            except ValueError:
                pass

    def register_log_callback(self, cb: Optional[LogCallback]) -> None:
        with self._lock:
            self._log_cb = cb

    def register_tuner_waiter(self, seq: int) -> Queue:
        q: Queue = Queue(maxsize=1)
        with self._lock:
            self._tuner_waiters[seq & 0xFF] = q
        return q

    def unregister_tuner_waiter(self, seq: int) -> None:
        with self._lock:
            self._tuner_waiters.pop(seq & 0xFF, None)

    # --- Reader loop -----------------------------------------------------

    def _run(self) -> None:
        while not self._stop_flag.is_set():
            try:
                data = self.transport.read_bytes(128, timeout=0.1)
            except Exception:
                # transport closed under us; exit cleanly
                return
            if not data:
                continue
            for b in data:
                st = self._dec.push(b)
                if st == Status.OK:
                    channel = self._dec.channel
                    seq = self._dec.seq
                    payload = bytes(self._dec.payload)
                    self._dispatch(channel, seq, payload)
                    self._dec.reset()

    def _dispatch(self, channel: int, seq: int, payload: bytes) -> None:
        if channel == int(Channel.TUNER):
            with self._lock:
                q = self._tuner_waiters.get(seq)
            if q is not None:
                try:
                    q.put_nowait(payload)
                except Exception:
                    pass  # waiter already received something; drop duplicate
            return
        if channel == int(Channel.SCOPE):
            with self._lock:
                cbs = list(self._scope_cbs)
            for cb in cbs:
                try:
                    cb(channel, seq, payload)
                except Exception:
                    pass
            return
        if channel == int(Channel.LOG):
            with self._lock:
                cb = self._log_cb
            if cb is not None:
                try:
                    cb(seq, payload)
                except Exception:
                    pass
            return
