"""Shared reader for the espFoC link protocol (protocol v2)."""

from __future__ import annotations

import struct
import threading
import time
from queue import Empty, Queue
from typing import Callable, Dict, Optional

from .codec import Channel, Decoder, Status, encode
from .transport import Transport


ScopeCallback = Callable[[int, int, bytes], None]
HeartbeatCallback = Callable[[int, int, int], None]  # counter, state, last_err
LinkLostCallback = Callable[[], None]

HB_MSG_FW = 0x01
HB_MSG_ACK = 0x02
HEARTBEAT_PERIOD_S = 0.5
HEARTBEAT_MISS_MAX = 2


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
        self._hb_cb: Optional[HeartbeatCallback] = None
        self._link_lost_cb: Optional[LinkLostCallback] = None

        self._watch_heartbeat = False
        self._last_hb_mono: float = 0.0
        self._pending_scope: list[tuple[int, int, bytes]] = []

    def set_heartbeat_watch(self, enabled: bool) -> None:
        with self._lock:
            self._watch_heartbeat = enabled
            if enabled:
                self._last_hb_mono = time.monotonic()
            else:
                self._last_hb_mono = 0.0

    def register_heartbeat_callback(self, cb: Optional[HeartbeatCallback]) -> None:
        with self._lock:
            self._hb_cb = cb

    def register_link_lost_callback(self, cb: Optional[LinkLostCallback]) -> None:
        with self._lock:
            self._link_lost_cb = cb

    @property
    def is_running(self) -> bool:
        t = self._thread
        return t is not None and t.is_alive()

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop_flag.clear()
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="espfoc-link-reader")
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
        with self._lock:
            for q in self._tuner_waiters.values():
                try:
                    q.put_nowait(None)
                except Exception:
                    pass

    def register_scope_callback(self, cb: ScopeCallback) -> None:
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

    def register_tuner_waiter(self, seq: int) -> Queue:
        q: Queue = Queue(maxsize=1)
        with self._lock:
            self._tuner_waiters[seq & 0xFF] = q
        return q

    def unregister_tuner_waiter(self, seq: int) -> None:
        with self._lock:
            self._tuner_waiters.pop(seq & 0xFF, None)

    def _check_heartbeat_timeout(self) -> None:
        with self._lock:
            if not self._watch_heartbeat or self._last_hb_mono <= 0.0:
                return
            elapsed = time.monotonic() - self._last_hb_mono
            if elapsed <= HEARTBEAT_PERIOD_S * HEARTBEAT_MISS_MAX:
                return
            self._watch_heartbeat = False
            cb = self._link_lost_cb
        if cb is not None:
            try:
                cb()
            except Exception:
                pass
        try:
            self.transport.close()
        except Exception:
            pass

    def _send_hb_ack(self, counter: int) -> None:
        body = bytes([HB_MSG_ACK, counter & 0xFF])
        try:
            self.transport.send_bytes(encode(Channel.HEARTBEAT, 0, body))
        except Exception:
            pass

    def _run(self) -> None:
        while not self._stop_flag.is_set():
            self._check_heartbeat_timeout()
            try:
                data = self.transport.read_bytes(128, timeout=0.1)
            except Exception:
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
        if channel == int(Channel.HEARTBEAT):
            if len(payload) >= 4 and payload[0] == HB_MSG_FW:
                counter = payload[1]
                state = payload[2]
                last_err = struct.unpack("<b", payload[3:4])[0]
                with self._lock:
                    self._last_hb_mono = time.monotonic()
                    hb_cb = self._hb_cb
                self._send_hb_ack(counter)
                if hb_cb is not None:
                    try:
                        hb_cb(counter, state, last_err)
                    except Exception:
                        pass
            return

        if channel == int(Channel.TUNER):
            with self._lock:
                q = self._tuner_waiters.get(seq)
            if q is not None:
                try:
                    q.put_nowait(payload)
                except Exception:
                    pass
            self._flush_pending_scope()
            return

        if channel == int(Channel.SCOPE):
            with self._lock:
                has_tuner_wait = bool(self._tuner_waiters)
            if has_tuner_wait:
                self._pending_scope.append((channel, seq, payload))
                if len(self._pending_scope) > 64:
                    self._pending_scope.pop(0)
                return
            self._deliver_scope(channel, seq, payload)

    def _flush_pending_scope(self) -> None:
        pending: list[tuple[int, int, bytes]]
        with self._lock:
            if not self._tuner_waiters:
                pending = self._pending_scope
                self._pending_scope = []
            else:
                return
        for ch, sq, pl in pending:
            self._deliver_scope(ch, sq, pl)

    def _deliver_scope(self, channel: int, seq: int, payload: bytes) -> None:
        with self._lock:
            cbs = list(self._scope_cbs)
        for cb in cbs:
            try:
                cb(channel, seq, payload)
            except Exception:
                pass
