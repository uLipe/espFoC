"""Synchronous tuner protocol client.

Wraps a Transport in the link codec and the application-level
[op][id][axis][payload] envelope. Built around a single round-trip per
call: send request, decode bytes until a tuner-channel response with the
same seq comes back. The reactor on the firmware side is also synchronous,
so this matches it without needing async io on the host.

Q16.16 helpers convert between Python floats and the 32-bit signed
fixed-point format used everywhere in the firmware.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum, IntFlag
import threading
from queue import Empty
from typing import Optional, Union

from ..link import (
    Channel,
    LinkReader,
    Transport,
    encode,
)


def _tuner_error_from_link_io(err: BaseException) -> TunerError:
    return TunerError(f"link I/O: {err}")


def _is_transport_io_failure(err: BaseException) -> bool:
    if isinstance(err, (OSError, ConnectionError, BrokenPipeError)):
        return True
    if isinstance(err, TypeError):
        return True
    ex_mod = getattr(type(err), "__module__", "") or ""
    return ex_mod.startswith("serial")


class Op(IntEnum):
    READ = 0x01
    WRITE = 0x02
    EXEC = 0x03


class ParamId(IntEnum):
    KP_Q16          = 0x0010
    KI_Q16          = 0x0011
    INT_LIM_Q16     = 0x0012
    V_MAX_Q16       = 0x0013
    I_FILTER_FC     = 0x0014
    LOOP_FS_HZ      = 0x0015
    KD_Q16          = 0x0016
    KFF_Q16         = 0x0017
    MOTOR_POLE_PAIRS = 0x0018
    AXIS_STATE      = 0x0040
    AXIS_LAST_ERR   = 0x0041
    NVS_PRESENT     = 0x0042
    FIRMWARE_TYPE   = 0x0050
    WRITE_KP        = 0x0020
    WRITE_KI        = 0x0021
    WRITE_INT_LIM   = 0x0022
    WRITE_I_FILTER_FC = 0x0023
    WRITE_MOTOR_POLE_PAIRS = 0x0024
    WRITE_KD        = 0x0025
    WRITE_KFF       = 0x0026
    WRITE_TARGET_ID = 0x0060
    WRITE_TARGET_IQ = 0x0061
    CMD_CONNECT         = 0x00A0
    CMD_DISCONNECT      = 0x00A1
    CMD_ALIGN_AXIS      = 0x00A2
    CMD_STOP_AXIS       = 0x00A3
    CMD_RUN_AXIS        = 0x00A4
    CMD_STORE_NVS       = 0x00B0
    CMD_ERASE_NVS       = 0x00B2
    CMD_RESET_BOARD     = 0x00B3
    CMD_SCOPE_START     = 0x00C0
    CMD_SCOPE_STOP      = 0x00C1

LINK_PROTO_VER = 2
HB_MSG_FW = 0x01
HB_MSG_ACK = 0x02
TUNER_FIRMWARE_TYPE_TSGX = 0x58475354


class AxisStateFlag(IntFlag):
    INITIALIZED    = 1 << 0
    ALIGNED        = 1 << 1
    RUNNING        = 1 << 2
    ALIGNING       = 1 << 4


# Mirror of esp_foc_err_t. Negative values indicate failure.
ESP_FOC_OK = 0
ESP_FOC_ERR_NOT_ALIGNED          = -1
ESP_FOC_ERR_INVALID_ARG          = -2
ESP_FOC_ERR_AXIS_INVALID_STATE   = -3
ESP_FOC_ERR_ALIGNMENT_IN_PROGRESS = -4
ESP_FOC_ERR_TIMESTEP_TOO_SMALL   = -5
ESP_FOC_ERR_ROTOR_STARTUP        = -6
ESP_FOC_ERR_ROTOR_STARTUP_PI     = -7


_ERR_NAMES = {
    ESP_FOC_OK: "OK",
    ESP_FOC_ERR_NOT_ALIGNED: "NOT_ALIGNED",
    ESP_FOC_ERR_INVALID_ARG: "INVALID_ARG",
    ESP_FOC_ERR_AXIS_INVALID_STATE: "AXIS_INVALID_STATE",
    ESP_FOC_ERR_ALIGNMENT_IN_PROGRESS: "ALIGNMENT_IN_PROGRESS",
    ESP_FOC_ERR_TIMESTEP_TOO_SMALL: "TIMESTEP_TOO_SMALL",
    ESP_FOC_ERR_ROTOR_STARTUP: "ROTOR_STARTUP",
    ESP_FOC_ERR_ROTOR_STARTUP_PI: "ROTOR_STARTUP_PI",
}


def _err_name(code: int) -> str:
    return _ERR_NAMES.get(code, f"errno {code}")


class TunerError(Exception):
    """Raised when the firmware returns a non-OK status, the response
    times out, or the link layer fails to parse a frame."""


@dataclass
class TunerResponse:
    status: int
    payload: bytes


@dataclass
class ConnectInfo:
    firmware_type: int
    num_axes: int
    scope_channels: int
    heartbeat_period_ms: int


def q16_from_float(x: float) -> int:
    v = round(x * 65536.0)
    if v > 0x7FFFFFFF:
        v = 0x7FFFFFFF
    if v < -0x80000000:
        v = -0x80000000
    return int(v)


def q16_to_float(v: int) -> float:
    return v / 65536.0


class TunerClient:
    """Send tuner requests over a Transport and wait for matching responses.

    Under the hood every client shares a LinkReader with the rest of the
    app (scope panel, log viewer, ...). Each round-trip parks itself on
    a per-seq Queue, so concurrent scope traffic on the same bus can't
    starve a pending tuner response.

    Round-trips are mutex-serialized so concurrent callers (poll worker,
    alignment thread, …) never interleave ``send_bytes`` on the same
    serial/USB endpoint — pyserial is not thread-safe.

    Call sites that care about responsive UI should invoke reads/writes from
    a dedicated thread — **not** the Qt GUI thread — so the mutex never
    stalls painting (model/controller vs view).
    """

    DEFAULT_TIMEOUT = 0.5  # seconds per round-trip

    def __init__(self, transport_or_reader: Union[Transport, LinkReader],
                 axis: int = 0) -> None:
        self._axis = axis
        self._seq = 0
        self._connected = False
        self._connect_info: Optional[ConnectInfo] = None
        self._hb_state = 0
        self._hb_last_err = 0
        self._bus_lock = threading.Lock()
        if isinstance(transport_or_reader, LinkReader):
            self._reader = transport_or_reader
            self._owns_reader = False
        else:
            self._reader = LinkReader(transport_or_reader)
            self._reader.start()
            self._owns_reader = True
        self._reader.register_heartbeat_callback(self._on_heartbeat)
        self._reader.register_link_lost_callback(self._on_link_lost)

    @property
    def reader(self) -> LinkReader:
        return self._reader

    def replace_reader(self, new_reader: LinkReader) -> None:
        """Swap the active LinkReader after a transport failure / reconnect
        (same TunerClient instance; axis ID unchanged). The sequence
        counter resets so a fresh port session and firmware boot share a
        clear round-trip key space."""
        if new_reader is None:  # pragma: no cover - guard for callers
            raise ValueError("replace_reader: reader is None")
        self._reader = new_reader
        self._seq = 0
        self._connected = False
        self._reader.register_heartbeat_callback(self._on_heartbeat)
        self._reader.register_link_lost_callback(self._on_link_lost)

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def connect_info(self) -> Optional[ConnectInfo]:
        return self._connect_info

    @property
    def heartbeat_state(self) -> int:
        return self._hb_state

    @property
    def heartbeat_last_err(self) -> int:
        return self._hb_last_err

    def _on_heartbeat(self, _counter: int, state: int, last_err: int) -> None:
        self._hb_state = state
        self._hb_last_err = last_err

    def _on_link_lost(self) -> None:
        self._connected = False
        self._reader.set_heartbeat_watch(False)

    def close(self) -> None:
        if self._connected:
            try:
                self.disconnect()
            except TunerError:
                pass
        if self._owns_reader:
            self._reader.stop()

    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFF
        return self._seq

    def _round_trip(self, op: Op, id_: ParamId,
                    cmd_payload: bytes = b"",
                    timeout: Optional[float] = None,
                    require_session: bool = True) -> TunerResponse:
        if require_session and not self._connected:
            raise TunerError("not connected — run connect first")
        with self._bus_lock:
            seq = self._next_seq()
            app = bytes([
                int(op),
                int(id_) & 0xFF, (int(id_) >> 8) & 0xFF,
                self._axis & 0xFF,
            ]) + cmd_payload
            q = self._reader.register_tuner_waiter(seq)
            try:
                frame = encode(Channel.TUNER, seq, app)
                try:
                    self._reader.transport.send_bytes(frame)
                except Exception as e:
                    if _is_transport_io_failure(e):
                        raise _tuner_error_from_link_io(e) from e
                    raise
                try:
                    body = q.get(timeout=timeout if timeout is not None
                                 else self.DEFAULT_TIMEOUT)
                except Empty:
                    raise TunerError(f"timeout waiting for response (seq={seq})")
                if body is None:
                    raise TunerError("reader stopped")
                if len(body) < 2:
                    raise TunerError(
                        f"response too short ({len(body)} bytes)")
                status = struct.unpack("<b", body[:1])[0]
                return TunerResponse(status=status, payload=body[2:])
            finally:
                self._reader.unregister_tuner_waiter(seq)

    # --- High-level helpers -------------------------------------------------

    def _read_q16(self, id_: ParamId) -> float:
        r = self._round_trip(Op.READ, id_)
        if r.status != ESP_FOC_OK:
            raise TunerError(f"read {id_.name} failed: {_err_name(r.status)}")
        if len(r.payload) != 4:
            raise TunerError(f"read {id_.name}: expected 4 bytes, got {len(r.payload)}")
        return q16_to_float(struct.unpack("<i", r.payload)[0])

    def _write_q16(self, id_: ParamId, value_float: float) -> None:
        payload = struct.pack("<i", q16_from_float(value_float))
        r = self._round_trip(Op.WRITE, id_, payload)
        if r.status != ESP_FOC_OK:
            raise TunerError(
                f"write {id_.name}={value_float} failed: {_err_name(r.status)}")

    def _exec(self, id_: ParamId, payload: bytes = b"",
              timeout: Optional[float] = None,
              require_session: bool = True) -> TunerResponse:
        r = self._round_trip(Op.EXEC, id_, payload, timeout=timeout,
                             require_session=require_session)
        if r.status != ESP_FOC_OK:
            raise TunerError(f"exec {id_.name} failed: {_err_name(r.status)}")
        return r

    def connect(self) -> ConnectInfo:
        payload = struct.pack("<BH", LINK_PROTO_VER, 0)
        r = self._round_trip(
            Op.EXEC, ParamId.CMD_CONNECT, payload, require_session=False)
        if r.status != ESP_FOC_OK:
            raise TunerError(f"connect failed: {_err_name(r.status)}")
        if len(r.payload) < 9:
            raise TunerError(f"connect_ack too short ({len(r.payload)} bytes)")
        proto = r.payload[0]
        if proto != LINK_PROTO_VER:
            raise TunerError(f"unsupported proto_ver {proto}")
        fw = struct.unpack("<I", r.payload[1:5])[0]
        info = ConnectInfo(
            firmware_type=fw,
            num_axes=r.payload[5],
            scope_channels=r.payload[6],
            heartbeat_period_ms=struct.unpack("<H", r.payload[7:9])[0],
        )
        self._connect_info = info
        self._connected = True
        self._reader.set_heartbeat_watch(True)
        return info

    def disconnect(self) -> None:
        try:
            self._exec(ParamId.CMD_DISCONNECT, require_session=True)
        finally:
            self._connected = False
            self._reader.set_heartbeat_watch(False)

    # Public API ------------------------------------------------------------

    def read_kp(self) -> float:
        return self._read_q16(ParamId.KP_Q16)

    def read_ki(self) -> float:
        return self._read_q16(ParamId.KI_Q16)

    def read_int_lim(self) -> float:
        return self._read_q16(ParamId.INT_LIM_Q16)

    def read_kd(self) -> float:
        return self._read_q16(ParamId.KD_Q16)

    def read_kff(self) -> float:
        return self._read_q16(ParamId.KFF_Q16)

    def read_v_max(self) -> float:
        return self._read_q16(ParamId.V_MAX_Q16)

    def read_axis_state(self) -> AxisStateFlag:
        r = self._round_trip(Op.READ, ParamId.AXIS_STATE)
        if r.status != ESP_FOC_OK:
            raise TunerError(f"read axis state failed: {_err_name(r.status)}")
        if len(r.payload) != 1:
            raise TunerError(
                f"axis state response has {len(r.payload)} bytes, want 1")
        return AxisStateFlag(r.payload[0])

    def read_last_error(self) -> int:
        r = self._round_trip(Op.READ, ParamId.AXIS_LAST_ERR)
        if r.status != ESP_FOC_OK:
            raise TunerError(f"read last err failed: {_err_name(r.status)}")
        return struct.unpack("<b", r.payload[:1])[0]

    def write_kp(self, kp: float) -> None:
        self._write_q16(ParamId.WRITE_KP, kp)

    def write_ki(self, ki: float) -> None:
        self._write_q16(ParamId.WRITE_KI, ki)

    def read_current_filter_fc(self) -> float:
        """Cutoff (Hz) of the per-phase Butterworth in the isensor driver."""
        return self._read_q16(ParamId.I_FILTER_FC)

    def read_loop_fs_hz(self) -> float:
        """Sample rate (Hz) at which the current PI fires on the
        firmware. Equals the PWM rate under ISR_HOT_PATH and
        pwm_rate / decimation under the legacy task path. The host
        uses this to discretise the analysis-tab plant correctly."""
        return self._read_q16(ParamId.LOOP_FS_HZ)

    def read_motor_pole_pairs(self) -> int:
        r = self._round_trip(Op.READ, ParamId.MOTOR_POLE_PAIRS)
        if r.status != ESP_FOC_OK:
            raise TunerError(
                f"read motor_pole_pairs failed: {_err_name(r.status)}")
        if len(r.payload) != 4:
            raise TunerError(
                f"motor_pole_pairs response has {len(r.payload)} bytes, want 4")
        return struct.unpack("<i", r.payload[:4])[0]

    def write_current_filter_fc(self, fc_hz: float) -> None:
        """Re-design the per-phase biquad with the supplied cutoff (Hz).
        fs is fixed by the firmware to the loop rate captured at init."""
        self._write_q16(ParamId.WRITE_I_FILTER_FC, fc_hz)

    def write_int_lim(self, lim: float) -> None:
        self._write_q16(ParamId.WRITE_INT_LIM, lim)

    def write_kd(self, kd: float) -> None:
        self._write_q16(ParamId.WRITE_KD, kd)

    def write_kff(self, kff: float) -> None:
        self._write_q16(ParamId.WRITE_KFF, kff)

    def write_motor_pole_pairs(self, pole_pairs: int) -> None:
        if not (1 <= int(pole_pairs) <= 64):
            raise TunerError("motor pole pairs must be 1..64")
        p = struct.pack("<i", int(pole_pairs))
        r = self._round_trip(Op.WRITE, ParamId.WRITE_MOTOR_POLE_PAIRS, p)
        if r.status != ESP_FOC_OK:
            raise TunerError(
                f"write motor_pole_pairs failed: {_err_name(r.status)}")

    def write_target_id(self, id_amps: float) -> None:
        self._write_q16(ParamId.WRITE_TARGET_ID, id_amps)

    def write_target_iq(self, iq_amps: float) -> None:
        self._write_q16(ParamId.WRITE_TARGET_IQ, iq_amps)

    def read_firmware_type(self) -> int:
        r = self._round_trip(Op.READ, ParamId.FIRMWARE_TYPE)
        if r.status != ESP_FOC_OK:
            raise TunerError(
                f"read firmware_type failed: {_err_name(r.status)}")
        if len(r.payload) != 4:
            raise TunerError(
                f"firmware_type response has {len(r.payload)} bytes, want 4")
        return struct.unpack("<I", r.payload)[0]

    def is_calibration_present(self) -> bool:
        r = self._round_trip(Op.READ, ParamId.NVS_PRESENT)
        if r.status != ESP_FOC_OK:
            raise TunerError(
                f"read nvs_present failed: {_err_name(r.status)}")
        return bool(r.payload[0]) if r.payload else False

    def align_axis(self, timeout: float = 8.0) -> None:
        """Triggers blocking alignment on the firmware. Default timeout
        is generous: open-loop sweeps and re-park take several seconds."""
        self._exec(ParamId.CMD_ALIGN_AXIS, timeout=timeout)

    def stop_axis(self) -> None:
        self._exec(ParamId.CMD_STOP_AXIS)

    def run_axis(self) -> None:
        """Start the FOC loop after alignment."""
        self._exec(ParamId.CMD_RUN_AXIS)

    def store_calibration(self) -> None:
        """Write live tuning fields to NVS when they differ from stored."""
        self._exec(ParamId.CMD_STORE_NVS, b"")

    def erase_calibration(self) -> None:
        self._exec(ParamId.CMD_ERASE_NVS)

    def scope_start(self) -> None:
        self._exec(ParamId.CMD_SCOPE_START)

    def scope_stop(self) -> None:
        self._exec(ParamId.CMD_SCOPE_STOP)

    def reset_board(self, timeout: float = 1.2) -> None:
        """Ask firmware to `esp_restart()`. The link usually drops before
        the response returns; a timeout is normal and is treated as success.
        """
        try:
            self._exec(ParamId.CMD_RESET_BOARD, timeout=timeout)
        except TunerError as e:
            err = str(e).lower()
            if "timeout" in err or "link" in err or "response" in err:
                return
            raise
