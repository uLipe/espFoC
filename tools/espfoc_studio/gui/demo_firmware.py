"""Embedded "firmware" for GUI demo mode.

Extends the protocol mirror used in tests/test_tuner_protocol.py with:
  * full coverage of every tuner request the GUI issues
  * a very simple plant simulation: first-order R-L model driven by the
    current PID using the last commanded iq reference
  * an MPZ recompute path that uses the authoritative espfoc_studio.model
    so the GUI sees exactly the same gains the real firmware would
    produce for the same motor parameters

The firmware runs in its own daemon thread and is paired to the GUI via
a LoopbackTransport. When the user drives a motion target through the
override flow, the plant simulation settles toward it following the
current closed-loop dynamics; this is what the scope panel plots.
"""

from __future__ import annotations

import struct
import threading
import time
from typing import Optional

from ..link import Channel, Decoder, LoopbackTransport, MAX_PAYLOAD, Status, encode
from ..link.scope_sample import pack_scope_i32_to_payload
from ..model import MotorParams, PiGains, mpz_design
from ..protocol import AxisStateFlag, Op, ParamId
from ..protocol.tuner import TUNER_FIRMWARE_TYPE_TSGX

# Mirror of ESP_FOC_* status codes.
OK = 0
ERR_NOT_ALIGNED = -1
ERR_INVALID_ARG = -2
ERR_AXIS_INVALID_STATE = -3


def _q16_to_bytes(v: float) -> bytes:
    raw = max(-0x80000000, min(0x7FFFFFFF, round(v * 65536.0)))
    return struct.pack("<i", raw)


def _q16_from_bytes(b: bytes) -> float:
    return struct.unpack("<i", b)[0] / 65536.0


class DemoFirmware(threading.Thread):
    """Fake firmware used by the GUI's --demo mode.

    Internally keeps the full tuner state (gains, axis state, override)
    plus a plant simulation that the scope panel can sample.
    """

    def __init__(self, transport: LoopbackTransport,
                 motor: Optional[MotorParams] = None) -> None:
        super().__init__(daemon=True, name="espfoc-demo-firmware")
        self._t = transport
        self._stop = threading.Event()
        self._dec = Decoder()
        self._state_lock = threading.Lock()

        # ts_s mirrors the ISR_HOT_PATH default loop period (40 kHz
         # PWM = 25 us). The demo runs SUB_STEPS=40 sub-iterations of
         # the synthetic plant per wall-clock millisecond so the scope
         # frame rate stays where the GUI expects it (~250 Hz, one
         # frame per 4 ms wallclock = SUB_STEPS * SCOPE_DECIMATION).
         # This keeps mpz_design(BW=4 kHz, fs=40 kHz) inside Nyquist
         # and matches what the host's analysis tab discretises against.
        self.motor = motor or MotorParams(r_ohm=1.08, l_h=0.0018,
                                          ts_s=25e-6, v_max=12.0)
        initial_gains = mpz_design(self.motor, bandwidth_hz=150.0)
        self.kp = initial_gains.kp
        self.ki = initial_gains.ki
        self.lim = initial_gains.int_lim
        self.aligned = True  # demo axis pretends it is ready to run
        self.override = False
        self.skip_torque = False
        self.target_id = 0.0
        self.target_iq = 0.0
        self.target_ud = 0.0
        self.target_uq = 0.0

        # Pretend to be a tuner_studio_target firmware so the GUI's
        # Generate App tab shows up in --demo mode.
        self.firmware_type = TUNER_FIRMWARE_TYPE_TSGX
        # Current-sense low-pass cutoff (matches the firmware Kconfig
        # default). The demo doesn't actually run a biquad; we just
        # round-trip the value through the protocol.
        self.current_filter_fc = 300.0
        self.motor_pole_pairs = 7
        # In-memory NVS overlay; populated by CMD_PERSIST_NVS, cleared
        # by CMD_ERASE_NVS, applied by CMD_LOAD_NVS.
        self.nvs_overlay: Optional[dict] = None

        # Plant state.
        self._i_actual = 0.0
        self._integ = 0.0
        self._integ_prev = 0.0
        self._sim_thread = threading.Thread(target=self._simulate,
                                            daemon=True,
                                            name="espfoc-demo-plant")

    # --- Public helpers used by the GUI ------------------------------------

    def snapshot_plant(self) -> tuple[float, float, float]:
        """Return (target_iq, i_actual, time_s) for the scope panel."""
        with self._state_lock:
            return (self.target_iq, self._i_actual, time.monotonic())

    # Configurable set of channels the demo streams on the SCOPE channel.
    # The firmware's esp_foc_scope.c emits SCOPE v1 binary; the demo
    # matches so the same decoder runs on the host.
    # Sub-iterations of the synthetic plant per wall-clock tick. With
    # ts_s = 25 us this gives a wall-clock budget of 1 ms per outer
    # loop iteration, matching the previous demo cadence so the scope
    # frame rate (one frame every SCOPE_DECIMATION outer iterations =
    # 4 ms = 250 Hz) stays where the rest of the GUI expects it.
    SUB_STEPS = 40
    SCOPE_DECIMATION = 4

    @staticmethod
    def _float_to_q16_i32(f: float) -> int:
        d = f * 65536.0
        vi = int(round(d))
        if vi > 0x7FFFFFFF:
            return 0x7FFFFFFF
        if vi < -0x80000000:
            return -0x80000000
        return vi

    def _scope_frame_payload(self) -> bytes:
        """SCOPE v1: same channel order as the real firmware (six channels
        in this demo).

        Channel convention (shared with the real firmware):
          ch0 = u_u, ch1 = u_v, ch2 = u_w  (SVPWM three-phase, for the
                                            Hexagon view)
          ch3 = target_iq, ch4 = i_q meas, ch5 = u_q command

        The SVM panel reads channels 0/1/2 exclusively; ordering them
        first gives the hexagon view a stable contract no matter how
        many extra channels the application adds later.

        On a real motor the SVPWM voltage vector keeps rotating at the
        electrical speed even at steady state (back-EMF soaks up u_q).
        Our demo plant has no mechanical model, so we synthesise a
        visible steady-state magnitude from the active set-point to
        mimic that rotation — otherwise the hexagon view would collapse
        to the origin as soon as i_q reached its reference."""
        import math
        angle = (time.monotonic() * 2.0 * math.pi * 4.0) % (2.0 * math.pi)
        if self.skip_torque:
            u_q_cmd = self.target_uq if self.override else 0.0
        else:
            u_q_cmd = self.kp * (self.target_iq - self._i_actual)
            if self.override:
                u_q_cmd += self.target_uq
        u_q_cmd = max(-self.motor.v_max, min(self.motor.v_max, u_q_cmd))
        # Visible magnitude: either the active loop effort or a fraction
        # of the target current, whichever is larger. The sign tracks the
        # reference so reversing iq swaps the vector direction.
        floor = 0.3 * abs(self.target_iq) if self.override else 0.0
        display_mag = max(abs(u_q_cmd), floor)
        if self.target_iq < 0:
            display_mag = -display_mag
        # Pure three-phase sinusoids...
        u_u_raw = display_mag * math.sin(angle)
        u_v_raw = display_mag * math.sin(angle - 2.0 * math.pi / 3.0)
        u_w_raw = display_mag * math.sin(angle + 2.0 * math.pi / 3.0)
        # ... with the same min-max common-mode injection the real
        # firmware SVPWM applies (source/motor_control/.../space_vector_
        # modulator.h: v_cm = (max+min)/2; phases are shifted by -v_cm).
        # This is what produces the characteristic "saddle" waveform
        # that lets SVPWM push 15 % more DC bus than pure sine.
        v_cm = 0.5 * (max(u_u_raw, u_v_raw, u_w_raw)
                      + min(u_u_raw, u_v_raw, u_w_raw))
        u_u = u_u_raw - v_cm
        u_v = u_v_raw - v_cm
        u_w = u_w_raw - v_cm
        fields = (u_u,
                  u_v,
                  u_w,
                  self.target_iq,
                  self._i_actual,
                  u_q_cmd,
                  0.0,
                  0.0,
                  0.0,
                  0.0,
                  0.0,
                  0.0,
                  42.0)
        return pack_scope_i32_to_payload(
            [self._float_to_q16_i32(f) for f in fields])

    # --- Thread lifecycle --------------------------------------------------

    def start(self) -> None:  # type: ignore[override]
        super().start()
        self._sim_thread.start()

    def stop(self) -> None:
        self._stop.set()

    # --- Simulation loop ---------------------------------------------------

    def _simulate(self) -> None:
        """Run the plant + PID at Ts (matches the firmware ISR_HOT_PATH
        loop rate of 40 kHz). Each wall-clock tick advances SUB_STEPS
        sub-iterations of the closed loop in tight Python so a host
        request like mpz_design(BW=4 kHz) lands on a plant that was
        actually discretised at the same rate the design was computed
        for. Scope frames are emitted every SUB_STEPS * SCOPE_DECIMATION
        sub-steps (= one frame per 4 ms wallclock) so the GUI's frame
        rate stays where it has been since the SVM panel was wired."""
        import math
        alpha = math.exp(-self.motor.r_ohm * self.motor.ts_s / self.motor.l_h)
        sub_counter = 0
        seq = 0
        # Wall-clock budget per outer iteration = SUB_STEPS * ts_s, e.g.
        # 40 * 25 us = 1 ms — same cadence the demo had before the
        # ts_s rebase to 40 kHz.
        wall_dt = self.SUB_STEPS * self.motor.ts_s
        while not self._stop.is_set():
            with self._state_lock:
                for _ in range(self.SUB_STEPS):
                    if self.skip_torque:
                        u = self.target_uq if self.override else 0.0
                        u = max(-self.motor.v_max, min(self.motor.v_max, u))
                        self._integ_prev = self._integ
                    else:
                        ref = self.target_iq if self.override else 0.0
                        err = ref - self._i_actual
                        u = self.kp * err + self._integ_prev
                        if self.override:
                            u += self.target_uq
                        u = max(-self.motor.v_max, min(self.motor.v_max, u))
                        self._integ_prev = self._integ
                        self._integ += self.ki * err * self.motor.ts_s
                        if self.lim > 0:
                            self._integ = max(-self.lim,
                                              min(self.lim, self._integ))
                    self._i_actual = (alpha * self._i_actual
                                      + (1.0 - alpha) / self.motor.r_ohm * u)
            sub_counter += 1
            if sub_counter >= self.SCOPE_DECIMATION:
                sub_counter = 0
                payload = self._scope_frame_payload()
                if len(payload) <= MAX_PAYLOAD:
                    try:
                        self._t.send_bytes(
                            encode(Channel.SCOPE, seq & 0xFF, payload))
                    except Exception:
                        pass
                    seq = (seq + 1) & 0xFF
            time.sleep(wall_dt)

    # --- Dispatcher --------------------------------------------------------

    def run(self) -> None:  # type: ignore[override]
        while not self._stop.is_set():
            data = self._t.read_bytes(64, timeout=0.05)
            if not data:
                continue
            for b in data:
                st = self._dec.push(b)
                if st == Status.OK:
                    if self._dec.channel == int(Channel.TUNER):
                        self._dispatch(self._dec.seq,
                                       bytes(self._dec.payload))
                    self._dec.reset()

    def _send_response(self, seq: int, status: int,
                       payload: bytes = b"") -> None:
        body = struct.pack("<bB", status, seq) + payload
        self._t.send_bytes(encode(Channel.TUNER, seq, body))

    def _dispatch(self, seq: int, payload: bytes) -> None:
        if len(payload) < 4:
            self._send_response(seq, ERR_INVALID_ARG)
            return
        op = payload[0]
        pid = payload[1] | (payload[2] << 8)
        cmd = payload[4:]

        with self._state_lock:
            if op == int(Op.READ):
                self._handle_read(seq, pid)
            elif op == int(Op.WRITE):
                self._handle_write(seq, pid, cmd)
            elif op == int(Op.EXEC):
                self._handle_exec(seq, pid, cmd)
            else:
                self._send_response(seq, ERR_INVALID_ARG)

    def _handle_read(self, seq: int, pid: int) -> None:
        if pid == int(ParamId.KP_Q16):
            self._send_response(seq, OK, _q16_to_bytes(self.kp))
        elif pid == int(ParamId.KI_Q16):
            self._send_response(seq, OK, _q16_to_bytes(self.ki))
        elif pid == int(ParamId.INT_LIM_Q16):
            self._send_response(seq, OK, _q16_to_bytes(self.lim))
        elif pid == int(ParamId.V_MAX_Q16):
            self._send_response(seq, OK, _q16_to_bytes(self.motor.v_max))
        elif pid == int(ParamId.AXIS_STATE):
            flags = AxisStateFlag.INITIALIZED
            if self.aligned:
                flags |= AxisStateFlag.ALIGNED
            if self.override:
                flags |= AxisStateFlag.TUNER_OVERRIDE
            self._send_response(seq, OK, bytes([int(flags)]))
        elif pid == int(ParamId.AXIS_LAST_ERR):
            self._send_response(seq, OK, struct.pack("<b", 0))
        elif pid == int(ParamId.I_FILTER_FC):
            self._send_response(seq, OK, _q16_to_bytes(self.current_filter_fc))
        elif pid == int(ParamId.LOOP_FS_HZ):
            # Demo firmware runs the synthetic plant + PID at ts_s, so
            # the "loop rate" exposed to the host matches that. Real
            # firmware reports either the PWM rate (ISR_HOT_PATH=y) or
            # pwm_rate / decimation (legacy).
            self._send_response(seq, OK,
                                _q16_to_bytes(1.0 / max(self.motor.ts_s, 1e-9)))
        elif pid == int(ParamId.NVS_PRESENT):
            self._send_response(seq, OK,
                                bytes([1 if self.nvs_overlay else 0]))
        elif pid == int(ParamId.FIRMWARE_TYPE):
            self._send_response(seq, OK,
                                struct.pack("<I", self.firmware_type))
        elif pid == int(ParamId.MOTOR_POLE_PAIRS):
            self._send_response(seq, OK,
                                struct.pack("<i", int(self.motor_pole_pairs)))
        elif pid == int(ParamId.SKIP_TORQUE):
            self._send_response(seq, OK,
                                bytes([1 if self.skip_torque else 0]))
        else:
            self._send_response(seq, ERR_INVALID_ARG)

    def _handle_write(self, seq: int, pid: int, cmd: bytes) -> None:
        if pid == int(ParamId.WRITE_SKIP_TORQUE):
            if len(cmd) < 1:
                self._send_response(seq, ERR_INVALID_ARG)
                return
            self.skip_torque = cmd[0] != 0
            self._send_response(seq, OK)
            return
        if len(cmd) < 4:
            self._send_response(seq, ERR_INVALID_ARG)
            return
        if pid == int(ParamId.WRITE_MOTOR_POLE_PAIRS):
            p = struct.unpack("<i", cmd[:4])[0]
            if not (1 <= p <= 64):
                self._send_response(seq, ERR_INVALID_ARG)
                return
            self.motor_pole_pairs = p
            self._send_response(seq, OK)
            return
        v = _q16_from_bytes(cmd[:4])
        if pid == int(ParamId.WRITE_KP):
            self.kp = v
            self._send_response(seq, OK)
        elif pid == int(ParamId.WRITE_KI):
            self.ki = v
            self._send_response(seq, OK)
        elif pid == int(ParamId.WRITE_INT_LIM):
            self.lim = v
            self._send_response(seq, OK)
        elif pid == int(ParamId.WRITE_I_FILTER_FC):
            self.current_filter_fc = v
            self._log(f"current filter: fc = {v:.1f} Hz")
            self._send_response(seq, OK)
        elif pid in (int(ParamId.WRITE_TARGET_ID),
                     int(ParamId.WRITE_TARGET_IQ),
                     int(ParamId.WRITE_TARGET_UD),
                     int(ParamId.WRITE_TARGET_UQ)):
            if not self.override:
                self._send_response(seq, ERR_AXIS_INVALID_STATE)
                return
            if pid == int(ParamId.WRITE_TARGET_ID): self.target_id = v
            elif pid == int(ParamId.WRITE_TARGET_IQ): self.target_iq = v
            elif pid == int(ParamId.WRITE_TARGET_UD): self.target_ud = v
            else: self.target_uq = v
            self._send_response(seq, OK)
        else:
            self._send_response(seq, ERR_INVALID_ARG)

    def _handle_exec(self, seq: int, pid: int, cmd: bytes) -> None:
        if pid == int(ParamId.CMD_OVERRIDE_ON):
            if not self.aligned:
                self._send_response(seq, ERR_NOT_ALIGNED)
                return
            self.override = True
            self._send_response(seq, OK)
        elif pid == int(ParamId.CMD_OVERRIDE_OFF):
            self.override = False
            self.target_id = 0.0
            self.target_iq = 0.0
            self._send_response(seq, OK)
        elif pid == int(ParamId.CMD_RECOMPUTE_GAINS):
            if len(cmd) < 12:
                self._send_response(seq, ERR_INVALID_ARG)
                return
            r = _q16_from_bytes(cmd[0:4])
            l = _q16_from_bytes(cmd[4:8])
            bw = _q16_from_bytes(cmd[8:12])
            try:
                motor = MotorParams(r_ohm=r, l_h=l, ts_s=self.motor.ts_s,
                                    v_max=self.motor.v_max)
                g = mpz_design(motor, bw)
            except ValueError:
                self._send_response(seq, ERR_INVALID_ARG)
                return
            self.motor = motor
            self.kp = g.kp
            self.ki = g.ki
            self.lim = g.int_lim
            self._send_response(seq, OK)
        elif pid == int(ParamId.CMD_ALIGN_AXIS):
            # Demo: no real motor, just announce success after a tiny
            # synthetic pause so the host's progress dialog has something
            # to show.
            self._log("alignment: started")
            time.sleep(0.4)
            self.aligned = True
            self._log("alignment: complete (direction = CW)")
            self._send_response(seq, OK)
        elif pid == int(ParamId.CMD_PERSIST_NVS):
            r = _q16_from_bytes(cmd[0:4]) if len(cmd) >= 4 else 0.0
            l = _q16_from_bytes(cmd[4:8]) if len(cmd) >= 8 else 0.0
            bw = _q16_from_bytes(cmd[8:12]) if len(cmd) >= 12 else 0.0
            self.nvs_overlay = {
                "kp": self.kp, "ki": self.ki, "lim": self.lim,
                "r": r, "l": l, "bw": bw,
                "fc": self.current_filter_fc,
                "pole_pairs": int(self.motor_pole_pairs),
            }
            self._log("calibration: saved to NVS")
            self._send_response(seq, OK)
        elif pid == int(ParamId.CMD_LOAD_NVS):
            if not self.nvs_overlay:
                self._log("calibration: nothing to load")
                self._send_response(seq, -3)  # AXIS_INVALID_STATE
                return
            self.kp = self.nvs_overlay["kp"]
            self.ki = self.nvs_overlay["ki"]
            self.lim = self.nvs_overlay["lim"]
            stored_fc = self.nvs_overlay.get("fc", 0.0)
            if stored_fc > 0:
                self.current_filter_fc = stored_fc
            pp = int(self.nvs_overlay.get("pole_pairs", 0))
            if 1 <= pp <= 64:
                self.motor_pole_pairs = pp
            self._log("calibration: NVS overlay applied")
            self._send_response(seq, OK)
        elif pid == int(ParamId.CMD_ERASE_NVS):
            self.nvs_overlay = None
            self._log("calibration: NVS namespace erased")
            self._send_response(seq, OK)
        elif pid == int(ParamId.CMD_RESET_BOARD):
            self._log("board: host requested reset (demo — no real reboot)")
            self._send_response(seq, OK)
        else:
            self._send_response(seq, ERR_INVALID_ARG)

    # --- LOG channel helper ------------------------------------------------

    def _log(self, msg: str) -> None:
        try:
            self._t.send_bytes(encode(Channel.LOG, 0,
                                      msg.encode("ascii", errors="replace")))
        except Exception:
            pass
