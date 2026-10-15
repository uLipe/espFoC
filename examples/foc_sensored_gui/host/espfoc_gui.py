#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Felipe Neves
# SPDX-License-Identifier: Apache-2.0
"""
espFoC motor control GUI.

A single-file desktop front end for examples/foc_sensored_gui and
examples/foc_sensorless_gui. It talks to the board over the console UART with
a tiny binary protocol and adapts to whichever firmware answers:

    sensored    commissioning phase map -> identification -> cogging learn,
                then torque, speed or position mode, three scopes
    sensorless  commissioning phase map -> identification, a start-up panel
                (arm, align, I-f ramp, PLL lock, handoff, running) with its
                Start button, then torque or speed mode; the scopes shade
                the start-up window

Scopes plot reference against measurement.

    pip install PySide6 pyserial
    python3 espfoc_gui.py                 # pick the port in the window
    python3 espfoc_gui.py --port /dev/ttyUSB0

How the file is organised, top to bottom:

    1. Protocol      frame layout, CRC-8, encoder and a resyncing parser
    2. SerialLink    pyserial in a reader thread, frames out as Qt signals
    3. ScopeItem     a QQuickItem that draws its traces as scene-graph
                     geometry, so the GPU (through Qt's RHI: OpenGL, Vulkan,
                     Metal or D3D, whatever the platform offers) does the
                     rasterising; Python only fills vertex arrays
    4. Backend       the state the QML binds to, and the commands it sends
    5. QML           the whole window, inline
    6. main()

The board is the source of truth. Every telemetry frame carries which
commissioning steps are done and what the board is busy with, so the GUI
never keeps its own copy of the sequence: reconnecting, or a board reboot,
just shows whatever the board reports next.
"""

import argparse
import math
import struct
import sys
import threading
from collections import deque

import serial
import serial.tools.list_ports
from PySide6.QtCore import QByteArray, QObject, QTimer, Property, Signal, Slot
from PySide6.QtGui import QColor, QGuiApplication
from PySide6.QtQml import QmlElement, QQmlApplicationEngine
from PySide6.QtQuick import (QQuickItem, QSGFlatColorMaterial, QSGGeometry,
                             QSGGeometryNode, QSGNode)
from PySide6.QtQuickControls2 import QQuickStyle

# =============================================================================
# 1. Protocol
#
#   0xA5 | type | len | payload[len] | crc8(type, len, payload)
#
# CRC-8 polynomial 0x07, initial value 0. Little endian, floats are IEEE-754
# single precision. Must match main/main.c.
# =============================================================================

SYNC = 0xA5
MAX_PAYLOAD = 64

# Host -> board
CMD_PING = 0x01   # board answers INFO
CMD_MAP = 0x02    # commissioning step 1
CMD_IDENT = 0x03  # commissioning step 2 (ends with the controller running)
CMD_COG = 0x04    # commissioning step 3
CMD_MODE = 0x05   # u8: 0 torque, 1 speed, 2 position
CMD_REF = 0x06    # f32 in the unit of the mode: A, rpm or deg
CMD_STOP = 0x07
CMD_START = 0x08  # sensorless only: launch with the current reference

# Board -> host
MSG_INFO = 0x81   # see INFO_FORMAT / INFO_SENSORLESS_FORMAT
MSG_TEL = 0x82    # telemetry, 10 Hz, see TEL_FORMAT
MSG_PROG = 0x83   # u8 step, u8 percent (0xFF = unknown)
MSG_LOG = 0x84    # text
MSG_ACK = 0x85    # u8 command, i32 esp_err_t
MSG_STARTUP = 0x86  # sensorless only, one per controller event, STARTUP_FORMAT

# u8 pole pairs, f32 max rpm, f32 max iq [A], f32 max deg. A sensorless
# board appends u8 kind (1), f32 min rpm, f32 min iq [A]: the references
# under which its observer cannot hold the rotor.
INFO_FORMAT = "<B3f"
INFO_SENSORLESS_FORMAT = "<B3fB2f"
KIND_SENSORED = 0
KIND_SENSORLESS = 1

# steps done, busy, controller state, mode, flags, then seven floats:
# speed ref/meas [rpm], position ref/meas [deg], iq ref/meas [A], Vdc [V]
TEL_FORMAT = "<BBBBB7f2x"

# event, state, failure, abort, fault, then observer speed [rpm], observer
# minus open-loop angle [deg], seconds since the start-up began
STARTUP_FORMAT = "<5B3f"

MODES = ("Torque", "Speed", "Position")
MODE_UNITS = ("A", "rpm", "deg")
STEP_NAMES = ("Phase map", "Identify", "Cogging learn")
BUSY_NAMES = ("", "phase map", "identification", "cogging learn", "starting")
STATE_NAMES = ("IDLE", "ARMED", "RUNNING", "LEARNING", "FAULT")
COMMAND_NAMES = {CMD_MAP: "MAP", CMD_IDENT: "IDENT", CMD_COG: "COG",
                 CMD_MODE: "MODE", CMD_REF: "REF", CMD_STOP: "STOP",
                 CMD_START: "START"}

FLAG_RUNNING = 1 << 0
FLAG_ABORT = 1 << 1
FLAG_FAULT = 1 << 2
FLAG_INPOS = 1 << 3       # sensored
FLAG_COGGING = 1 << 4     # sensored
FLAG_LOCKED = 1 << 3      # sensorless: observer locked
FLAG_OBSERVER = 1 << 4    # sensorless: Park on the observer angle
FLAG_START_FAIL = 1 << 5  # sensorless
FLAG_LAUNCH = 1 << 6      # sensorless: START given

# Sensorless controller (esp_foc_sensorless.h). The enums are mirrored here
# by value; the board sends them raw.
SL_STATE_NAMES = ("IDLE", "ARMED", "ALIGN", "I-F RAMP", "PLL ACQUIRE",
                  "HANDOFF", "CATCH", "RUNNING", "REVERSING", "FAULT")
SL_ARMED, SL_ALIGN, SL_CATCH, SL_RUNNING, SL_REVERSING = 1, 2, 6, 7, 8
(SL_EV_ARMED, SL_EV_STARTUP, SL_EV_LOCKED, SL_EV_HANDOFF, SL_EV_RUNNING,
 SL_EV_REVERSING, SL_EV_CUT, SL_EV_STOPPED, SL_EV_STARTUP_FAILED,
 SL_EV_ABORT, SL_EV_FAULT, SL_EV_FAULT_CLEARED) = range(12)
SL_FAIL_NAMES = ("", "inverter refused enable", "sequence config refused",
                 "no current on the ramp", "shaft not following the ramp",
                 "PLL lock timeout", "speed too low after handoff",
                 "speed dropped while settling")
SL_ABORT_NAMES = ("", "observer lock lost", "overspeed", "back-EMF too low",
                  "current collapse")
# Start-up stepper nodes, and which node each controller state is on.
STARTUP_STAGES = ("Arm", "Align", "I-f ramp", "PLL lock", "Handoff", "Running")
SL_STATE_STAGE = {1: 0, 2: 1, 3: 2, 4: 3, 5: 4, 6: 4, 7: 5}

# The esp_err_t values the board actually returns to commands.
ESP_ERR_NAMES = {
    -1: "ESP_FAIL", 0x101: "ESP_ERR_NO_MEM", 0x102: "ESP_ERR_INVALID_ARG",
    0x103: "ESP_ERR_INVALID_STATE", 0x105: "ESP_ERR_NOT_FOUND",
    0x106: "ESP_ERR_NOT_SUPPORTED", 0x107: "ESP_ERR_TIMEOUT",
    0x108: "ESP_ERR_INVALID_RESPONSE",
}


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def encode_frame(msg_type: int, payload: bytes = b"") -> bytes:
    body = bytes((msg_type, len(payload))) + payload
    return bytes((SYNC,)) + body + bytes((crc8(body),))


class FrameParser:
    """Turns a byte stream into (type, payload) tuples.

    The console also carries the ROM and bootloader banners as plain text,
    so anything that does not parse as a frame with a good CRC is skipped
    one byte at a time until the next 0xA5. Text that is not part of a
    frame is collected line by line and handed back too, so the boot log
    still shows up in the GUI.
    """

    def __init__(self):
        self._buf = bytearray()
        self._text = bytearray()

    def feed(self, data: bytes):
        """Returns (frames, text_lines) found in everything seen so far."""
        self._buf += data
        frames, lines = [], []
        while self._buf:
            if self._buf[0] != SYNC:
                self._take_text(self._buf.pop(0), lines)
                continue
            if len(self._buf) < 3:
                break                      # header not complete yet
            length = self._buf[2]
            if length > MAX_PAYLOAD:
                self._take_text(self._buf.pop(0), lines)
                continue
            if len(self._buf) < length + 4:
                break                      # payload not complete yet
            body = bytes(self._buf[1:3 + length])
            if crc8(body) == self._buf[3 + length]:
                frames.append((body[0], body[2:]))
                del self._buf[:length + 4]
            else:
                self._take_text(self._buf.pop(0), lines)
        return frames, lines

    def _take_text(self, byte, lines):
        if byte == 0x0A:
            text = self._text.decode("ascii", "replace").strip()
            if text:
                lines.append(text)
            self._text.clear()
        elif 0x20 <= byte < 0x7F and len(self._text) < 200:
            self._text.append(byte)


# =============================================================================
# 2. SerialLink
#
# pyserial blocks on read, so a plain Python thread owns the port's RX side.
# Qt delivers a signal emitted from that thread to slots of objects living in
# the GUI thread through the event queue, which is all the locking needed.
# =============================================================================

class SerialLink(QObject):
    frameReceived = Signal(int, bytes)
    textReceived = Signal(str)
    closed = Signal(str)

    def __init__(self):
        super().__init__()
        self._port = None
        self._thread = None
        self._running = False

    def open(self, name: str, baud: int = 115200):
        port = serial.Serial()
        port.port = name
        port.baudrate = baud
        port.timeout = 0.05
        # On the usual ESP32 auto-reset circuit DTR/RTS drive BOOT and EN:
        # leave both released so opening the port does not reset the chip.
        port.dtr = False
        port.rts = False
        port.open()
        self._port = port
        self._running = True
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def close(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._port is not None:
            self._port.close()
        self._port = None
        self._thread = None

    def is_open(self) -> bool:
        return self._port is not None

    def send(self, msg_type: int, payload: bytes = b""):
        if self._port is not None:
            try:
                self._port.write(encode_frame(msg_type, payload))
            except serial.SerialException as exc:
                self.closed.emit(str(exc))

    def _reader(self):
        parser = FrameParser()
        while self._running:
            try:
                data = self._port.read(256)
            except serial.SerialException as exc:
                self._running = False
                self.closed.emit(str(exc))
                return
            if not data:
                continue
            frames, lines = parser.feed(data)
            for line in lines:
                self.textReceived.emit(line)
            for msg_type, payload in frames:
                self.frameReceived.emit(msg_type, payload)


# =============================================================================
# 3. ScopeItem: a GPU-drawn strip chart
#
# QQuickItem with ItemHasContents set gets updatePaintNode() called on the
# render thread whenever update() was requested. There we hand Qt a small
# tree of QSGGeometryNodes: vertex arrays plus a flat-colour material. The
# scene graph batches and uploads them; no QPainter, no CPU rasterising.
#
# Lines thicker than one pixel are not portable across RHI backends, so each
# trace is a triangle strip: two vertices per sample, offset along the
# trace's normal by half the line width.
#
# Node tree (built once, geometry refreshed on every update):
#
#   root
#    +- shade      DrawTriangles, background bands over flagged samples
#    +- grid       DrawLines, thin
#    +- zero       DrawLines, the y = 0 line when it is in range
#    +- reference  DrawTriangleStrip
#    +- measured   DrawTriangleStrip
# =============================================================================

QML_IMPORT_NAME = "EspFoc"
QML_IMPORT_MAJOR_VERSION = 1

# QSGGeometry keeps a pointer to its attribute set, not a copy. The object
# PySide returns here must outlive every geometry built from it, or the
# geometry reads freed memory (garbage vertex size, then a crash).
POINT2D_ATTRIBUTES = QSGGeometry.defaultAttributes_Point2D()


@QmlElement
class ScopeItem(QQuickItem):
    rangeChanged = Signal()
    colorsChanged = Signal()

    GRID_X = 6          # vertical grid lines (5 s apart over 30 s)
    GRID_Y = 4          # horizontal grid lines
    LINE_WIDTH = 2.0    # trace width in pixels

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFlag(QQuickItem.ItemHasContents, True)
        self._history = 300                 # 30 s at the 10 Hz telemetry rate
        self._ref = deque(maxlen=self._history)
        self._meas = deque(maxlen=self._history)
        self._shade = deque(maxlen=self._history)
        self._y_min, self._y_max = -1.0, 1.0
        self._min_span = 1.0
        self._ref_color = QColor("#ffb020")
        self._meas_color = QColor("#36c5f0")
        self._grid_color = QColor("#2a3440")
        self._zero_color = QColor("#4a5868")
        self._shade_color = QColor("#2b2a1c")
        # Python keeps a handle on every scene-graph object it created, so
        # the garbage collector never frees one the renderer still uses.
        self._sg = None

    # --- properties the QML sets or reads ---------------------------------

    def _get_y_min(self):
        return self._y_min

    def _get_y_max(self):
        return self._y_max

    yMin = Property(float, _get_y_min, notify=rangeChanged)
    yMax = Property(float, _get_y_max, notify=rangeChanged)

    def _get_min_span(self):
        return self._min_span

    def _set_min_span(self, value):
        self._min_span = max(1e-6, float(value))
        self._rescale()

    minSpan = Property(float, _get_min_span, _set_min_span, notify=rangeChanged)

    def _get_ref_color(self):
        return self._ref_color

    def _set_ref_color(self, color):
        self._ref_color = QColor(color)
        self.colorsChanged.emit()
        self.update()

    refColor = Property(QColor, _get_ref_color, _set_ref_color, notify=colorsChanged)

    def _get_meas_color(self):
        return self._meas_color

    def _set_meas_color(self, color):
        self._meas_color = QColor(color)
        self.colorsChanged.emit()
        self.update()

    measColor = Property(QColor, _get_meas_color, _set_meas_color, notify=colorsChanged)

    # --- data in -----------------------------------------------------------

    @Slot(float, float, bool)
    def push(self, reference, measured, shaded):
        """One sample. shaded paints a band behind it (the sensorless
        start-up window)."""
        self._ref.append(reference)
        self._meas.append(measured)
        self._shade.append(bool(shaded))
        self._rescale()
        self.update()

    @Slot()
    def clear(self):
        self._ref.clear()
        self._meas.clear()
        self._shade.clear()
        self._rescale()
        self.update()

    def _rescale(self):
        """Autoscale y to everything on screen, never narrower than minSpan,
        with 10 % head room so a trace does not ride the frame."""
        values = list(self._ref) + list(self._meas)
        lo, hi = (min(values), max(values)) if values else (0.0, 0.0)
        if hi - lo < self._min_span:
            mid = 0.5 * (hi + lo)
            lo, hi = mid - 0.5 * self._min_span, mid + 0.5 * self._min_span
        pad = 0.1 * (hi - lo)
        lo, hi = lo - pad, hi + pad
        if (lo, hi) != (self._y_min, self._y_max):
            self._y_min, self._y_max = lo, hi
            self.rangeChanged.emit()

    def geometryChange(self, new_geometry, old_geometry):
        super().geometryChange(new_geometry, old_geometry)
        self.update()

    # --- scene graph (render thread) --------------------------------------

    @staticmethod
    def _make_node(mode, color):
        geometry = QSGGeometry(POINT2D_ATTRIBUTES, 0)
        geometry.setDrawingMode(mode)
        material = QSGFlatColorMaterial()
        material.setColor(color)
        node = QSGGeometryNode()
        node.setGeometry(geometry)
        node.setMaterial(material)
        return node, geometry, material

    @staticmethod
    def _fill(node, geometry, points):
        geometry.allocate(len(points))
        vertices = []
        for x, y in points:
            vertex = QSGGeometry.Point2D()
            vertex.set(x, y)
            vertices.append(vertex)
        if vertices:
            geometry.setVertexDataAsPoint2D(vertices)
        node.markDirty(QSGNode.DirtyGeometry)

    def _thick_strip(self, samples, to_px):
        """Triangle-strip vertices for a polyline of the given samples."""
        pts = [to_px(i, v) for i, v in enumerate(samples)]
        if len(pts) < 2:
            return []
        half = 0.5 * self.LINE_WIDTH
        strip = []
        for i, (x, y) in enumerate(pts):
            # Direction from the neighbours: a cheap miter that keeps the
            # width even at gentle bends, which is all a 10 Hz trace has.
            x0, y0 = pts[max(i - 1, 0)]
            x1, y1 = pts[min(i + 1, len(pts) - 1)]
            dx, dy = x1 - x0, y1 - y0
            norm = math.hypot(dx, dy) or 1.0
            nx, ny = -dy / norm * half, dx / norm * half
            strip.append((x + nx, y + ny))
            strip.append((x - nx, y - ny))
        return strip

    def updatePaintNode(self, old_node, _data):
        if old_node is None or self._sg is None:
            root = QSGNode()
            parts = {
                "shade": self._make_node(QSGGeometry.DrawTriangles, self._shade_color),
                "grid": self._make_node(QSGGeometry.DrawLines, self._grid_color),
                "zero": self._make_node(QSGGeometry.DrawLines, self._zero_color),
                "ref": self._make_node(QSGGeometry.DrawTriangleStrip, self._ref_color),
                "meas": self._make_node(QSGGeometry.DrawTriangleStrip, self._meas_color),
            }
            for node, _, _ in parts.values():
                root.appendChildNode(node)
            self._sg = (root, parts)
        root, parts = self._sg

        w, h = self.width(), self.height()
        span = (self._y_max - self._y_min) or 1.0
        step_x = w / max(self._history - 1, 1)
        # The newest sample sits on the right edge; older ones scroll left.
        offset = self._history - len(self._meas)

        def to_px(index, value):
            return ((index + offset) * step_x, h - (value - self._y_min) / span * h)

        # One rectangle (two triangles) per run of consecutive flagged
        # samples, half a sample wider on each side.
        shade, flags = [], list(self._shade) + [False]
        run_start = None
        for i, flagged in enumerate(flags):
            if flagged and run_start is None:
                run_start = i
            elif not flagged and run_start is not None:
                x0 = max(0.0, (run_start + offset - 0.5) * step_x)
                x1 = min(w, (i - 1 + offset + 0.5) * step_x)
                shade += [(x0, 0.0), (x1, 0.0), (x0, h), (x1, 0.0), (x1, h), (x0, h)]
                run_start = None
        self._fill(parts["shade"][0], parts["shade"][1], shade)

        grid = []
        for i in range(self.GRID_X + 1):
            x = i * w / self.GRID_X
            grid += [(x, 0.0), (x, h)]
        for i in range(self.GRID_Y + 1):
            y = i * h / self.GRID_Y
            grid += [(0.0, y), (w, y)]
        self._fill(parts["grid"][0], parts["grid"][1], grid)

        zero = []
        if self._y_min < 0.0 < self._y_max:
            y0 = h - (0.0 - self._y_min) / span * h
            zero = [(0.0, y0), (w, y0)]
        self._fill(parts["zero"][0], parts["zero"][1], zero)

        for key, samples, color in (("ref", self._ref, self._ref_color),
                                    ("meas", self._meas, self._meas_color)):
            node, geometry, material = parts[key]
            if material.color() != color:
                material.setColor(color)
                node.markDirty(QSGNode.DirtyMaterial)
            self._fill(node, geometry, self._thick_strip(list(samples), to_px))
        return root


# =============================================================================
# 4. Backend: what QML binds to
#
# Properties mirror the last telemetry frame. Slots are the buttons. The
# reference slider can fire hundreds of times a second; REF frames are sent
# at most every 50 ms, always with the newest value.
# =============================================================================

class Backend(QObject):
    portsChanged = Signal()
    connectionChanged = Signal()
    infoChanged = Signal()
    statusChanged = Signal()
    progressChanged = Signal()
    startupChanged = Signal()
    logLine = Signal(str)
    # speed ref/meas, position ref/meas, iq ref/meas, then whether the
    # sample falls in a sensorless start-up: one call per TEL frame
    telemetry = Signal(float, float, float, float, float, float, bool)

    REF_PERIOD_MS = 50

    def __init__(self):
        super().__init__()
        self._link = SerialLink()
        self._link.frameReceived.connect(self._on_frame)
        self._link.textReceived.connect(lambda text: self.logLine.emit("[boot] " + text))
        self._link.closed.connect(self._on_link_closed)
        self._ports = []
        self._port_name = ""
        # From INFO
        self._pole_pairs = 0
        self._max_rpm, self._max_iq, self._max_deg = 2000.0, 0.5, 3600.0
        self._kind = KIND_SENSORED
        self._min_rpm, self._min_iq = 0.0, 0.0
        # From STARTUP (sensorless): the stepper node reached, the node a
        # start-up failed on (-1 none), and what the last events reported
        self._stage = -1
        self._fail_stage = -1
        self._startup_text = ""
        self._lock_rpm = self._lock_dang = self._startup_time = float("nan")
        self._trip_reason = ""
        # From TEL
        self._steps_done = 0
        self._busy = 0
        self._state = 0
        self._mode = 2
        self._flags = 0
        self._vdc = 0.0
        self._last = (0.0,) * 6
        # From PROG / ACK
        self._progress = [0, 0, 0]
        self._failed = [False, False, False]
        self._pending_ref = None
        self._ref_timer = QTimer(self)
        self._ref_timer.setInterval(self.REF_PERIOD_MS)
        self._ref_timer.timeout.connect(self._flush_ref)
        self._ref_timer.start()
        self.refreshPorts()

    # --- connection --------------------------------------------------------

    def _get_ports(self):
        return self._ports

    ports = Property("QVariantList", _get_ports, notify=portsChanged)

    def _get_connected(self):
        return self._link.is_open()

    connected = Property(bool, _get_connected, notify=connectionChanged)

    def _get_port_name(self):
        return self._port_name

    portName = Property(str, _get_port_name, notify=connectionChanged)

    @Slot()
    def refreshPorts(self):
        self._ports = sorted(p.device for p in serial.tools.list_ports.comports())
        self.portsChanged.emit()

    @Slot(str)
    def connectPort(self, name):
        if self._link.is_open():
            self._link.close()
        try:
            self._link.open(name)
        except (serial.SerialException, OSError) as exc:
            self.logLine.emit(f"cannot open {name}: {exc}")
            return
        self._port_name = name
        self.connectionChanged.emit()
        self.logLine.emit(f"connected to {name}")
        self._link.send(CMD_PING)

    @Slot()
    def disconnectPort(self):
        self._link.close()
        self.connectionChanged.emit()
        self.logLine.emit("disconnected")

    def _on_link_closed(self, reason):
        self.logLine.emit(f"port closed: {reason}")
        self._link.close()
        self.connectionChanged.emit()

    # --- board limits (INFO) -------------------------------------------------

    def _get_max_rpm(self):
        return self._max_rpm

    def _get_max_iq(self):
        return self._max_iq

    def _get_max_deg(self):
        return self._max_deg

    def _get_pole_pairs(self):
        return self._pole_pairs

    def _get_kind(self):
        return self._kind

    def _get_min_rpm(self):
        return self._min_rpm

    def _get_min_iq(self):
        return self._min_iq

    def _get_commissioned(self):
        # stepsDone once every commissioning step is through
        return 0b011 if self._kind == KIND_SENSORLESS else 0b111

    maxRpm = Property(float, _get_max_rpm, notify=infoChanged)
    maxIq = Property(float, _get_max_iq, notify=infoChanged)
    maxDeg = Property(float, _get_max_deg, notify=infoChanged)
    polePairs = Property(int, _get_pole_pairs, notify=infoChanged)
    sensorless = Property(bool, lambda self: self._kind == KIND_SENSORLESS, notify=infoChanged)
    minRpm = Property(float, _get_min_rpm, notify=infoChanged)
    minIq = Property(float, _get_min_iq, notify=infoChanged)
    commissioned = Property(int, _get_commissioned, notify=infoChanged)

    # --- sensorless start-up (STARTUP) -------------------------------------

    startupStage = Property(int, lambda self: self._stage, notify=startupChanged)
    startupFailStage = Property(int, lambda self: self._fail_stage, notify=startupChanged)
    startupText = Property(str, lambda self: self._startup_text, notify=startupChanged)
    lockRpm = Property(float, lambda self: self._lock_rpm, notify=startupChanged)
    lockDang = Property(float, lambda self: self._lock_dang, notify=startupChanged)
    startupTime = Property(float, lambda self: self._startup_time, notify=startupChanged)

    # --- status (TEL) ------------------------------------------------------

    def _get_steps_done(self):
        return self._steps_done

    def _get_busy(self):
        return self._busy

    def _get_state(self):
        return self._state

    def _get_mode(self):
        return self._mode

    def _get_flags(self):
        return self._flags

    def _get_vdc(self):
        return self._vdc

    def _get_status_text(self):
        if not self._link.is_open():
            return "not connected"
        if self._busy:
            return "busy: " + BUSY_NAMES[self._busy] if self._busy < len(BUSY_NAMES) else "busy"
        if self._kind == KIND_SENSORLESS:
            return self._sensorless_status()
        text = STATE_NAMES[self._state] if self._state < len(STATE_NAMES) else "?"
        if self._flags & FLAG_ABORT:
            text += "  (abort, press STOP to clear)"
        if self._flags & FLAG_FAULT:
            text += "  (fault, press STOP to clear)"
        if self._steps_done == 0b111 and self._state == 0:
            text += "  - select a mode to start again"
        return text

    def _sensorless_status(self):
        text = SL_STATE_NAMES[self._state] if self._state < len(SL_STATE_NAMES) else "?"
        if self._flags & (FLAG_START_FAIL | FLAG_ABORT | FLAG_FAULT):
            return text + f"  ({self._trip_reason or 'tripped'}, press STOP to clear)"
        if (self._steps_done == 0b011 and self._state == SL_ARMED
                and not self._flags & FLAG_LAUNCH):
            text += "  - set a reference and press Start"
        return text

    def _get_measured(self):
        return list(self._last)

    stepsDone = Property(int, _get_steps_done, notify=statusChanged)
    busy = Property(int, _get_busy, notify=statusChanged)
    state = Property(int, _get_state, notify=statusChanged)
    mode = Property(int, _get_mode, notify=statusChanged)
    flags = Property(int, _get_flags, notify=statusChanged)
    vdc = Property(float, _get_vdc, notify=statusChanged)
    statusText = Property(str, _get_status_text, notify=statusChanged)
    measured = Property("QVariantList", _get_measured, notify=statusChanged)

    # --- commissioning progress (PROG / ACK) --------------------------------

    def _get_progress(self):
        return self._progress

    def _get_failed(self):
        return self._failed

    progress = Property("QVariantList", _get_progress, notify=progressChanged)
    failed = Property("QVariantList", _get_failed, notify=progressChanged)

    # --- commands ----------------------------------------------------------

    def _start_step(self, step, command):
        self._failed[step] = False
        self._progress[step] = 0
        self.progressChanged.emit()
        self.logLine.emit(f"> {STEP_NAMES[step]}")
        self._link.send(command)

    @Slot()
    def runMap(self):
        self._start_step(0, CMD_MAP)

    @Slot()
    def runIdent(self):
        self._start_step(1, CMD_IDENT)

    @Slot()
    def runCog(self):
        self._start_step(2, CMD_COG)

    @Slot(int)
    def setMode(self, mode):
        self._pending_ref = None
        self.logLine.emit(f"> mode {MODES[mode]}")
        self._link.send(CMD_MODE, struct.pack("<B", mode))

    @Slot(float)
    def sendRef(self, value):
        self._pending_ref = value

    def _flush_ref(self):
        if self._pending_ref is not None and self._link.is_open():
            self._link.send(CMD_REF, struct.pack("<f", self._pending_ref))
            self._pending_ref = None

    @Slot(float)
    def startMotor(self, value):
        """Sensorless launch: the reference first, so the board starts on
        what the slider shows rather than on a REF still in the throttle."""
        self._pending_ref = None
        self._link.send(CMD_REF, struct.pack("<f", value))
        self._link.send(CMD_START)
        self.logLine.emit("> Start")

    @Slot()
    def stop(self):
        # STOP skips the REF throttle and goes out now.
        self._pending_ref = None
        self._link.send(CMD_STOP)
        self.logLine.emit("> STOP")

    # --- frames in -----------------------------------------------------------

    def _on_frame(self, msg_type, payload):
        if msg_type == MSG_TEL and len(payload) == struct.calcsize(TEL_FORMAT):
            (self._steps_done, self._busy, self._state, self._mode, self._flags,
             w_ref, w, th_ref, th, iq_ref, iq, self._vdc) = struct.unpack(TEL_FORMAT, payload)
            self._last = (w_ref, w, th_ref, th, iq_ref, iq)
            starting = False
            if self._kind == KIND_SENSORLESS:
                starting = SL_ALIGN <= self._state <= SL_CATCH or self._state == SL_REVERSING
                self._track_stage()
            self.statusChanged.emit()
            self.telemetry.emit(w_ref, w, th_ref, th, iq_ref, iq, starting)
        elif msg_type == MSG_STARTUP and len(payload) == struct.calcsize(STARTUP_FORMAT):
            self._on_startup(*struct.unpack(STARTUP_FORMAT, payload))
        elif msg_type == MSG_PROG and len(payload) == 2:
            step, pct = payload
            if step < 3:
                self._progress[step] = -1 if pct == 0xFF else pct
                self.progressChanged.emit()
        elif msg_type == MSG_LOG:
            self.logLine.emit(payload.decode("utf-8", "replace"))
        elif msg_type == MSG_ACK and len(payload) == 5:
            command, err = struct.unpack("<Bi", payload)
            self._on_ack(command, err)
        elif msg_type == MSG_INFO and len(payload) == struct.calcsize(INFO_FORMAT):
            self._pole_pairs, self._max_rpm, self._max_iq, self._max_deg = \
                struct.unpack(INFO_FORMAT, payload)
            self._kind = KIND_SENSORED
            self.infoChanged.emit()
            self.logLine.emit(f"sensored board: {self._pole_pairs} pole pairs, "
                              f"|speed| <= {self._max_rpm:.0f} rpm, "
                              f"|iq| <= {self._max_iq:.2f} A, "
                              f"|position| <= {self._max_deg:.0f} deg")
        elif msg_type == MSG_INFO and len(payload) == struct.calcsize(INFO_SENSORLESS_FORMAT):
            (self._pole_pairs, self._max_rpm, self._max_iq, self._max_deg,
             self._kind, self._min_rpm, self._min_iq) = \
                struct.unpack(INFO_SENSORLESS_FORMAT, payload)
            self.infoChanged.emit()
            self.logLine.emit(f"sensorless board: {self._pole_pairs} pole pairs, "
                              f"speed {self._min_rpm:.0f}..{self._max_rpm:.0f} rpm, "
                              f"iq {self._min_iq:.2f}..{self._max_iq:.2f} A")

    def _on_ack(self, command, err):
        step = {CMD_MAP: 0, CMD_IDENT: 1, CMD_COG: 2}.get(command)
        if step is not None:
            self._failed[step] = err != 0
            if err == 0:
                self._progress[step] = 100
            self.progressChanged.emit()
        if err != 0:
            name = ESP_ERR_NAMES.get(err, f"0x{err:x}")
            self.logLine.emit(f"{COMMAND_NAMES.get(command, hex(command))} refused: {name}")
        elif step is not None:
            self.logLine.emit(f"{STEP_NAMES[step]}: done")

    def _track_stage(self):
        """Advance the stepper from TEL, which also covers the states no
        event announces (I-f ramp, PLL acquire)."""
        stage = SL_STATE_STAGE.get(self._state)
        if stage is None or self._fail_stage >= 0:
            return
        if self._stage < 0 or stage > self._stage:
            self._stage = stage
            if stage == 2:
                self._startup_text = "open-loop I-f ramp"
            elif stage == 3:
                self._startup_text = "PLL acquiring the rotor"
            self.startupChanged.emit()

    def _on_startup(self, event, state, fail, abort, fault, rpm, dang, t_s):
        if event == SL_EV_STARTUP:
            self._stage, self._fail_stage = 1, -1
            self._lock_rpm = self._lock_dang = self._startup_time = float("nan")
            self._startup_text = "aligning the rotor"
        elif event == SL_EV_LOCKED:
            self._lock_rpm, self._lock_dang = rpm, dang
            self._stage = max(self._stage, 4)
            self._startup_text = "observer locked"
        elif event == SL_EV_HANDOFF:
            self._stage = max(self._stage, 4)
            self._startup_text = "handoff: Park on the observer"
        elif event == SL_EV_RUNNING:
            self._stage, self._startup_time = 5, t_s
            self._startup_text = f"running, start-up took {t_s:.2f} s"
        elif event == SL_EV_REVERSING:
            self._startup_text = "reversing: slowing down to relaunch"
        elif event == SL_EV_CUT:
            self._startup_text = "bridge off, coasting"
        elif event == SL_EV_STARTUP_FAILED:
            self._fail_stage = SL_STATE_STAGE.get(state, max(self._stage, 0))
            name = SL_FAIL_NAMES[fail] if fail < len(SL_FAIL_NAMES) else f"failure {fail}"
            self._trip_reason = self._startup_text = "start-up failed: " + name
        elif event == SL_EV_ABORT:
            name = SL_ABORT_NAMES[abort] if abort < len(SL_ABORT_NAMES) else f"abort {abort}"
            self._trip_reason = self._startup_text = "abort: " + name
        elif event == SL_EV_FAULT:
            self._trip_reason = self._startup_text = f"inverter fault {fault}"
        elif event == SL_EV_FAULT_CLEARED:
            self._stage, self._fail_stage = 0, -1
            self._trip_reason, self._startup_text = "", "cleared, armed"
        else:
            return
        self.startupChanged.emit()
        self.statusChanged.emit()

    def shutdown(self):
        if self._link.is_open():
            self._link.send(CMD_STOP)
            self._link.close()


# =============================================================================
# 5. QML
#
# One window: controls on the left, scopes on the right, status bar below.
# Everything enables itself from backend properties, so the buttons follow
# what the board reports rather than what was clicked.
# =============================================================================

QML = r"""
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import EspFoc

ApplicationWindow {
    id: win
    width: 1440; height: 880
    visible: true
    title: backend.sensorless ? "espFoC - sensorless motor control"
                              : "espFoC - sensored motor control"
    color: "#12171d"

    // Set on the window, the palette reaches every control below it. An
    // application QPalette does not: the style follows the OS colour scheme.
    palette {
        window: "#12171d"; windowText: "#c8d2dc"
        base: "#0e1318"; alternateBase: "#1a2129"; text: "#c8d2dc"
        button: "#26303b"; buttonText: "#e0e6ec"
        highlight: "#1f7fa8"; highlightedText: "#ffffff"
        light: "#3a4654"; midlight: "#303a46"; mid: "#2a3440"; dark: "#0e1318"
        shadow: "#000000"; placeholderText: "#5d6a78"
    }

    // Axis label with as many decimals as the visible span needs.
    function axisLabel(value, span) {
        const decimals = Math.max(0, Math.min(4, 2 - Math.floor(Math.log10(span))))
        return value.toFixed(decimals)
    }

    readonly property color panel: "#1a2129"
    readonly property color edge: "#2a3440"
    readonly property color dim: "#8a97a6"
    readonly property color accent: "#36c5f0"
    readonly property color refCol: "#ffb020"
    readonly property bool ready: backend.connected && backend.busy === 0
                                  && backend.stepsDone === backend.commissioned
    // A sensorless controller that latched a failure, abort or fault.
    readonly property bool tripped: (backend.flags & (2 | 4 | 32)) !== 0
    onReadyChanged: if (ready) control.resetFields(backend.mode)

    // Esc is STOP from anywhere in the window.
    Shortcut { sequence: "Esc"; onActivated: backend.stop() }

    // A titled panel: everything on the left is built from these.
    component Panel: Rectangle {
        property string title
        default property alias content: body.data
        color: win.panel; radius: 6; border.color: win.edge
        Layout.fillWidth: true
        implicitHeight: body.implicitHeight + 44
        Label {
            text: parent.title; color: win.dim; font.pixelSize: 12
            font.capitalization: Font.AllUppercase; font.letterSpacing: 1
            x: 12; y: 10
        }
        ColumnLayout { id: body; x: 12; y: 32; width: parent.width - 24; spacing: 8 }
    }

    // One commissioning step: button, status chip and its progress bar.
    component StepRow: ColumnLayout {
        property int step
        property bool unlocked
        function start() {
            if (step === 0) backend.runMap()
            else if (step === 1) backend.runIdent()
            else backend.runCog()
        }
        readonly property bool done: (backend.stepsDone >> step) & 1
        readonly property bool running: backend.busy === step + 1
        readonly property bool failed: backend.failed[step]
        Layout.fillWidth: true
        spacing: 4
        RowLayout {
            Layout.fillWidth: true
            Button {
                text: (step + 1) + ". " + ["Phase map", "Identify", "Cogging learn"][step]
                Layout.preferredWidth: 190
                enabled: backend.connected && unlocked && !done && backend.busy === 0
                onClicked: start()
            }
            Item { Layout.fillWidth: true }
            Rectangle {
                radius: 9; implicitHeight: 20; implicitWidth: chip.implicitWidth + 16
                color: done ? "#1e4d2b" : running ? "#1d3d55" : failed ? "#5a1f1f" : "#262f39"
                Label {
                    id: chip; anchors.centerIn: parent; font.pixelSize: 11
                    text: done ? "done" : running ? "running" : failed ? "failed"
                                : unlocked ? "ready" : "locked"
                }
            }
        }
        ProgressBar {
            Layout.fillWidth: true
            from: 0; to: 100
            value: done ? 100 : Math.max(0, backend.progress[step])
            indeterminate: running && backend.progress[step] < 0
            visible: running || done || failed
        }
    }

    RowLayout {
        anchors.fill: parent
        anchors.margins: 12
        anchors.bottomMargin: 40
        spacing: 12

        // ---------------------------------------------------------- controls
        ColumnLayout {
            Layout.preferredWidth: 400
            Layout.maximumWidth: 400
            Layout.fillHeight: true
            spacing: 12

            Panel {
                title: "Connection"
                RowLayout {
                    Layout.fillWidth: true
                    ComboBox {
                        id: portBox
                        Layout.fillWidth: true
                        model: backend.ports
                        enabled: !backend.connected
                        Component.onCompleted: {
                            const i = find(initialPort)
                            if (i >= 0) currentIndex = i
                        }
                    }
                    Button {
                        text: "\u21bb"; enabled: !backend.connected
                        onClicked: backend.refreshPorts()
                    }
                    Button {
                        text: backend.connected ? "Disconnect" : "Connect"
                        enabled: backend.connected || portBox.currentText !== ""
                        onClicked: backend.connected ? backend.disconnectPort()
                                                     : backend.connectPort(portBox.currentText)
                    }
                }
            }

            Panel {
                title: "Commissioning"
                StepRow { step: 0; unlocked: true }
                StepRow { step: 1; unlocked: backend.stepsDone & 1 }
                StepRow { step: 2; unlocked: backend.stepsDone & 2; visible: !backend.sensorless }
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: backend.sensorless ? 130 : 200
                    color: "#0e1318"; radius: 4; border.color: win.edge
                    ListView {
                        id: logView
                        anchors.fill: parent; anchors.margins: 6
                        clip: true
                        model: ListModel { id: logModel }
                        delegate: Text {
                            width: logView.width; wrapMode: Text.WrapAnywhere
                            text: line; color: line.startsWith(">") ? win.accent : "#c8d2dc"
                            font.family: "monospace"; font.pixelSize: 11
                        }
                        ScrollBar.vertical: ScrollBar {}
                    }
                }
            }

            // Sensorless only: where the last start-up got to, and the
            // launch button. The stepper follows the board's controller
            // state and events; a failed node turns red with the reason.
            Panel {
                title: "Startup"
                visible: backend.sensorless
                enabled: win.ready
                opacity: enabled ? 1.0 : 0.45
                Item {
                    Layout.fillWidth: true
                    implicitHeight: 46
                    Repeater {
                        id: stages
                        model: ["Arm", "Align", "I-f ramp", "PLL lock", "Handoff", "Running"]
                        Item {
                            readonly property real slot: parent.width / stages.count
                            readonly property bool failed: backend.startupFailStage === index
                            readonly property bool done: (backend.startupFailStage < 0
                                                          || index < backend.startupFailStage)
                                                         && (index < backend.startupStage
                                                             || backend.startupStage === 5)
                            readonly property bool active: !failed && !done
                                                           && index === backend.startupStage
                            x: index * slot; width: slot; height: parent.height
                            Rectangle {
                                // connector to the previous node
                                visible: index > 0
                                x: -parent.slot / 2 + 12; y: 10
                                width: parent.slot - 24; height: 2
                                color: done || active || failed ? "#4a8a5a" : win.edge
                            }
                            Rectangle {
                                id: dot
                                x: (parent.width - width) / 2; width: 22; height: 22; radius: 11
                                color: failed ? "#c62828" : done ? "#2e7d32"
                                       : active ? "#1f7fa8" : "#262f39"
                                border.color: active ? win.accent : "transparent"
                                Label {
                                    anchors.centerIn: parent; font.pixelSize: 11
                                    text: failed ? "\u2715" : done ? "\u2713" : index + 1
                                }
                            }
                            Label {
                                anchors.horizontalCenter: dot.horizontalCenter
                                y: 26; font.pixelSize: 11
                                text: modelData
                                color: failed ? "#ff6b6b" : active ? win.accent : win.dim
                            }
                        }
                    }
                }
                Label {
                    Layout.fillWidth: true; elide: Text.ElideRight
                    text: backend.startupText || " "
                    color: backend.startupFailStage >= 0 || win.tripped ? "#ff6b6b" : "#c8d2dc"
                }
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    function show(v, decimals, unit) {
                        return isNaN(v) ? "\u2013" : v.toFixed(decimals) + " " + unit
                    }
                    Label { color: win.dim; text: "\u03c9\u0302 lock"; font.pixelSize: 11 }
                    Label { text: parent.show(backend.lockRpm, 0, "rpm") }
                    Label { color: win.dim; text: "\u03b8\u0302\u2212\u03b8ol"; font.pixelSize: 11 }
                    Label { text: parent.show(backend.lockDang, 1, "deg") }
                    Label { color: win.dim; text: "took"; font.pixelSize: 11 }
                    Label { text: parent.show(backend.startupTime, 2, "s") }
                }
                Button {
                    objectName: "startButton"
                    text: "Start"
                    Layout.fillWidth: true
                    highlighted: enabled
                    enabled: win.ready && backend.state === 1 && !win.tripped
                    onClicked: backend.startMotor(control.command)
                }
            }

            Panel {
                title: "Control"
                enabled: win.ready
                opacity: enabled ? 1.0 : 0.45

                // Segmented mode selector. The sensorless controller has no
                // position mode and only changes mode while stopped.
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 0
                    Repeater {
                        model: backend.sensorless ? ["Torque", "Speed"]
                                                  : ["Torque", "Speed", "Position"]
                        Button {
                            Layout.fillWidth: true
                            text: modelData
                            checkable: true
                            checked: backend.mode === index
                            highlighted: checked
                            enabled: !backend.sensorless || backend.state <= 1
                            onClicked: control.selectMode(index)
                        }
                    }
                }

                // The control logic lives in this item so the slider, the
                // limit fields and the direction box share it.
                Item {
                    id: control
                    objectName: "modeControl"
                    Layout.fillWidth: true
                    implicitHeight: controlBody.implicitHeight
                    readonly property var units: ["A", "rpm", "deg"]
                    readonly property var limits: [backend.maxIq, backend.maxRpm, backend.maxDeg]
                    readonly property real limit: limits[backend.mode]
                    readonly property real command: slider.value * (reverse.checked ? -1 : 1)

                    readonly property real observerMin: backend.mode === 0 ? backend.minIq
                                                                           : backend.minRpm

                    function resetFields(m) {
                        // Torque and speed start at zero, position starts
                        // where the shaft is, so a mode change never jumps.
                        // Sensorless starts at the observer's minimum: any
                        // less and the board would not launch.
                        reverse.checked = false
                        const decimals = m === 0 ? 2 : 0
                        if (m === 2) {
                            const here = backend.measured[3]
                            minField.text = Math.round(here - 360)
                            maxField.text = Math.round(here + 360)
                            slider.value = here
                        } else {
                            const low = backend.sensorless ? (m === 0 ? backend.minIq : backend.minRpm) : 0
                            minField.text = low.toFixed(decimals)
                            maxField.text = (m === 0 ? backend.maxIq : backend.maxRpm).toFixed(decimals)
                            slider.value = low
                        }
                    }
                    function selectMode(m) {
                        resetFields(m)
                        backend.setMode(m)
                    }
                    function clampField(t) {
                        const v = Number(t)
                        return isNaN(v) ? 0 : Math.max(-limit, Math.min(limit, v))
                    }

                    ColumnLayout {
                        id: controlBody
                        width: parent.width
                        RowLayout {
                            Layout.fillWidth: true
                            Label { text: "min"; color: win.dim }
                            TextField {
                                id: minField; Layout.preferredWidth: 80; text: "-360"
                                validator: DoubleValidator {}
                            }
                            Item { Layout.fillWidth: true }
                            Label { text: "max"; color: win.dim }
                            TextField {
                                id: maxField; Layout.preferredWidth: 80; text: "360"
                                validator: DoubleValidator {}
                            }
                        }
                        Slider {
                            id: slider
                            objectName: "refSlider"
                            Layout.fillWidth: true
                            from: control.clampField(minField.text)
                            to: control.clampField(maxField.text)
                            onMoved: backend.sendRef(control.command)
                        }
                        RowLayout {
                            Layout.fillWidth: true
                            CheckBox {
                                id: reverse; objectName: "reverseBox"
                                text: "Reverse direction"
                                onToggled: backend.sendRef(control.command)
                            }
                            Item { Layout.fillWidth: true }
                            Label {
                                text: control.command.toFixed(backend.mode === 0 ? 3 : 1)
                                      + " " + control.units[backend.mode]
                                font.pixelSize: 20; color: win.refCol
                            }
                        }
                        Label {
                            visible: backend.sensorless
                            Layout.fillWidth: true; wrapMode: Text.WordWrap
                            color: win.dim; font.pixelSize: 11
                            text: "below " + control.observerMin.toFixed(backend.mode === 0 ? 2 : 0)
                                  + " " + control.units[backend.mode]
                                  + " the observer loses the rotor and the board cuts the bridge"
                        }
                    }
                }
            }

            Item { Layout.fillHeight: true }

            Button {
                Layout.fillWidth: true
                Layout.preferredHeight: 64
                enabled: backend.connected
                text: "STOP  (Esc)"
                font.pixelSize: 22; font.bold: true
                palette.button: "#c62828"; palette.buttonText: "white"
                onClicked: backend.stop()
            }
        }

        // ------------------------------------------------------------ scopes
        ColumnLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 12
            Repeater {
                id: scopes
                model: [
                    { title: "Speed", unit: "rpm", span: 50 },
                    { title: "Position", unit: "deg", span: 20 },
                    { title: "Torque current iq", unit: "A", span: 0.1 }
                ]
                Rectangle {
                    property alias scope: plot
                    visible: index !== 1 || !backend.sensorless
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    color: win.panel; radius: 6; border.color: win.edge
                    readonly property int valueIndex: index * 2
                    Label {
                        x: 12; y: 8; text: modelData.title + "  [" + modelData.unit + "]"
                        color: win.dim; font.pixelSize: 12
                        font.capitalization: Font.AllUppercase; font.letterSpacing: 1
                    }
                    Row {
                        anchors.right: parent.right; anchors.rightMargin: 12; y: 6
                        spacing: 16
                        Label {
                            color: win.refCol; font.pixelSize: 13
                            text: "\u25ac reference " + backend.measured[valueIndex].toFixed(2)
                        }
                        Label {
                            color: win.accent; font.pixelSize: 13
                            text: "\u25ac measured " + backend.measured[valueIndex + 1].toFixed(2)
                        }
                    }
                    // y-axis labels, bound to the scope's autoscaled range
                    Column {
                        x: 8; y: plot.y - 7; width: 56
                        Repeater {
                            model: 5
                            Label {
                                height: plot.height / 4
                                width: 56; horizontalAlignment: Text.AlignRight
                                color: win.dim; font.pixelSize: 10
                                text: win.axisLabel(plot.yMax - index * (plot.yMax - plot.yMin) / 4,
                                                    plot.yMax - plot.yMin)
                            }
                        }
                    }
                    ScopeItem {
                        id: plot
                        x: 72; y: 34
                        width: parent.width - 84
                        height: parent.height - 56
                        minSpan: modelData.span
                        refColor: win.refCol
                        measColor: win.accent
                    }
                    Label {
                        x: plot.x; y: plot.y + plot.height + 2
                        text: "-30 s"; color: win.dim; font.pixelSize: 10
                    }
                    Label {
                        x: plot.x + plot.width - width; y: plot.y + plot.height + 2
                        text: "now"; color: win.dim; font.pixelSize: 10
                    }
                }
            }
        }
    }

    // ---------------------------------------------------------- status bar
    Rectangle {
        anchors.left: parent.left; anchors.right: parent.right; anchors.bottom: parent.bottom
        height: 30; color: "#0e1318"
        Row {
            anchors.verticalCenter: parent.verticalCenter; x: 12; spacing: 28
            Label { text: backend.connected ? "\u25cf " + backend.portName : "\u25cb offline"
                    color: backend.connected ? "#4caf50" : win.dim }
            Label { text: "state: " + backend.statusText; color: "#c8d2dc" }
            Label { text: "Vdc " + backend.vdc.toFixed(1) + " V"; color: "#c8d2dc" }
            // bits 3 and 4 mean different things on each board kind
            Label {
                text: !(backend.flags & 16) ? ""
                      : backend.sensorless ? "Park on observer" : "cogging ff on"
                color: win.dim
            }
            Label {
                text: !(backend.flags & 8) ? ""
                      : backend.sensorless ? "PLL locked" : "in position"
                color: "#4caf50"
            }
        }
    }

    // ------------------------------------------------------------- wiring
    Connections {
        target: backend
        function onTelemetry(wr, w, pr, p, ir, i, starting) {
            scopes.itemAt(0).scope.push(wr, w, starting)
            scopes.itemAt(1).scope.push(pr, p, starting)
            scopes.itemAt(2).scope.push(ir, i, starting)
        }
        function onLogLine(text) {
            logModel.append({ line: text })
            if (logModel.count > 500) logModel.remove(0)
            logView.positionViewAtEnd()
        }
    }
}
"""


# =============================================================================
# 6. main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="espFoC motor control GUI")
    parser.add_argument("--port", default="", help="serial port to preselect and open")
    args = parser.parse_args()

    app = QGuiApplication(sys.argv)
    # Fusion draws every control from the palette the QML window sets, so the
    # GUI looks the same on every OS. Must be chosen before QML loads.
    QQuickStyle.setStyle("Fusion")
    backend = Backend()
    engine = QQmlApplicationEngine()
    engine.rootContext().setContextProperty("backend", backend)
    engine.rootContext().setContextProperty("initialPort", args.port)
    engine.loadData(QByteArray(QML.encode()))
    if not engine.rootObjects():
        return 1
    if args.port:
        backend.connectPort(args.port)
    app.aboutToQuit.connect(backend.shutdown)
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
