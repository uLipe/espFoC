#!/usr/bin/env python3
"""Generate the animated tour GIF used in the main README.

Boots the full GUI against the in-process DemoFirmware (same setup
as the smoke test), walks the operator through every tab, fires a
tuner override + iq command so the SVM hexagon actually rotates,
then opens the Generate App tab and simulates a firmware build by
streaming canned text into the build log. Each tick captures the
window via `QWidget.grab()` and the frames are stitched into a GIF
with Pillow.

Run locally:

    pip install Pillow
    PYTHONPATH=tools python3 tools/espfoc_studio/scripts/make_demo_gif.py \
        --output doc/images/tuner_studio.gif

Run headless (CI / sandbox — fonts and antialiasing may differ from
the operator's display, so prefer running on a real desktop for the
public asset):

    QT_QPA_PLATFORM=offscreen PYTHONPATH=tools \
        python3 tools/espfoc_studio/scripts/make_demo_gif.py \
        --output /tmp/demo.gif
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Callable, List, Optional, Tuple

# Default to offscreen if no display — the script still works for
# preview, just not pretty.
os.environ.setdefault("QT_QPA_PLATFORM",
                      "xcb" if os.environ.get("DISPLAY") else "offscreen")

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(HERE))))

from PySide6.QtCore import Qt
from PySide6.QtGui import QImage
from PySide6.QtWidgets import QApplication

from espfoc_studio.gui.demo_firmware import DemoFirmware
from espfoc_studio.gui.main_window import MainWindow
from espfoc_studio.gui.theme import apply_dark_theme
from espfoc_studio.link import LinkReader, LoopbackTransport
from espfoc_studio.protocol import TunerClient


# Scene = (label, tab_index, on_enter callback, dwell seconds).
# Tab order matches MainWindow: Analysis(0) / Sensors(1) / Scope(2) /
# SVM Hexagon(3) / Generate App(4, --demo only).
Scene = Tuple[str, int, Callable[[MainWindow, TunerClient], None], float]


def _scene_analysis(_w: MainWindow, _c: TunerClient) -> None:
    pass


def _scene_scope(_w: MainWindow, _c: TunerClient) -> None:
    pass


def _scene_svm_override(_w: MainWindow, c: TunerClient) -> None:
    """Engage the override and command +1.5 A on iq so the hexagon
    actually rotates and the trail draws something interesting."""
    try:
        c.override_on()
        c.write_target_iq(1.5)
    except Exception:
        pass


def _scene_generate_idle(_w: MainWindow, c: TunerClient) -> None:
    # Drop the override before showing the Generate tab so the
    # waveform stops dancing under the new section.
    try:
        c.override_off()
    except Exception:
        pass


_FAKE_BUILD_LINES = [
    ">>> building tuner_studio_target for esp32s3",
    "    IDF_PATH = /opt/esp/esp-idf",
    "Executing action: all (aliases: build)",
    "Running cmake in directory build",
    "[1/812] Building C object esp-idf/freertos/CMakeFiles/...",
    "[212/812] Building C object esp-idf/espFoC/esp_foc_core.c.obj",
    "[418/812] Building C object esp-idf/espFoC/esp_foc_tuner.c.obj",
    "[604/812] Building C object esp-idf/main/main.c.obj",
    "[810/812] Generating binary image from built executable",
    "[811/812] cd .../build && check_sizes.py --offset 0x10000",
    "[812/812] espfoc_tuner_studio_target.bin: 0x49cb0 bytes",
    "Project build complete.",
    ">>> build OK: build/espfoc_tuner_studio_target.bin",
    "    Flash with: idf.py -p <PORT> flash",
]


def _scene_generate_build(w: MainWindow, _c: TunerClient) -> None:
    panel = getattr(w, "_generate", None)
    if panel is None:
        return
    panel._build_fw_box.setChecked(True)


def _stream_build_lines(w: MainWindow) -> int:
    """Append one fake build line to the panel's log. Returns the
    number of lines remaining so the caller can keep ticking until
    the queue drains."""
    panel = getattr(w, "_generate", None)
    if panel is None:
        return 0
    if not hasattr(_stream_build_lines, "_idx"):
        _stream_build_lines._idx = 0  # type: ignore[attr-defined]
    idx = _stream_build_lines._idx  # type: ignore[attr-defined]
    if idx < len(_FAKE_BUILD_LINES):
        panel._append(_FAKE_BUILD_LINES[idx])
        _stream_build_lines._idx = idx + 1  # type: ignore[attr-defined]
    return max(0, len(_FAKE_BUILD_LINES) - _stream_build_lines._idx)  # type: ignore[attr-defined]


def _grab_frame(window) -> QImage:
    return window.grab().toImage()


def _qimage_to_pillow(img: QImage):
    from PIL import Image
    img = img.convertToFormat(QImage.Format_RGBA8888)
    w, h = img.width(), img.height()
    ptr = img.constBits()
    raw = bytes(ptr)
    return Image.frombytes("RGBA", (w, h), raw)


def _resize_for_gif(im, max_side: int):
    from PIL import Image
    w, h = im.size
    if max(w, h) <= max_side:
        return im
    if w >= h:
        new_w = max_side
        new_h = int(h * (max_side / w))
    else:
        new_h = max_side
        new_w = int(w * (max_side / h))
    return im.resize((new_w, new_h), Image.LANCZOS)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", required=True, help="GIF output path")
    p.add_argument("--fps", type=int, default=4,
                   help="frames per second (lower = smaller GIF)")
    p.add_argument("--max-side", type=int, default=960,
                   help="resize the longer window edge to this many pixels")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=900)
    args = p.parse_args()

    try:
        from PIL import Image  # noqa: F401  (early failure if missing)
    except ImportError:
        print("error: Pillow not installed. pip install Pillow",
              file=sys.stderr)
        return 1

    # Boot the same plumbing as the smoke test.
    host_t, fw_t = LoopbackTransport.pair()
    fw = DemoFirmware(fw_t)
    fw.start()
    reader = LinkReader(host_t)
    reader.start()

    app = QApplication.instance() or QApplication(sys.argv)
    apply_dark_theme(app)
    client = TunerClient(reader)
    w = MainWindow(
        client,
        title="espFoC TunerStudio",
        link_mode="demo",
        link_descr="demo gif (DemoFirmware)",
    )
    w.resize(args.width, args.height)
    w.show()

    # Wait until the panels populate (gain readout, scope channels).
    deadline = time.monotonic() + 4.0
    while time.monotonic() < deadline:
        app.processEvents()
        if (w._tuning._kp_label.text().strip() not in ("-", "")
                and w._scope._n_channels > 0):
            break
        time.sleep(0.05)

    has_generate = hasattr(w, "_generate")
    scenes: List[Scene] = [
        ("Analysis",       0, _scene_analysis,        2.5),
        ("Scope",          2, _scene_scope,           2.5),
        ("SVM Hexagon",    3, _scene_svm_override,    4.0),
    ]
    if has_generate:
        scenes.append(("Generate App", 4, _scene_generate_idle, 1.5))
        scenes.append(("Generate App", 4, _scene_generate_build, 4.0))

    frames: List = []
    frame_dt = 1.0 / max(1, args.fps)

    for label, tab_idx, on_enter, dwell in scenes:
        w.findChild(type(w.centralWidget()))  # noqa: keeps name mangling happy
        # Switch tabs.
        tabs = _find_tabs(w)
        if tabs is not None and 0 <= tab_idx < tabs.count():
            tabs.setCurrentIndex(tab_idx)
        on_enter(w, client)
        # Let the panel react before snapping.
        for _ in range(3):
            app.processEvents()
            time.sleep(0.05)

        ticks = max(1, int(round(dwell / frame_dt)))
        for _ in range(ticks):
            # During the build scene, drip canned log lines.
            if label == "Generate App" and on_enter is _scene_generate_build:
                if _stream_build_lines(w) >= 0:
                    pass  # one line per frame, dries up naturally
            app.processEvents()
            time.sleep(frame_dt * 0.5)
            app.processEvents()
            img = _grab_frame(w)
            pil = _qimage_to_pillow(img).convert("RGB")
            pil = _resize_for_gif(pil, args.max_side)
            frames.append(pil)

    fw.stop()
    reader.stop()

    if not frames:
        print("error: captured zero frames", file=sys.stderr)
        return 2

    # Build one shared palette from the first frame and force every
    # other frame to use it. With a single global palette the inter-
    # frame deltas compress much better — README-sized GIFs go from
    # ~6 MB down to ~2 MB at the same dimensions / frame rate without
    # any visible quality loss on the dark theme.
    from PIL import Image
    palette_src = frames[0].quantize(colors=128, method=Image.MEDIANCUT)
    pal_frames = [palette_src] + [
        f.quantize(colors=128, method=Image.MEDIANCUT,
                   palette=palette_src, dither=Image.NONE)
        for f in frames[1:]
    ]

    duration_ms = int(round(1000.0 / max(1, args.fps)))
    out_path = os.path.abspath(args.output)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    pal_frames[0].save(
        out_path,
        save_all=True,
        append_images=pal_frames[1:],
        duration=duration_ms,
        loop=0,
        optimize=True,
        disposal=2,
    )
    size_kb = os.path.getsize(out_path) / 1024.0
    print(f"wrote {out_path}  ({len(frames)} frames, {size_kb:.0f} KB)")
    return 0


def _find_tabs(window):
    """Locate the QTabWidget the MainWindow built. Robust against the
    field name changing — searches for any QTabWidget in the central
    widget hierarchy."""
    from PySide6.QtWidgets import QTabWidget
    return window.findChild(QTabWidget)


if __name__ == "__main__":
    raise SystemExit(main())
