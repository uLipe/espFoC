#!/usr/bin/env python3
"""tunerctl — talk to the espFoC tuner backend over a serial bus.

Examples:

    # Inspect the axis state
    tunerctl --port /dev/ttyACM0 axis-state

    # Read current PI gains
    tunerctl --port /dev/ttyACM0 read

    # Recompute gains via MPZ for a motor on the fly
    tunerctl --port /dev/ttyACM0 retune --r 1.08 --l 0.0018 --bw 150

    # Engage tuner override and command iq = 1.5 A
    tunerctl --port /dev/ttyACM0 override on
    tunerctl --port /dev/ttyACM0 set-target iq 1.5
    tunerctl --port /dev/ttyACM0 override off
"""

from __future__ import annotations

import argparse
import sys
from typing import Optional

from ..link.transport_serial import SerialTransport
from ..protocol import (
    AxisStateFlag,
    TunerClient,
    TunerError,
)


def _format_state(s: AxisStateFlag) -> str:
    pretty = []
    for flag in (AxisStateFlag.INITIALIZED,
                 AxisStateFlag.ALIGNED,
                 AxisStateFlag.RUNNING,
                 AxisStateFlag.TUNER_OVERRIDE):
        pretty.append(f"{'+' if s & flag else '-'}{flag.name}")
    return " ".join(pretty)


def _make_client(args) -> TunerClient:
    transport = SerialTransport(port=args.port, baud=args.baud)
    return TunerClient(transport, axis=args.axis)


def cmd_axis_state(args) -> int:
    cli = _make_client(args)
    state = cli.read_axis_state()
    err = cli.read_last_error()
    print(f"axis {args.axis}: state={int(state):#04x} ({_format_state(state)}) "
          f"last_err={err}")
    return 0


def cmd_read(args) -> int:
    cli = _make_client(args)
    print(f"axis {args.axis}: "
          f"Kp={cli.read_kp():.4f} V/A  "
          f"Ki={cli.read_ki():.2f} V/(A*s)  "
          f"ILim={cli.read_int_lim():.3f} V  "
          f"Vmax={cli.read_v_max():.3f} V")
    return 0


def cmd_write(args) -> int:
    cli = _make_client(args)
    if args.kp is not None:
        cli.write_kp(args.kp)
    if args.ki is not None:
        cli.write_ki(args.ki)
    if args.ilim is not None:
        cli.write_int_lim(args.ilim)
    return cmd_read(args)


def cmd_retune(args) -> int:
    cli = _make_client(args)
    cli.recompute_gains(args.r, args.l, args.bw)
    return cmd_read(args)


def cmd_override(args) -> int:
    cli = _make_client(args)
    if args.action == "on":
        cli.override_on()
    else:
        cli.override_off()
    return cmd_axis_state(args)


def cmd_set_target(args) -> int:
    cli = _make_client(args)
    setter = {
        "id": cli.write_target_id,
        "iq": cli.write_target_iq,
        "ud": cli.write_target_ud,
        "uq": cli.write_target_uq,
    }[args.target]
    setter(args.value)
    print(f"axis {args.axis}: target_{args.target}={args.value}")
    return 0


def cmd_align(args) -> int:
    cli = _make_client(args)
    print("aligning... (blocking on firmware)")
    cli.align_axis(timeout=args.timeout)
    state = cli.read_axis_state()
    print(f"alignment complete: {_format_state(state)}")
    return 0


def cmd_persist(args) -> int:
    cli = _make_client(args)
    cli.persist_calibration(motor_r=args.r or 0.0,
                            motor_l=args.l or 0.0,
                            bandwidth_hz=args.bw or 0.0)
    print(f"axis {args.axis}: calibration saved to NVS")
    return 0


def cmd_load(args) -> int:
    cli = _make_client(args)
    cli.load_calibration()
    return cmd_read(args)


def cmd_erase(args) -> int:
    cli = _make_client(args)
    cli.erase_calibration()
    print("calibration namespace erased")
    return 0


def cmd_firmware_type(args) -> int:
    cli = _make_client(args)
    fw = cli.read_firmware_type()
    fourcc = fw.to_bytes(4, "little").decode("ascii", errors="replace")
    print(f"firmware_type = 0x{fw:08x}  (\"{fourcc}\")")
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="tunerctl", description=__doc__)
    p.add_argument("--port", required=True, help="serial port (e.g. /dev/ttyACM0)")
    p.add_argument("--baud", type=int, default=921600, help="baud rate")
    p.add_argument("--axis", type=int, default=0, help="axis id (0..3)")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("axis-state", help="show axis state flags").set_defaults(func=cmd_axis_state)
    sub.add_parser("read", help="read current PI gains + Vmax").set_defaults(func=cmd_read)

    pw = sub.add_parser("write", help="manually override one or more gains")
    pw.add_argument("--kp", type=float)
    pw.add_argument("--ki", type=float)
    pw.add_argument("--ilim", type=float)
    pw.set_defaults(func=cmd_write)

    pr = sub.add_parser("retune", help="recompute MPZ gains from motor params")
    pr.add_argument("--r", type=float, required=True, help="phase resistance [ohm]")
    pr.add_argument("--l", type=float, required=True, help="phase inductance [H]")
    pr.add_argument("--bw", type=float, required=True, help="target bandwidth [Hz]")
    pr.set_defaults(func=cmd_retune)

    po = sub.add_parser("override", help="engage / release tuner motion control")
    po.add_argument("action", choices=["on", "off"])
    po.set_defaults(func=cmd_override)

    ps = sub.add_parser("set-target", help="set a motion target (override must be ON)")
    ps.add_argument("target", choices=["id", "iq", "ud", "uq"])
    ps.add_argument("value", type=float)
    ps.set_defaults(func=cmd_set_target)

    pa = sub.add_parser("align", help="run the alignment routine on the firmware")
    pa.add_argument("--timeout", type=float, default=8.0,
                    help="seconds to wait for alignment to complete")
    pa.set_defaults(func=cmd_align)

    pp = sub.add_parser("persist", help="save current gains to NVS")
    pp.add_argument("--r",  type=float, help="phase resistance recorded with the blob [ohm]")
    pp.add_argument("--l",  type=float, help="phase inductance recorded with the blob [H]")
    pp.add_argument("--bw", type=float, help="bandwidth recorded with the blob [Hz]")
    pp.set_defaults(func=cmd_persist)

    sub.add_parser("load", help="apply NVS overlay live").set_defaults(func=cmd_load)
    sub.add_parser("erase", help="erase calibration namespace").set_defaults(func=cmd_erase)
    sub.add_parser("firmware-type",
                   help="show the firmware-type magic the target reports"
                  ).set_defaults(func=cmd_firmware_type)

    return p


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except TunerError as e:
        print(f"tunerctl: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
