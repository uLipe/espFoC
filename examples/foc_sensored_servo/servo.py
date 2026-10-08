#!/usr/bin/env python3
"""Command line for the foc_sensored_servo example.

Sends position commands to the servo over its console UART and prints what
it answers. Trajectories (sine, steps) are generated here as waypoint lists;
the firmware turns each waypoint into a smooth minimum-jerk segment.

  servo.py -p /dev/ttyUSB0 wait                  boot log until the servo is ready
  servo.py -p /dev/ttyUSB0 move 90               go to 90 deg
  servo.py -p /dev/ttyUSB0 traj 90:2 -90:4 0:2   waypoints <deg>:<seconds>
  servo.py -p /dev/ttyUSB0 sine -a 45 -T 4 -n 3  three 4 s cycles of +-45 deg
  servo.py -p /dev/ttyUSB0 steps 90 0 -90 0      step through angles, dwell between
  servo.py -p /dev/ttyUSB0 where                 current angle
  servo.py -p /dev/ttyUSB0 stop                  stop the servo, bridge off
  servo.py -p /dev/ttyUSB0 shell                 type commands interactively

Angles are mechanical degrees from the origin the servo set at start-up.
Needs pyserial (shipped with ESP-IDF's Python environment).
"""

import argparse
import os
import re
import sys
import time

import serial

# The firmware takes up to this many waypoints in one traj command.
MAX_WAYPOINTS = 32

# Commissioning (phase discovery, identification, cogging learn) takes
# 3 to 4 minutes.
BOOT_TIMEOUT_S = 600.0
# Margin over a command's own duration for the settle and the reply.
REPLY_MARGIN_S = 15.0

WHERE_REPLY = re.compile(r"^at -?\d+\.\d+ deg$")


def open_console(port, baud):
    # ESP32 boards reset when RTS is asserted alone (two-transistor auto-reset
    # on EN). Opening with both DTR and RTS asserted, pyserial's default,
    # leaves EN alone; changing one line before the other would reset the
    # chip and restart commissioning.
    return serial.Serial(port, baud, timeout=0.2)


def reset_board(console):
    console.dtr = False
    console.rts = True
    time.sleep(0.1)
    console.rts = False


def read_line(console):
    raw = console.readline()
    if not raw:
        return None
    return raw.decode("utf-8", errors="replace").rstrip("\r\n")


def wait_for(console, is_last_line, timeout_s):
    """Prints console lines until is_last_line(line) is true.

    Returns that line, or None on timeout.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = read_line(console)
        if line is None:
            continue
        print(line, flush=True)
        if "Controller tripped" in line:
            return line
        if is_last_line(line):
            return line
    return None


def send(console, command, is_last_line, timeout_s):
    console.reset_input_buffer()
    console.write((command + "\n").encode("ascii"))
    console.flush()
    last = wait_for(console, is_last_line, timeout_s)
    if last is None:
        print("servo.py: no answer to '%s' in %.0f s" % (command, timeout_s),
              file=sys.stderr)
        return False
    return last.startswith(("stopped at", "Done:")) or bool(WHERE_REPLY.match(last))


def end_of_move(line):
    return line.startswith(("stopped at", "rejected:"))


def run_waypoints(console, waypoints):
    if len(waypoints) > MAX_WAYPOINTS:
        print("servo.py: %d waypoints, the servo takes at most %d"
              % (len(waypoints), MAX_WAYPOINTS), file=sys.stderr)
        return False
    command = "traj " + " ".join("%.3f:%.3f" % (deg, sec) for deg, sec in waypoints)
    total_s = sum(sec for _, sec in waypoints)
    return send(console, command, end_of_move, total_s + REPLY_MARGIN_S)


def parse_waypoint(text):
    try:
        deg, sec = text.split(":")
        return float(deg), float(sec)
    except ValueError:
        raise argparse.ArgumentTypeError("'%s' is not <deg>:<seconds>" % text)


def sine_waypoints(amplitude, period, cycles, center):
    # Minimum-jerk segments between the peaks rest at each peak, as a sine
    # does, and peak at the same speed: 7.5 * amplitude / period.
    quarter = period / 4.0
    half = period / 2.0
    waypoints = [(center + amplitude, quarter)]
    for swing in range(2 * cycles - 1):
        sign = -1.0 if swing % 2 == 0 else 1.0
        waypoints.append((center + sign * amplitude, half))
    waypoints.append((center, quarter))
    return waypoints


def step_waypoints(targets, move_time, dwell):
    waypoints = []
    for target in targets:
        waypoints.append((target, move_time))
        if dwell > 0.0:
            # Same angle again: the servo holds it for this long.
            waypoints.append((target, dwell))
    return waypoints


def shell(console):
    print("servo shell: move <deg> | traj <deg>:<s> ... | where | stop | quit")
    while True:
        try:
            line = input("servo> ").strip()
        except EOFError:
            return True
        if line in ("", "quit", "exit"):
            if line:
                return True
            continue
        if line == "stop":
            return send(console, line, lambda l: l.startswith("Done:"), REPLY_MARGIN_S)
        if line == "where":
            send(console, line, lambda l: bool(WHERE_REPLY.match(l)), REPLY_MARGIN_S)
            continue
        # A long trajectory can take minutes; the reply is what ends the wait.
        send(console, line, end_of_move, BOOT_TIMEOUT_S)


def main():
    parser = argparse.ArgumentParser(
        description="Command the foc_sensored_servo example over its console.")
    parser.add_argument("-p", "--port", default=os.environ.get("ESPPORT", "/dev/ttyUSB0"),
                        help="console serial port (default: $ESPPORT or /dev/ttyUSB0)")
    parser.add_argument("-b", "--baud", type=int, default=115200)
    commands = parser.add_subparsers(dest="command", required=True)

    wait = commands.add_parser("wait", help="print the boot log until the servo is ready")
    wait.add_argument("--reset", action="store_true",
                      help="reset the board first (commissioning runs again)")

    move = commands.add_parser("move", help="go to an angle")
    move.add_argument("degrees", type=float)

    traj = commands.add_parser("traj", help="run waypoints <deg>:<seconds>")
    traj.add_argument("waypoints", type=parse_waypoint, nargs="+")

    sine = commands.add_parser("sine", help="swing around a center angle")
    sine.add_argument("-a", "--amplitude", type=float, required=True, help="degrees")
    sine.add_argument("-T", "--period", type=float, required=True, help="seconds per cycle")
    sine.add_argument("-n", "--cycles", type=int, default=1)
    sine.add_argument("-c", "--center", type=float, default=0.0, help="degrees")

    steps = commands.add_parser("steps", help="step through angles")
    steps.add_argument("targets", type=float, nargs="+", help="degrees")
    steps.add_argument("--move-time", type=float, default=2.5, help="seconds per step")
    steps.add_argument("--dwell", type=float, default=1.0, help="seconds held at each angle")

    commands.add_parser("where", help="print the current angle")
    commands.add_parser("stop", help="stop the servo, bridge off")
    commands.add_parser("shell", help="type commands interactively")

    argv = sys.argv[1:]
    # argparse reads "-45:3" as an option; everything after "traj" is a waypoint.
    if "traj" in argv and "--" not in argv:
        argv.insert(argv.index("traj") + 1, "--")
    args = parser.parse_args(argv)
    console = open_console(args.port, args.baud)
    try:
        if args.command == "wait":
            if args.reset:
                reset_board(console)
            ok = wait_for(console, lambda l: l == "ready", BOOT_TIMEOUT_S) == "ready"
        elif args.command == "move":
            # The firmware picks the duration; its reply ends the wait.
            ok = send(console, "move %.3f" % args.degrees, end_of_move, BOOT_TIMEOUT_S)
        elif args.command == "traj":
            ok = run_waypoints(console, args.waypoints)
        elif args.command == "sine":
            ok = run_waypoints(console, sine_waypoints(args.amplitude, args.period,
                                                       args.cycles, args.center))
        elif args.command == "steps":
            ok = run_waypoints(console, step_waypoints(args.targets, args.move_time,
                                                       args.dwell))
        elif args.command == "where":
            ok = send(console, "where", lambda l: bool(WHERE_REPLY.match(l)), REPLY_MARGIN_S)
        elif args.command == "stop":
            ok = send(console, "stop", lambda l: l.startswith("Done:"), REPLY_MARGIN_S)
        else:
            ok = shell(console)
    finally:
        console.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
