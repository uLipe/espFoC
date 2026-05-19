#!/usr/bin/env python3
"""tunerctl — interactive espFoC tuner client (protocol v2).

Opens one serial session, runs connect, then accepts subcommands on stdin.
One-shot mode: ``tunerctl --port DEV align`` → connect → align → disconnect.
"""

from __future__ import annotations

import argparse
import shlex
import sys
from typing import Callable, Optional

from ..link.transport_serial import SerialTransport
from ..protocol import (
    AxisStateFlag,
    ConnectInfo,
    TunerClient,
    TunerError,
)


def _format_state(s: AxisStateFlag) -> str:
    parts = []
    for flag in (AxisStateFlag.INITIALIZED,
                 AxisStateFlag.ALIGNING,
                 AxisStateFlag.ALIGNED,
                 AxisStateFlag.RUNNING):
        parts.append(f"{'+' if s & flag else '-'}{flag.name}")
    return " ".join(parts)


def _format_hb(cli: TunerClient) -> str:
    return _format_state(AxisStateFlag(cli.heartbeat_state))


class TunerShell:
    def __init__(self, cli: TunerClient) -> None:
        self.cli = cli
        self._handlers: dict[str, Callable[[list[str]], int]] = {
            "help": self._cmd_help,
            "connect": self._cmd_connect,
            "disconnect": self._cmd_disconnect,
            "status": self._cmd_status,
            "read": self._cmd_read,
            "write": self._cmd_write,
            "align": self._cmd_align,
            "run": self._cmd_run,
            "stop": self._cmd_stop,
            "store": self._cmd_store,
            "erase": self._cmd_erase,
            "set-target": self._cmd_set_target,
            "scope-start": self._cmd_scope_start,
            "scope-stop": self._cmd_scope_stop,
            "cutoff": self._cmd_cutoff,
            "firmware-type": self._cmd_firmware_type,
            "quit": self._cmd_quit,
            "exit": self._cmd_quit,
        }

    def _cmd_help(self, _args: list[str]) -> int:
        print("commands: connect disconnect status read write align run stop "
              "store erase set-target scope-start scope-stop cutoff "
              "firmware-type quit")
        return 0

    def _cmd_connect(self, _args: list[str]) -> int:
        info = self.cli.connect()
        fourcc = info.firmware_type.to_bytes(4, "little").decode(
            "ascii", errors="replace")
        print(f"connected proto={info.firmware_type:#010x} ({fourcc}) "
              f"axes={info.num_axes} scope_ch={info.scope_channels} "
              f"hb_ms={info.heartbeat_period_ms}")
        return 0

    def _cmd_disconnect(self, _args: list[str]) -> int:
        self.cli.disconnect()
        print("disconnected")
        return 0

    def _cmd_status(self, _args: list[str]) -> int:
        if not self.cli.connected:
            print("not connected")
            return 1
        print(f"heartbeat: {_format_hb(self.cli)} last_err={self.cli.heartbeat_last_err}")
        try:
            st = self.cli.read_axis_state()
            print(f"poll:      {_format_state(st)}")
        except TunerError as e:
            print(f"poll:      (read failed: {e})")
        return 0

    def _cmd_read(self, _args: list[str]) -> int:
        print(f"Kp={self.cli.read_kp():.4f} Ki={self.cli.read_ki():.2f} "
              f"Kd={self.cli.read_kd():.6f} Kff={self.cli.read_kff():.4f} "
              f"ILim={self.cli.read_int_lim():.3f} Vmax={self.cli.read_v_max():.3f}")
        return 0

    def _cmd_write(self, args: list[str]) -> int:
        p = argparse.ArgumentParser(prog="write")
        p.add_argument("--kp", type=float)
        p.add_argument("--ki", type=float)
        p.add_argument("--kd", type=float)
        p.add_argument("--kff", type=float)
        p.add_argument("--ilim", type=float)
        w = p.parse_args(args)
        if w.kp is not None:
            self.cli.write_kp(w.kp)
        if w.ki is not None:
            self.cli.write_ki(w.ki)
        if w.kd is not None:
            self.cli.write_kd(w.kd)
        if w.kff is not None:
            self.cli.write_kff(w.kff)
        if w.ilim is not None:
            self.cli.write_int_lim(w.ilim)
        return self._cmd_read([])

    def _cmd_align(self, args: list[str]) -> int:
        p = argparse.ArgumentParser(prog="align")
        p.add_argument("--timeout", type=float, default=8.0)
        a = p.parse_args(args)
        print("aligning...")
        self.cli.align_axis(timeout=a.timeout)
        print(f"done: {_format_hb(self.cli)}")
        return 0

    def _cmd_run(self, _args: list[str]) -> int:
        self.cli.run_axis()
        print(f"run: {_format_hb(self.cli)}")
        return 0

    def _cmd_stop(self, _args: list[str]) -> int:
        self.cli.stop_axis()
        print(f"stop: {_format_hb(self.cli)}")
        return 0

    def _cmd_store(self, _args: list[str]) -> int:
        self.cli.store_calibration()
        print("stored")
        return 0

    def _cmd_erase(self, _args: list[str]) -> int:
        self.cli.erase_calibration()
        print("erased")
        return 0

    def _cmd_set_target(self, args: list[str]) -> int:
        p = argparse.ArgumentParser(prog="set-target")
        p.add_argument("target", choices=["id", "iq"])
        p.add_argument("value", type=float)
        t = p.parse_args(args)
        if t.target == "id":
            self.cli.write_target_id(t.value)
        else:
            self.cli.write_target_iq(t.value)
        print(f"target_{t.target}={t.value}")
        return 0

    def _cmd_scope_start(self, _args: list[str]) -> int:
        self.cli.scope_start()
        print("scope streaming on")
        return 0

    def _cmd_scope_stop(self, _args: list[str]) -> int:
        self.cli.scope_stop()
        print("scope streaming off")
        return 0

    def _cmd_cutoff(self, args: list[str]) -> int:
        p = argparse.ArgumentParser(prog="cutoff")
        p.add_argument("--set", type=float, default=None)
        c = p.parse_args(args)
        if c.set is not None:
            self.cli.write_current_filter_fc(c.set)
            print(f"cutoff set to {c.set:.1f} Hz")
            return 0
        print(f"cutoff = {self.cli.read_current_filter_fc():.1f} Hz")
        return 0

    def _cmd_firmware_type(self, _args: list[str]) -> int:
        fw = self.cli.read_firmware_type()
        fourcc = fw.to_bytes(4, "little").decode("ascii", errors="replace")
        print(f"firmware_type = 0x{fw:08x}  (\"{fourcc}\")")
        return 0

    def _cmd_quit(self, _args: list[str]) -> int:
        return 130

    def run_line(self, line: str) -> int:
        line = line.strip()
        if not line:
            return 0
        try:
            parts = shlex.split(line)
        except ValueError as e:
            print(f"parse error: {e}", file=sys.stderr)
            return 1
        cmd = parts[0].lower()
        handler = self._handlers.get(cmd)
        if handler is None:
            print(f"unknown command: {cmd}", file=sys.stderr)
            return 1
        return handler(parts[1:])

    def repl(self) -> int:
        print("tunerctl interactive — type 'help' or 'quit'")
        while True:
            try:
                line = input("espfoc> ")
            except (EOFError, KeyboardInterrupt):
                print()
                break
            code = self.run_line(line)
            if code == 130:
                break
        return 0


def _one_shot(cli: TunerClient, cmd: str, cmd_args: list[str]) -> int:
    shell = TunerShell(cli)
    try:
        cli.connect()
    except TunerError as e:
        print(f"tunerctl: connect failed: {e}", file=sys.stderr)
        return 1
    try:
        return shell.run_line(" ".join([cmd] + cmd_args))
    finally:
        try:
            cli.disconnect()
        except TunerError:
            pass


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="tunerctl")
    p.add_argument("--port", help="serial port (required except -h)")
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--axis", type=int, default=0)
    p.add_argument("-i", "--interactive", action="store_true",
                   help="stay open for REPL commands")
    p.add_argument("command", nargs="*", help="one-shot subcommand")
    return p


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if not args.port:
        build_parser().print_help()
        return 2

    transport = SerialTransport(port=args.port, baud=args.baud)
    cli = TunerClient(transport, axis=args.axis)
    shell = TunerShell(cli)

    try:
        if args.interactive or not args.command:
            try:
                shell._cmd_connect([])
            except TunerError as e:
                print(f"tunerctl: connect failed: {e}", file=sys.stderr)
                return 1
            try:
                return shell.repl()
            finally:
                try:
                    cli.disconnect()
                except TunerError:
                    pass
        return _one_shot(cli, args.command[0], args.command[1:])
    except TunerError as e:
        print(f"tunerctl: {e}", file=sys.stderr)
        return 1
    finally:
        cli.close()


if __name__ == "__main__":
    sys.exit(main())
