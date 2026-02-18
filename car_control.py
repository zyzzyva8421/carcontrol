#!/usr/bin/env python3
"""CLI control tool for a Raspberry Pi car over SSH."""

from __future__ import annotations

import argparse
import os
import select
import shlex
import subprocess
import sys
import termios
import time
import tty
from dataclasses import dataclass
from typing import Dict


DEFAULT_HOST = "192.168.2.21"
DEFAULT_USER = "pi"


@dataclass
class SSHConfig:
    host: str
    user: str
    port: int
    identity_file: str | None
    connect_timeout: int
    strict_host_key_checking: str


def build_ssh_command(config: SSHConfig, remote_command: str) -> list[str]:
    command = [
        "ssh",
        "-p",
        str(config.port),
        "-o",
        f"ConnectTimeout={config.connect_timeout}",
        "-o",
        f"StrictHostKeyChecking={config.strict_host_key_checking}",
    ]

    if config.identity_file:
        command.extend(["-i", os.path.expanduser(config.identity_file)])

    command.append(f"{config.user}@{config.host}")
    command.append(remote_command)
    return command


def run_remote_action(config: SSHConfig, template: str, action: str, dry_run: bool = False) -> int:
    remote_command = template.format(action=action)
    return run_remote_command(config, remote_command, dry_run=dry_run)


def run_remote_command(config: SSHConfig, remote_command: str, dry_run: bool = False) -> int:
    ssh_command = build_ssh_command(config, remote_command)

    if dry_run:
        printable = " ".join(shlex.quote(part) for part in ssh_command)
        print(printable)
        return 0

    process = subprocess.run(ssh_command)
    return process.returncode


def run_auto_mode(
    config: SSHConfig,
    remote_auto_path: str,
    python_bin: str,
    threshold_cm: float,
    speed: int,
    loop_seconds: float,
    dry_run: bool,
) -> int:
    remote_command = (
        f"{shlex.quote(python_bin)} {shlex.quote(remote_auto_path)} "
        f"--threshold-cm {threshold_cm} "
        f"--speed {speed} "
        f"--loop-seconds {loop_seconds}"
    )
    return run_remote_command(config, remote_command, dry_run=dry_run)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Control a Raspberry Pi car over SSH.",
    )

    parser.add_argument("--host", default=DEFAULT_HOST, help=f"Pi host (default: {DEFAULT_HOST})")
    parser.add_argument("--user", default=DEFAULT_USER, help=f"Pi user (default: {DEFAULT_USER})")
    parser.add_argument("--port", type=int, default=22, help="SSH port (default: 22)")
    parser.add_argument("--identity-file", help="Path to SSH private key")
    parser.add_argument("--connect-timeout", type=int, default=5, help="SSH connect timeout in seconds")
    parser.add_argument(
        "--strict-host-key-checking",
        choices=["yes", "no", "accept-new"],
        default="accept-new",
        help="SSH StrictHostKeyChecking option (default: accept-new)",
    )
    parser.add_argument(
        "--remote-template",
        default="python /home/pi/car_action.py {action}",
        help=(
            "Remote command template. Use {action} placeholder. "
            "Example: 'python /home/pi/car_action.py {action}'"
        ),
    )
    parser.add_argument("--dry-run", action="store_true", help="Print SSH command without running")

    subparsers = parser.add_subparsers(dest="mode", required=True)

    action_parser = subparsers.add_parser("action", help="Send a single action")
    action_parser.add_argument(
        "action",
        choices=["forward", "backward", "left", "right", "stop", "speed_up", "speed_down", "horn", "lights"],
        help="Action to send",
    )

    keyboard_parser = subparsers.add_parser("keyboard", help="Interactive keyboard control")
    keyboard_parser.add_argument(
        "--control-style",
        choices=["hold", "step"],
        default="hold",
        help="Keyboard behavior: hold for continuous move, or step for short pulses (default: hold)",
    )
    keyboard_parser.add_argument(
        "--repeat-stop-seconds",
        type=float,
        default=0.35,
        help="In hold mode, send stop if no key is pressed for this many seconds (default: 0.35)",
    )
    keyboard_parser.add_argument(
        "--step-seconds",
        type=float,
        default=0.25,
        help="In step mode, how long one movement key press lasts before auto-stop (default: 0.25)",
    )

    auto_parser = subparsers.add_parser("auto", help="Start automatic obstacle avoidance mode")
    auto_parser.add_argument(
        "--remote-auto-path",
        default="/home/pi/auto_avoid.py",
        help="Path to the auto-avoid script on Raspberry Pi (default: /home/pi/auto_avoid.py)",
    )
    auto_parser.add_argument(
        "--python-bin",
        default="python",
        help="Python executable on Raspberry Pi for auto mode (default: python)",
    )
    auto_parser.add_argument(
        "--threshold-cm",
        type=float,
        default=30.0,
        help="Obstacle distance threshold in cm (default: 30.0)",
    )
    auto_parser.add_argument(
        "--speed",
        type=int,
        default=45,
        help="Motor speed percentage in auto mode, 20-100 (default: 45)",
    )
    auto_parser.add_argument(
        "--loop-seconds",
        type=float,
        default=0.05,
        help="Control loop interval in seconds (default: 0.05)",
    )

    return parser


class RawTerminal:
    def __init__(self) -> None:
        self.fd = sys.stdin.fileno()
        self.original_settings = None

    def __enter__(self) -> "RawTerminal":
        self.original_settings = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        if self.original_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.original_settings)


def keyboard_mode(
    config: SSHConfig,
    template: str,
    control_style: str,
    repeat_stop_seconds: float,
    step_seconds: float,
    dry_run: bool,
) -> int:
    keymap: Dict[str, str] = {
        "w": "forward",
        "s": "backward",
        "a": "left",
        "d": "right",
        "x": "stop",
        "+": "speed_up",
        "-": "speed_down",
        "h": "horn",
        "l": "lights",
    }

    print("Keyboard mode started.")
    if control_style == "hold":
        print("Controls: hold w/a/s/d to move, x stop, +/- speed, h horn, l lights, q quit")
    else:
        print("Controls: w/a/s/d move one step, x stop, +/- speed, h horn, l lights, q quit")

    last_action = "stop"
    if run_remote_action(config, template, last_action, dry_run=dry_run) != 0:
        return 1

    with RawTerminal():
        tick = 0.05
        movement_actions = {"forward", "backward", "left", "right"}
        idle_seconds = 0.0

        while True:
            readable, _, _ = select.select([sys.stdin], [], [], tick)
            if readable:
                key = sys.stdin.read(1)
                if key == "q":
                    break

                action = keymap.get(key)
                if not action:
                    continue

                idle_seconds = 0.0

                if control_style == "hold":
                    if action != last_action:
                        code = run_remote_action(config, template, action, dry_run=dry_run)
                        if code != 0:
                            return code
                        last_action = action
                    continue

                code = run_remote_action(config, template, action, dry_run=dry_run)
                if code != 0:
                    return code

                if action in movement_actions and step_seconds > 0:
                    time.sleep(step_seconds)
                    code = run_remote_action(config, template, "stop", dry_run=dry_run)
                    if code != 0:
                        return code
                    last_action = "stop"
                else:
                    last_action = action
            else:
                if control_style == "hold":
                    idle_seconds += tick
                    if idle_seconds >= repeat_stop_seconds and last_action != "stop":
                        code = run_remote_action(config, template, "stop", dry_run=dry_run)
                        if code != 0:
                            return code
                        last_action = "stop"

    return run_remote_action(config, template, "stop", dry_run=dry_run)


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    config = SSHConfig(
        host=args.host,
        user=args.user,
        port=args.port,
        identity_file=args.identity_file,
        connect_timeout=args.connect_timeout,
        strict_host_key_checking=args.strict_host_key_checking,
    )

    if args.mode == "action":
        return run_remote_action(config, args.remote_template, args.action, dry_run=args.dry_run)

    if args.mode == "auto":
        return run_auto_mode(
            config=config,
            remote_auto_path=args.remote_auto_path,
            python_bin=args.python_bin,
            threshold_cm=args.threshold_cm,
            speed=args.speed,
            loop_seconds=args.loop_seconds,
            dry_run=args.dry_run,
        )

    return keyboard_mode(
        config=config,
        template=args.remote_template,
        control_style=args.control_style,
        repeat_stop_seconds=args.repeat_stop_seconds,
        step_seconds=args.step_seconds,
        dry_run=args.dry_run,
    )


if __name__ == "__main__":
    raise SystemExit(main())
