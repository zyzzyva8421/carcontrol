#!/usr/bin/env python3
"""WebSocket control server for Raspberry Pi car.

Usage:
  python3 ws_control_server.py --host 0.0.0.0 --port 8765 --token secret-token

The server accepts JSON messages with shape:
  {"type": "servo", "servo": 3, "angle": 80}

Only `type=="servo"` is accepted by default (safer). The server runs
`/home/pi/camera_servo.py` locally to perform the action and returns a JSON
response with `code`, `stdout`, `stderr`.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import shlex
import subprocess
from typing import Any, Dict

import websockets

DEFAULT_SERVO_PINS = {
    1: 16,
    2: 23,
    3: 21,
}


async def handle(ws: websockets.WebSocketServerProtocol, path: str, token: str) -> None:
    # Basic token check via query string: ws://host:port/?token=...
    q = ws.request_headers.get("Sec-WebSocket-Protocol")
    # We also accept token in first message for simplicity
    try:
        async for msg in ws:
            try:
                data = json.loads(msg)
            except Exception:
                await ws.send(json.dumps({"error": "invalid json"}))
                continue

            # token check (optional)
            if token:
                provided = data.get("token") or None
                if provided != token:
                    await ws.send(json.dumps({"error": "unauthorized"}))
                    continue

            if data.get("type") == "servo":
                servo = int(data.get("servo", 3))
                angle = float(data.get("angle", 90))
                pin = int(data.get("pin", DEFAULT_SERVO_PINS.get(servo, 21)))
                backend = data.get("backend", "pigpio")
                hold_ms = int(data.get("hold_ms", 300))

                cmd = [
                    "python3",
                    "/home/pi/camera_servo.py",
                    "--servo",
                    str(servo),
                    "--pin",
                    str(pin),
                    "--angle",
                    str(angle),
                    "--backend",
                    backend,
                    "--hold-ms",
                    str(hold_ms),
                ]

                proc = subprocess.run(cmd, capture_output=True, text=True)
                resp: Dict[str, Any] = {
                    "cmd": " ".join(shlex.quote(p) for p in cmd),
                    "returncode": proc.returncode,
                    "stdout": proc.stdout,
                    "stderr": proc.stderr,
                }
                await ws.send(json.dumps(resp))
                continue

            # Unknown message
            await ws.send(json.dumps({"error": "unsupported command type"}))
    except websockets.exceptions.ConnectionClosed:
        return


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--token", default="", help="Simple auth token (optional)")
    args = parser.parse_args()

    async def run() -> None:
        async with websockets.serve(lambda ws, path: handle(ws, path, args.token), args.host, args.port):
            print("ws control server listening on {}:{}".format(args.host, args.port))
            await asyncio.Future()  # run forever

    asyncio.run(run())


if __name__ == "__main__":
    main()
