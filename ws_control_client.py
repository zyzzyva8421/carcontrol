#!/usr/bin/env python3
"""Simple WebSocket client to control Pi servos remotely.

Usage examples:
  python3 ws_control_client.py --host 192.168.2.21 --servo 3 --angle 80 --token secret-token

Sends a JSON message and prints the JSON response from the server.
"""

from __future__ import annotations

import argparse
import asyncio
import json

import websockets


async def send_servo(host: str, port: int, token: str, servo: int, angle: float, pin: int | None, backend: str) -> None:
    uri = f"ws://{host}:{port}"
    async with websockets.connect(uri) as ws:
        msg = {"type": "servo", "servo": servo, "angle": angle}
        if pin is not None:
            msg["pin"] = pin
        if backend:
            msg["backend"] = backend
        if token:
            msg["token"] = token

        await ws.send(json.dumps(msg))
        resp = await ws.recv()
        print(resp)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--token", default="")
    parser.add_argument("--servo", type=int, default=3)
    parser.add_argument("--angle", type=float, required=True)
    parser.add_argument("--pin", type=int, default=None)
    parser.add_argument("--backend", default="pigpio")
    args = parser.parse_args()

    asyncio.run(send_servo(args.host, args.port, args.token, args.servo, args.angle, args.pin, args.backend))


if __name__ == "__main__":
    main()
