#!/usr/bin/env python3
"""ROS2 subscriber that executes servo commands received on /servo_control.

Usage:
  ros2 run or ./ros2_server.py (requires rclpy installed on the Pi and ROS2 setup)
"""
from __future__ import annotations

import json
import shlex
import subprocess
import sys

try:
    import rclpy
    from std_msgs.msg import String
except Exception:
    rclpy = None


def handle_message(msg: String) -> None:
    try:
        data = json.loads(msg.data)
    except Exception:
        print('invalid json', file=sys.stderr)
        return

    if data.get('type') != 'servo':
        print('unsupported type', file=sys.stderr)
        return

    servo = int(data.get('servo', 3))
    angle = float(data.get('angle', 90))
    pin = int(data.get('pin', 21))
    backend = data.get('backend', 'pigpio')

    cmd = ['python3', '/home/pi/camera_servo.py', '--servo', str(servo), '--pin', str(pin), '--angle', str(angle), '--backend', backend]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    print('cmd', ' '.join(shlex.quote(p) for p in cmd), 'rc', proc.returncode)
    if proc.stdout:
        print(proc.stdout)
    if proc.stderr:
        print(proc.stderr, file=sys.stderr)


def main() -> None:
    if rclpy is None:
        print('rclpy not available; install ROS2 and rclpy', file=sys.stderr)
        sys.exit(2)

    rclpy.init()
    node = rclpy.create_node('servo_control_server')
    sub = node.create_subscription(String, '/servo_control', handle_message, 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
