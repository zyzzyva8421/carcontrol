#!/bin/bash
set -e
# Source first available ROS2 setup.bash if present
for s in /opt/ros/*/setup.bash; do
  if [ -f "$s" ]; then
    . "$s"
    break
  fi
done
exec /usr/bin/python3 /home/pi/ros2_server.py
