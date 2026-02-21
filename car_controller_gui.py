#!/usr/bin/env python3
"""Simple GUI controller for Raspberry Pi car over SSH."""

from __future__ import annotations

import re
import threading
import tkinter as tk
from tkinter import ttk
import shlex
import io
import urllib.request
import time
import asyncio
import json
try:
    import websockets
except Exception:  # pragma: no cover
    websockets = None
import subprocess

try:
    from PIL import Image, ImageTk
except Exception:  # pragma: no cover
    Image = None
    ImageTk = None

from car_control import DEFAULT_HOST, DEFAULT_USER, SSHConfig, run_remote_command


class CarControllerGUI:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Raspberry Car Controller")
        self.root.resizable(False, False)

        self.host_var = tk.StringVar(value=DEFAULT_HOST)
        self.user_var = tk.StringVar(value=DEFAULT_USER)
        self.port_var = tk.IntVar(value=22)
        self.template_var = tk.StringVar(value="python /home/pi/car_action.py {action}")
        self.auto_path_var = tk.StringVar(value="/home/pi/auto_avoid.py")
        self.auto_threshold_var = tk.DoubleVar(value=30.0)
        self.control_style_var = tk.StringVar(value="hold")
        self.step_seconds_var = tk.DoubleVar(value=0.25)
        self.status_var = tk.StringVar(value="Ready")
        self.direction_var = tk.StringVar(value="Stopped")
        self.speed_var = tk.IntVar(value=45)
        self.mode_var = tk.StringVar(value="Manual")
        self.ir_left_var = tk.StringVar(value="Unknown")
        self.ir_right_var = tk.StringVar(value="Unknown")
        self.ir_left_raw_var = tk.StringVar(value="-")
        self.ir_right_raw_var = tk.StringVar(value="-")
        self.ir_auto_refresh_var = tk.BooleanVar(value=True)
        self.ir_swap_var = tk.BooleanVar(value=False)
        self._ir_read_in_progress = False
        self.ultra_distance_var = tk.StringVar(value="-")
        self.ultra_status_var = tk.StringVar(value="Unknown")
        self.ultra_auto_refresh_var = tk.BooleanVar(value=True)
        self._ultra_read_in_progress = False
        self.camera_angle_var = tk.IntVar(value=90)
        self.camera_angle2_var = tk.IntVar(value=90)
        self.camera_angle3_var = tk.IntVar(value=90)
        self.camera_step_var = tk.IntVar(value=10)
        self.camera_servo1_pin_var = tk.IntVar(value=23)  # J1: IO23
        self.camera_servo2_pin_var = tk.IntVar(value=11)  # J2: IO11 (SCLK)
        self.camera_servo3_pin_var = tk.IntVar(value=9)   # J3: IO9 (MISO)
        self.camera_backend_var = tk.StringVar(value="pigpio")
        self.camera_center_j1_var = tk.IntVar(value=0)
        self.camera_center_j3_var = tk.IntVar(value=0)
        # WebSocket control settings (optional): if enabled, GUI will send
        # servo commands directly to the Pi's websocket server instead of
        # using SSH/remote commands.
        self.ws_enabled_var = tk.BooleanVar(value=True)
        self.ws_host_var = tk.StringVar(value=DEFAULT_HOST)
        self.ws_port_var = tk.IntVar(value=8765)
        self.ws_token_var = tk.StringVar(value="")
        # ROS2 option: publish servo commands to /servo_control (std_msgs/String)
        self.ros_enabled_var = tk.BooleanVar(value=False)
        self.camera_url_var = tk.StringVar(value=f"http://{DEFAULT_HOST}:8080/?action=snapshot")
        self.camera_preview_on_var = tk.BooleanVar(value=False)
        self._camera_preview_image = None
        self._camera_fetch_in_progress = False

        self.pressed_keys: set[str] = set()

        # Real-time status tab variables (must be before _build_ui)
        self.rt_direction = tk.StringVar(value="-")
        self.rt_speed = tk.StringVar(value="-")
        self.rt_left_blocked = tk.StringVar(value="-")
        self.rt_right_blocked = tk.StringVar(value="-")
        self.rt_distance = tk.StringVar(value="-")
        self.rt_too_close = tk.StringVar(value="-")
        self.rt_obstacle = tk.StringVar(value="-")

        self._build_ui()
        # When websocket is enabled, try to fetch the token from the Pi
        # so the GUI is pre-filled with the correct secret. This uses the
        # same SSH settings the GUI already holds.
        self.ws_enabled_var.trace_add("write", lambda *a: self._on_ws_enabled_changed())
        self._bind_keys()
        self._schedule_sensor_refresh()

        self._start_status_ws_client()

    def _on_ws_enabled_changed(self) -> None:
        if not self.ws_enabled_var.get():
            return
        # fetch token from remote Pi if available
        threading.Thread(target=self._fetch_ws_token_from_pi, daemon=True).start()

    def _fetch_ws_token_from_pi(self) -> None:
        config = self._current_config()
        ssh_cmd = [
            "ssh",
            "-p",
            str(config.port),
            "-o",
            f"ConnectTimeout={config.connect_timeout}",
            "-o",
            f"StrictHostKeyChecking={config.strict_host_key_checking}",
        ]
        if config.identity_file:
            ssh_cmd.extend(["-i", config.identity_file])
        ssh_cmd.append(f"{config.user}@{config.host}")
        ssh_cmd.append("cat /home/pi/ws_token.txt || true")

        try:
            proc = subprocess.run(ssh_cmd, capture_output=True, text=True, timeout=6)
            if proc.returncode == 0 and proc.stdout:
                token = proc.stdout.strip()
                self.root.after(0, lambda: self.ws_token_var.set(token))
                self.root.after(0, lambda: self.status_var.set("Fetched WS token from Pi"))
            else:
                self.root.after(0, lambda: self.status_var.set("WS token not found on Pi"))
        except Exception:
            self.root.after(0, lambda: self.status_var.set("Failed to fetch WS token"))

    def _build_ui(self) -> None:
        root_frame = ttk.Frame(self.root, padding=8)
        root_frame.grid(row=0, column=0, sticky="nsew")

        config_frame = ttk.LabelFrame(root_frame, text="Connection", padding=8)
        config_frame.grid(row=0, column=0, sticky="ew")

        ttk.Label(config_frame, text="Host").grid(row=0, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Entry(config_frame, textvariable=self.host_var, width=20).grid(row=0, column=1, sticky="ew", pady=4)

        ttk.Label(config_frame, text="User").grid(row=0, column=2, sticky="w", padx=(12, 8), pady=4)
        ttk.Entry(config_frame, textvariable=self.user_var, width=10).grid(row=0, column=3, sticky="ew", pady=4)

        ttk.Label(config_frame, text="Port").grid(row=0, column=4, sticky="w", padx=(12, 8), pady=4)
        ttk.Entry(config_frame, textvariable=self.port_var, width=6).grid(row=0, column=5, sticky="ew", pady=4)

        ttk.Label(config_frame, text="Remote template").grid(row=1, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Entry(config_frame, textvariable=self.template_var, width=42).grid(
            row=1,
            column=1,
            columnspan=5,
            sticky="ew",
            pady=4,
        )

        ttk.Label(config_frame, text="Auto script").grid(row=2, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Entry(config_frame, textvariable=self.auto_path_var, width=24).grid(
            row=2,
            column=1,
            columnspan=3,
            sticky="ew",
            pady=4,
        )
        ttk.Label(config_frame, text="Threshold(cm)").grid(row=2, column=4, sticky="w", padx=(12, 8), pady=4)
        ttk.Entry(config_frame, textvariable=self.auto_threshold_var, width=8).grid(row=2, column=5, sticky="ew", pady=4)
        ttk.Button(config_frame, text="Test Connection", command=self.test_connection).grid(
            row=3,
            column=0,
            columnspan=2,
            sticky="w",
            pady=(6, 0),
        )

        ttk.Label(config_frame, text="Control style").grid(row=3, column=2, sticky="w", padx=(12, 8), pady=(6, 0))
        ttk.Radiobutton(config_frame, text="Hold", value="hold", variable=self.control_style_var).grid(
            row=3,
            column=3,
            sticky="w",
            pady=(6, 0),
        )
        ttk.Radiobutton(config_frame, text="Step", value="step", variable=self.control_style_var).grid(
            row=3,
            column=4,
            sticky="w",
            pady=(6, 0),
        )
        ttk.Entry(config_frame, textvariable=self.step_seconds_var, width=8).grid(row=3, column=5, sticky="ew", pady=(6, 0))

        tabs = ttk.Notebook(root_frame)
        tabs.grid(row=1, column=0, sticky="ew", pady=(8, 0))

        drive_tab = ttk.Frame(tabs, padding=6)
        sensors_tab = ttk.Frame(tabs, padding=6)
        camera_tab = ttk.Frame(tabs, padding=6)
        pi_status_tab = ttk.Frame(tabs, padding=6)
        tabs.add(drive_tab, text="Drive")
        tabs.add(sensors_tab, text="Sensors")
        tabs.add(camera_tab, text="Camera")
        tabs.add(pi_status_tab, text="Pi Status")

        # Pi Status tab content
        status_grid = ttk.Frame(pi_status_tab, padding=8)
        status_grid.grid(row=0, column=0, sticky="nsew")
        ttk.Label(status_grid, text="Direction:").grid(row=0, column=0, sticky="w")
        ttk.Label(status_grid, textvariable=self.rt_direction).grid(row=0, column=1, sticky="w")
        ttk.Label(status_grid, text="Speed:").grid(row=1, column=0, sticky="w")
        ttk.Label(status_grid, textvariable=self.rt_speed).grid(row=1, column=1, sticky="w")
        ttk.Label(status_grid, text="Left Blocked:").grid(row=2, column=0, sticky="w")
        ttk.Label(status_grid, textvariable=self.rt_left_blocked).grid(row=2, column=1, sticky="w")
        ttk.Label(status_grid, text="Right Blocked:").grid(row=3, column=0, sticky="w")
        ttk.Label(status_grid, textvariable=self.rt_right_blocked).grid(row=3, column=1, sticky="w")
        ttk.Label(status_grid, text="Distance:").grid(row=4, column=0, sticky="w")
        ttk.Label(status_grid, textvariable=self.rt_distance).grid(row=4, column=1, sticky="w")
        ttk.Label(status_grid, text="Too Close:").grid(row=5, column=0, sticky="w")
        ttk.Label(status_grid, textvariable=self.rt_too_close).grid(row=5, column=1, sticky="w")
        ttk.Label(status_grid, text="Obstacle:").grid(row=6, column=0, sticky="w")
        ttk.Label(status_grid, textvariable=self.rt_obstacle).grid(row=6, column=1, sticky="w")
        ttk.Label(status_grid, text="Emergency Brake:").grid(row=7, column=0, sticky="w")
        self.rt_emergency_brake = tk.StringVar(value="-")
        ttk.Label(status_grid, textvariable=self.rt_emergency_brake).grid(row=7, column=1, sticky="w")
        ttk.Label(status_grid, text="Ultrasonic Servo:").grid(row=8, column=0, sticky="w")
        self.rt_ultra_servo = tk.StringVar(value="center")
        ttk.Label(status_grid, textvariable=self.rt_ultra_servo).grid(row=8, column=1, sticky="w")

        controls_frame = ttk.LabelFrame(drive_tab, text="Controls", padding=6)
        controls_frame.grid(row=0, column=0, sticky="ew")

        button_pad = ttk.Frame(controls_frame)
        button_pad.grid(row=0, column=0, padx=(0, 12), pady=2)
        self._movement_button(button_pad, "Forward", "forward", 0, 1)
        self._movement_button(button_pad, "Left", "left", 1, 0)
        self._action_button(button_pad, "Stop", "stop", 1, 1, style="Accent.TButton")
        self._movement_button(button_pad, "Right", "right", 1, 2)
        self._movement_button(button_pad, "Backward", "backward", 2, 1)

        extras = ttk.Frame(controls_frame)
        extras.grid(row=0, column=1, sticky="n", pady=2)
        self._action_button(extras, "Horn", "horn", 0, 0)
        self._action_button(extras, "Lights", "lights", 1, 0)
        self._action_button(extras, "Speed +", "speed_up", 2, 0)
        self._action_button(extras, "Speed -", "speed_down", 3, 0)
        ttk.Button(extras, text="Start Auto", width=9, command=self.start_auto).grid(row=4, column=0, padx=2, pady=2)
        ttk.Button(extras, text="Stop Auto", width=9, command=self.stop_auto).grid(row=5, column=0, padx=2, pady=2)

        status_frame = ttk.LabelFrame(drive_tab, text="Status", padding=6)
        status_frame.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        ttk.Label(status_frame, text="Mode:").grid(row=0, column=0, sticky="w")
        ttk.Label(status_frame, textvariable=self.mode_var).grid(row=0, column=1, sticky="w", padx=(0, 12))
        ttk.Label(status_frame, text="Dir:").grid(row=0, column=2, sticky="w")
        ttk.Label(status_frame, textvariable=self.direction_var).grid(row=0, column=3, sticky="w", padx=(0, 12))
        ttk.Label(status_frame, text="Speed:").grid(row=0, column=4, sticky="w")
        ttk.Label(status_frame, textvariable=self.speed_var).grid(row=0, column=5, sticky="w")
        ttk.Scale(status_frame, from_=20, to=100, orient="horizontal", variable=self.speed_var).grid(
            row=1,
            column=0,
            columnspan=6,
            sticky="ew",
            pady=(6, 0),
        )

        ir_frame = ttk.LabelFrame(sensors_tab, text="Infrared Obstacle Sensors", padding=6)
        ir_frame.grid(row=0, column=0, sticky="ew")
        ttk.Label(ir_frame, text="Left:").grid(row=0, column=0, sticky="w")
        ttk.Label(ir_frame, textvariable=self.ir_left_var).grid(row=0, column=1, sticky="w", padx=(0, 8))
        ttk.Label(ir_frame, text="Right:").grid(row=0, column=2, sticky="w")
        ttk.Label(ir_frame, textvariable=self.ir_right_var).grid(row=0, column=3, sticky="w", padx=(0, 8))
        ttk.Label(ir_frame, text="Raw:").grid(row=0, column=4, sticky="w")
        ttk.Label(ir_frame, textvariable=self.ir_left_raw_var).grid(row=0, column=5, sticky="w")
        ttk.Label(ir_frame, text="/").grid(row=0, column=6, sticky="w")
        ttk.Label(ir_frame, textvariable=self.ir_right_raw_var).grid(row=0, column=7, sticky="w", padx=(0, 8))
        ttk.Button(ir_frame, text="Refresh", width=8, command=self.refresh_ir_status).grid(row=0, column=8, sticky="w")
        ttk.Checkbutton(ir_frame, text="Auto", variable=self.ir_auto_refresh_var).grid(row=0, column=9, sticky="w", padx=(6, 0))
        ttk.Checkbutton(ir_frame, text="Swap", variable=self.ir_swap_var, command=self.refresh_ir_status).grid(row=0, column=10, sticky="w", padx=(6, 0))

        ultra_frame = ttk.LabelFrame(sensors_tab, text="Ultrasonic Sensor", padding=6)
        ultra_frame.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        ttk.Label(ultra_frame, text="Dist(cm)").grid(row=0, column=0, sticky="w")
        ttk.Label(ultra_frame, textvariable=self.ultra_distance_var).grid(row=0, column=1, sticky="w", padx=(0, 10))
        ttk.Label(ultra_frame, text="Status").grid(row=0, column=2, sticky="w")
        ttk.Label(ultra_frame, textvariable=self.ultra_status_var).grid(row=0, column=3, sticky="w", padx=(0, 10))
        ttk.Button(ultra_frame, text="Refresh", width=8, command=self.refresh_ultrasonic_status).grid(row=0, column=4, sticky="w")
        ttk.Checkbutton(ultra_frame, text="Auto", variable=self.ultra_auto_refresh_var).grid(row=0, column=5, sticky="w", padx=(6, 0))

        camera_frame = ttk.LabelFrame(camera_tab, text="Camera Control", padding=6)
        camera_frame.grid(row=0, column=0, sticky="ew")
        ttk.Label(camera_frame, text="Pan (J2) Angle").grid(row=0, column=0, sticky="w")
        ttk.Label(camera_frame, textvariable=self.camera_angle_var).grid(row=0, column=1, sticky="w", padx=(0, 8))
        ttk.Label(camera_frame, text="Pan (J2) Pin").grid(row=0, column=2, sticky="w")
        ttk.Entry(camera_frame, textvariable=self.camera_servo1_pin_var, width=4).grid(row=0, column=3, sticky="w", padx=(0, 8))
        ttk.Label(camera_frame, text="Step").grid(row=0, column=4, sticky="w")
        ttk.Entry(camera_frame, textvariable=self.camera_step_var, width=4).grid(row=0, column=5, sticky="w", padx=(0, 8))
        ttk.Button(camera_frame, text="Pan Left", width=8, command=lambda: self.move_camera(-abs(self.camera_step_var.get()), servo=1)).grid(row=0, column=6, sticky="w")
        ttk.Button(camera_frame, text="Pan Center", width=9, command=lambda: self.set_camera_angle(90, servo=1)).grid(row=0, column=7, sticky="w", padx=(6, 0))
        ttk.Button(camera_frame, text="Pan Right", width=8, command=lambda: self.move_camera(abs(self.camera_step_var.get()), servo=1)).grid(row=0, column=8, sticky="w", padx=(6, 0))
        ttk.Button(camera_frame, text="Pan Set", width=7, command=lambda: self.set_camera_angle(self.camera_angle_var.get(), servo=1)).grid(row=0, column=9, sticky="w", padx=(6, 0))
        ttk.Button(camera_frame, text="Pan Test", width=8, command=lambda: self.run_servo_self_test(servo=1)).grid(row=0, column=10, sticky="w", padx=(6, 0))
        ttk.Scale(camera_frame, from_=0, to=180, orient="horizontal", variable=self.camera_angle_var).grid(
            row=1,
            column=0,
            columnspan=11,
            sticky="ew",
            pady=(6, 0),
        )

        ttk.Label(camera_frame, text="Tilt (J3) Angle").grid(row=2, column=0, sticky="w", pady=(6, 0))
        ttk.Label(camera_frame, textvariable=self.camera_angle2_var).grid(row=2, column=1, sticky="w", padx=(0, 8), pady=(6, 0))
        ttk.Label(camera_frame, text="Tilt (J3) Pin").grid(row=2, column=2, sticky="w", pady=(6, 0))
        ttk.Entry(camera_frame, textvariable=self.camera_servo2_pin_var, width=4).grid(row=2, column=3, sticky="w", padx=(0, 8), pady=(6, 0))
        ttk.Label(camera_frame, text="Backend").grid(row=2, column=4, sticky="w", pady=(6, 0))
        ttk.Combobox(
            camera_frame,
            textvariable=self.camera_backend_var,
            values=("auto", "pigpio", "gpiozero", "rpi"),
            width=9,
            state="readonly",
        ).grid(row=2, column=5, sticky="w", padx=(0, 8), pady=(6, 0))
        ttk.Button(camera_frame, text="Tilt Down", width=8, command=lambda: self.move_camera(-abs(self.camera_step_var.get()), servo=2)).grid(row=2, column=6, sticky="w", pady=(6, 0))
        ttk.Button(camera_frame, text="Tilt Center", width=9, command=lambda: self.set_center_gui(servo=2, var=self.camera_center_j3_var)).grid(row=2, column=7, sticky="w", padx=(6, 0), pady=(6, 0))
        ttk.Button(camera_frame, text="Tilt Up", width=8, command=lambda: self.move_camera(abs(self.camera_step_var.get()), servo=2)).grid(row=2, column=8, sticky="w", padx=(6, 0), pady=(6, 0))
        ttk.Button(camera_frame, text="Tilt Set", width=7, command=lambda: self.set_camera_angle(self.camera_angle2_var.get(), servo=2)).grid(row=2, column=9, sticky="w", padx=(6, 0), pady=(6, 0))
        ttk.Button(camera_frame, text="Tilt Test", width=8, command=lambda: self.run_servo_self_test(servo=2)).grid(row=2, column=10, sticky="w", padx=(6, 0), pady=(6, 0))
        ttk.Scale(camera_frame, from_=0, to=180, orient="horizontal", variable=self.camera_angle2_var).grid(
            row=3,
            column=0,
            columnspan=11,
            sticky="ew",
            pady=(6, 0),
        )

        ttk.Label(camera_frame, text="Ultrasonic (J1) Angle").grid(row=4, column=0, sticky="w", pady=(6, 0))
        ttk.Label(camera_frame, textvariable=self.camera_angle3_var).grid(row=4, column=1, sticky="w", padx=(0, 8), pady=(6, 0))
        ttk.Label(camera_frame, text="Ultrasonic (J1) Pin").grid(row=4, column=2, sticky="w", pady=(6, 0))
        ttk.Entry(camera_frame, textvariable=self.camera_servo3_pin_var, width=4).grid(row=4, column=3, sticky="w", padx=(0, 8), pady=(6, 0))
        ttk.Button(camera_frame, text="J1 Left", width=8, command=lambda: self.move_camera(-abs(self.camera_step_var.get()), servo=3)).grid(row=4, column=6, sticky="w", pady=(6, 0))
        ttk.Button(camera_frame, text="J1 Center", width=9, command=lambda: self.set_center_gui(servo=3, var=self.camera_center_j1_var)).grid(row=4, column=7, sticky="w", padx=(6, 0), pady=(6, 0))
        ttk.Button(camera_frame, text="J1 Right", width=8, command=lambda: self.move_camera(abs(self.camera_step_var.get()), servo=3)).grid(row=4, column=8, sticky="w", padx=(6, 0), pady=(6, 0))
        ttk.Button(camera_frame, text="J1 Set", width=7, command=lambda: self.set_camera_angle(self.camera_angle3_var.get(), servo=3)).grid(row=4, column=9, sticky="w", padx=(6, 0), pady=(6, 0))
        ttk.Button(camera_frame, text="J1 Test", width=8, command=lambda: self.run_servo_self_test(servo=3)).grid(row=4, column=10, sticky="w", padx=(6, 0), pady=(6, 0))
        ttk.Scale(camera_frame, from_=0, to=180, orient="horizontal", variable=self.camera_angle3_var).grid(
            row=5,
            column=0,
            columnspan=11,
            sticky="ew",
            pady=(6, 0),
        )

        ttk.Button(camera_frame, text="Auto-Correct Camera", width=18, command=self.auto_correct_camera_gui).grid(
            row=6, column=0, columnspan=3, sticky="w", pady=(8, 0)
        )

        # WebSocket controls (optional)
        ttk.Checkbutton(config_frame, text="Use WebSocket", variable=self.ws_enabled_var).grid(row=4, column=0, sticky="w", pady=4)
        ttk.Label(config_frame, text="WS Host").grid(row=4, column=1, sticky="w")
        ttk.Entry(config_frame, textvariable=self.ws_host_var, width=14).grid(row=4, column=2, sticky="w")
        ttk.Label(config_frame, text="WS Port").grid(row=4, column=3, sticky="w")
        ttk.Entry(config_frame, textvariable=self.ws_port_var, width=6).grid(row=4, column=4, sticky="w")
        ttk.Label(config_frame, text="WS Token").grid(row=4, column=5, sticky="w")
        ttk.Entry(config_frame, textvariable=self.ws_token_var, width=16).grid(row=4, column=6, columnspan=2, sticky="w")
        ttk.Checkbutton(config_frame, text="Use ROS2", variable=self.ros_enabled_var).grid(row=4, column=8, sticky="w", padx=(8,0))

        # Debug / calibration: manual center controls for J1 (horizontal) and J3 (vertical)
        ttk.Label(camera_frame, text="J1 Center (deg)").grid(row=6, column=3, sticky="w", padx=(8, 0))
        ttk.Entry(camera_frame, textvariable=self.camera_center_j1_var, width=5).grid(row=6, column=4, sticky="w")
        ttk.Button(camera_frame, text="Set J1 Center", width=12, command=lambda: self.set_center_gui(servo=3, var=self.camera_center_j1_var)).grid(
            row=6, column=5, sticky="w", padx=(6, 0)
        )

        ttk.Label(camera_frame, text="J3 Center (deg)").grid(row=6, column=6, sticky="w", padx=(8, 0))
        ttk.Entry(camera_frame, textvariable=self.camera_center_j3_var, width=5).grid(row=6, column=7, sticky="w")
        ttk.Button(camera_frame, text="Set J3 Center", width=12, command=lambda: self.set_center_gui(servo=2, var=self.camera_center_j3_var)).grid(
            row=6, column=8, sticky="w", padx=(6, 0)
        )

        ttk.Button(camera_frame, text="Apply Both Centers", width=16, command=self.apply_both_centers_gui).grid(
            row=6, column=9, columnspan=2, sticky="w", padx=(8, 0)
        )

        stream_frame = ttk.LabelFrame(camera_tab, text="Camera View", padding=6)
        stream_frame.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        ttk.Label(stream_frame, text="URL").grid(row=0, column=0, sticky="w", padx=(0, 6))
        ttk.Entry(stream_frame, textvariable=self.camera_url_var, width=36).grid(row=0, column=1, columnspan=4, sticky="ew")
        ttk.Button(stream_frame, text="Start Stream", width=10, command=self.start_camera_stream_service).grid(row=0, column=5, sticky="w", padx=(6, 0))
        ttk.Checkbutton(stream_frame, text="Preview", variable=self.camera_preview_on_var, command=self.toggle_camera_preview).grid(
            row=0,
            column=6,
            sticky="w",
            padx=(6, 0),
        )
        self.camera_preview_label = ttk.Label(stream_frame, text="Preview Off", anchor="center")
        self.camera_preview_label.grid(row=1, column=0, columnspan=7, sticky="ew", pady=(6, 0))

        ttk.Label(root_frame, text="Keyboard: W/A/S/D or Arrow Keys, Space=Stop").grid(
            row=2,
            column=0,
            sticky="w",
            pady=(6, 2),
        )
        ttk.Label(root_frame, textvariable=self.status_var).grid(row=3, column=0, sticky="w")

    def _movement_button(self, parent: ttk.Frame, text: str, action: str, row: int, col: int) -> None:
        button = ttk.Button(parent, text=text, width=9)
        button.grid(row=row, column=col, padx=4, pady=4)
        button.bind("<ButtonPress-1>", lambda _: self._on_movement_press(action))
        button.bind("<ButtonRelease-1>", lambda _: self._on_movement_release())

    def _action_button(
        self,
        parent: ttk.Frame,
        text: str,
        action: str,
        row: int,
        col: int,
        style: str | None = None,
    ) -> None:
        button = ttk.Button(parent, text=text, width=9, style=style)
        button.grid(row=row, column=col, padx=4, pady=4)
        button.configure(command=lambda: self.send_action(action))

    def _bind_keys(self) -> None:
        self.root.bind("<KeyPress>", self._on_key_press)
        self.root.bind("<KeyRelease>", self._on_key_release)
        self.root.focus_force()

    def _on_key_press(self, event: tk.Event) -> None:
        key = event.keysym.lower()
        if key in self.pressed_keys:
            return
        self.pressed_keys.add(key)

        movement = {
            "w": "forward",
            "up": "forward",
            "a": "left",
            "left": "left",
            "s": "backward",
            "down": "backward",
            "d": "right",
            "right": "right",
        }

        if key in movement:
            self._on_movement_press(movement[key])
            return

        if key == "space":
            self.send_action("stop")

    def _on_key_release(self, event: tk.Event) -> None:
        key = event.keysym.lower()
        self.pressed_keys.discard(key)

        movement_keys = {"w", "a", "s", "d", "up", "left", "down", "right"}
        if key in movement_keys:
            self._on_movement_release()

    def _on_movement_press(self, action: str) -> None:
        if self.control_style_var.get() == "step":
            self.send_step_action(action)
        else:
            self.send_action(action)

    def _on_movement_release(self) -> None:
        if self.control_style_var.get() == "hold":
            self.send_action("stop")

    def send_step_action(self, action: str) -> None:
        self.send_action(action)

        try:
            step_seconds = max(0.01, float(self.step_seconds_var.get()))
        except (ValueError, tk.TclError):
            step_seconds = 0.25

        def delayed_stop() -> None:
            threading.Event().wait(step_seconds)
            self.send_action("stop")

        threading.Thread(target=delayed_stop, daemon=True).start()

    def _current_config(self) -> SSHConfig:
        return SSHConfig(
            host=self.host_var.get().strip(),
            user=self.user_var.get().strip(),
            port=int(self.port_var.get()),
            identity_file=None,
            connect_timeout=5,
            strict_host_key_checking="accept-new",
        )

    def _prepare_manual_control(self, config: SSHConfig) -> bool:
        conflict_kill = (
              "pkill -f '^python3 .*auto_avoid.py( |$)' >/dev/null 2>&1 || true; "
              "pkill -f '^python3 .*master_system_control.py( |$)' >/dev/null 2>&1 || true; "
              "pkill -f '^python3 .*wechat_control.py( |$)' >/dev/null 2>&1 || true; "
              "pkill -f '^python3 .*PS2_control.py( |$)' >/dev/null 2>&1 || true; "
            "pkill -x bluetooth_control >/dev/null 2>&1 || true; "
            "pkill -f '^sh /home/pi/labboot.sh$' >/dev/null 2>&1 || true; "
            "pkill -f '^sh /home/pi/lingshunlabboot.sh$' >/dev/null 2>&1 || true"
        )
        code = run_remote_command(config, conflict_kill, dry_run=False)
        return code == 0

    def send_action(self, action: str) -> None:
        config = self._current_config()
        template = self.template_var.get().strip()
        speed = self.speed_var.get()
        ws_enabled = self.ws_enabled_var.get()

        self.status_var.set(f"Sending: {action}")

        def worker() -> None:
            movement_actions = {"forward", "backward", "left", "right", "stop"}
            if ws_enabled:
                # WebSocket mode
                msg = {"type": "action", "action": action, "speed": speed}
                host = self.ws_host_var.get().strip()
                port = int(self.ws_port_var.get())
                token = self.ws_token_var.get().strip()
                uri = f"ws://{host}:{port}"
                async def _send():
                    import websockets
                    async with websockets.connect(uri) as ws:
                        if token:
                            msg["token"] = token
                        await ws.send(json.dumps(msg))
                        resp = await ws.recv()
                        return resp
                try:
                    text = asyncio.run(_send())
                    self.root.after(0, lambda: self._update_status_from_action(action))
                    self.root.after(0, lambda: self.status_var.set(f"WS OK: {action}"))
                except Exception as exc:
                    self.root.after(0, lambda exc=exc: self.status_var.set(f"WS Failed: {exc}"))
                return

            # SSH mode
            if action in movement_actions:
                ok = self._prepare_manual_control(config)
                if not ok:
                    self.root.after(0, lambda: self.status_var.set("Failed to take manual control"))
                    return
            try:
                remote_command = template.format(action=action, speed=speed)
            except KeyError as error:
                message = f"Template error: missing field {error}"
                self.root.after(0, lambda: self.status_var.set(message))
                return
            code = run_remote_command(config, remote_command, dry_run=False)
            if code == 2 and "--speed" in remote_command:
                fallback_command = re.sub(r"\s--speed\s+\S+", "", remote_command).strip()
                code = run_remote_command(config, fallback_command, dry_run=False)
            if code == 0:
                self.root.after(0, lambda: self._update_status_from_action(action))
                message = f"OK: {action}"
            else:
                message = f"Failed ({code}): {action}"
            self.root.after(0, lambda: self.status_var.set(message))

        thread = threading.Thread(target=worker, daemon=True)
        thread.start()

    def _update_status_from_action(self, action: str) -> None:
        direction_map = {
            "forward": "Forward",
            "backward": "Backward",
            "left": "Left",
            "right": "Right",
            "stop": "Stopped",
        }

        if action in direction_map:
            self.mode_var.set("Manual")
            self.direction_var.set(direction_map[action])
            return

        if action == "speed_up":
            self.speed_var.set(min(100, self.speed_var.get() + 5))
            return

        if action == "speed_down":
            self.speed_var.set(max(20, self.speed_var.get() - 5))

    def start_auto(self) -> None:
        config = self._current_config()
        auto_path = self.auto_path_var.get().strip()
        threshold = self.auto_threshold_var.get()
        speed = self.speed_var.get()
        ws_enabled = self.ws_enabled_var.get()

        self.status_var.set("Starting auto avoid...")

        def worker() -> None:
            if ws_enabled:
                msg = {"type": "auto", "script": auto_path, "threshold_cm": threshold, "speed": speed}
                host = self.ws_host_var.get().strip()
                port = int(self.ws_port_var.get())
                token = self.ws_token_var.get().strip()
                uri = f"ws://{host}:{port}"
                async def _send():
                    import websockets
                    async with websockets.connect(uri) as ws:
                        if token:
                            msg["token"] = token
                        await ws.send(json.dumps(msg))
                        resp = await ws.recv()
                        return resp
                try:
                    text = asyncio.run(_send())
                    self.root.after(0, lambda: self.mode_var.set("Auto"))
                    self.root.after(0, lambda: self.direction_var.set("Auto Avoid"))
                    self.root.after(0, lambda: self.status_var.set("WS Auto started"))
                except Exception as exc:
                    self.root.after(0, lambda exc=exc: self.status_var.set(f"WS Auto failed: {exc}"))
                return

            remote_command = (
                    f"nohup python3 {shlex.quote(auto_path)} --threshold-cm {threshold} --speed {speed} "
                f"> /tmp/auto_avoid.log 2>&1 &"
            )
            code = run_remote_command(config, remote_command, dry_run=False)
            if code == 0:
                self.root.after(0, lambda: self.mode_var.set("Auto"))
                self.root.after(0, lambda: self.direction_var.set("Auto Avoid"))
                message = "Auto avoid started"
            else:
                message = f"Failed to start auto ({code})"
            self.root.after(0, lambda: self.status_var.set(message))

        threading.Thread(target=worker, daemon=True).start()

    def stop_auto(self) -> None:
        config = self._current_config()
        ws_enabled = self.ws_enabled_var.get()

        self.status_var.set("Stopping auto avoid...")

        def worker() -> None:
            if ws_enabled:
                msg = {"type": "stop_auto"}
                host = self.ws_host_var.get().strip()
                port = int(self.ws_port_var.get())
                token = self.ws_token_var.get().strip()
                uri = f"ws://{host}:{port}"
                async def _send():
                    import websockets
                    async with websockets.connect(uri) as ws:
                        if token:
                            msg["token"] = token
                        await ws.send(json.dumps(msg))
                        resp = await ws.recv()
                        return resp
                try:
                    text = asyncio.run(_send())
                    self.root.after(0, lambda: self.mode_var.set("Manual"))
                    self.root.after(0, lambda: self.direction_var.set("Stopped"))
                    self.root.after(0, lambda: self.status_var.set("WS Auto stopped"))
                except Exception as exc:
                    self.root.after(0, lambda exc=exc: self.status_var.set(f"WS Stop auto failed: {exc}"))
                return

            code = run_remote_command(config, "pkill -f '^python .*auto_avoid.py( |$)' >/dev/null 2>&1 || true", dry_run=False)
            template = self.template_var.get().strip()
            speed = self.speed_var.get()
            try:
                stop_command = template.format(action="stop", speed=speed)
            except KeyError:
                stop_command = "python /home/pi/car_action.py stop"
            code2 = run_remote_command(config, stop_command, dry_run=False)
            if code == 0 and code2 == 0:
                self.root.after(0, lambda: self.mode_var.set("Manual"))
                self.root.after(0, lambda: self.direction_var.set("Stopped"))
                message = "Auto avoid stopped"
            else:
                message = f"Stop auto may have failed ({code}/{code2})"
            self.root.after(0, lambda: self.status_var.set(message))

        threading.Thread(target=worker, daemon=True).start()

    def test_connection(self) -> None:
        config = self._current_config()
        self.status_var.set("Testing connection...")

        def worker() -> None:
            code = run_remote_command(config, "echo connected", dry_run=False)
            if code == 0:
                message = "Connection OK"
            else:
                message = f"Connection failed ({code})"
            self.root.after(0, lambda: self.status_var.set(message))

        threading.Thread(target=worker, daemon=True).start()

    def _schedule_sensor_refresh(self) -> None:
        self.root.after(800, self._auto_refresh_sensors)

    def _auto_refresh_sensors(self) -> None:
        if self.ir_auto_refresh_var.get():
            self.refresh_ir_status(silent=True)
        if self.ultra_auto_refresh_var.get():
            self.refresh_ultrasonic_status(silent=True)
        self._schedule_sensor_refresh()

    def refresh_ir_status(self, silent: bool = False) -> None:
        if self._ir_read_in_progress:
            return

        config = self._current_config()
        self._ir_read_in_progress = True
        if not silent:
            self.status_var.set("Reading IR sensors...")

        remote_command = (
                "python3 - <<'PY'\n"
            "import RPi.GPIO as GPIO\n"
            "LEFT=12\n"
            "RIGHT=17\n"
            "GPIO.setmode(GPIO.BCM)\n"
            "GPIO.setwarnings(False)\n"
            "GPIO.setup(LEFT, GPIO.IN)\n"
            "GPIO.setup(RIGHT, GPIO.IN)\n"
            "left = GPIO.input(LEFT)\n"
            "right = GPIO.input(RIGHT)\n"
            "print('%d %d' % (left, right))\n"
            "PY"
        )

        def worker() -> None:
            ssh_command = ["ssh", "-p", str(config.port), "-o", f"ConnectTimeout={config.connect_timeout}", "-o", f"StrictHostKeyChecking={config.strict_host_key_checking}"]
            if config.identity_file:
                ssh_command.extend(["-i", config.identity_file])
            ssh_command.extend([f"{config.user}@{config.host}", remote_command])

            import subprocess

            process = subprocess.run(ssh_command, capture_output=True, text=True)
            if process.returncode != 0:
                if not silent:
                    self.root.after(0, lambda: self.status_var.set(f"IR read failed ({process.returncode})"))
                self._ir_read_in_progress = False
                return

            output = process.stdout.strip().split()
            if len(output) != 2 or output[0] not in {"0", "1"} or output[1] not in {"0", "1"}:
                if not silent:
                    self.root.after(0, lambda: self.status_var.set("IR read parse failed"))
                self._ir_read_in_progress = False
                return

            left_raw = output[0]
            right_raw = output[1]
            if self.ir_swap_var.get():
                left_raw, right_raw = right_raw, left_raw

            left_blocked = left_raw == "0"
            right_blocked = right_raw == "0"

            self.root.after(0, lambda: self.ir_left_var.set("Close" if left_blocked else "Clear"))
            self.root.after(0, lambda: self.ir_right_var.set("Close" if right_blocked else "Clear"))
            self.root.after(0, lambda: self.ir_left_raw_var.set(left_raw))
            self.root.after(0, lambda: self.ir_right_raw_var.set(right_raw))
            if not silent:
                self.root.after(0, lambda: self.status_var.set("IR sensors updated"))
            self._ir_read_in_progress = False

        threading.Thread(target=worker, daemon=True).start()

    def refresh_ultrasonic_status(self, silent: bool = False) -> None:
        if self._ultra_read_in_progress:
            return

        config = self._current_config()
        self._ultra_read_in_progress = True
        if not silent:
            self.status_var.set("Reading ultrasonic distance...")

        remote_command = (
            "python - <<'PY'\n"
            "import time\n"
            "import RPi.GPIO as GPIO\n"
            "TRIG=1\n"
            "ECHO=0\n"
            "GPIO.setmode(GPIO.BCM)\n"
            "GPIO.setwarnings(False)\n"
            "GPIO.setup(TRIG, GPIO.OUT)\n"
            "GPIO.setup(ECHO, GPIO.IN)\n"
            "GPIO.output(TRIG, GPIO.LOW)\n"
            "time.sleep(0.02)\n"
            "GPIO.output(TRIG, GPIO.HIGH)\n"
            "time.sleep(0.000015)\n"
            "GPIO.output(TRIG, GPIO.LOW)\n"
            "timeout=0.03\n"
            "start=time.time()\n"
            "while GPIO.input(ECHO)==0:\n"
            "    if time.time()-start>timeout:\n"
            "        print('nan')\n"
            "        raise SystemExit\n"
            "pulse_start=time.time()\n"
            "while GPIO.input(ECHO)==1:\n"
            "    if time.time()-pulse_start>timeout:\n"
            "        print('nan')\n"
            "        raise SystemExit\n"
            "pulse_end=time.time()\n"
            "distance=(pulse_end-pulse_start)*17150.0\n"
            "print('%.2f' % distance)\n"
            "PY"
        )

        def worker() -> None:
            ssh_command = ["ssh", "-p", str(config.port), "-o", f"ConnectTimeout={config.connect_timeout}", "-o", f"StrictHostKeyChecking={config.strict_host_key_checking}"]
            if config.identity_file:
                ssh_command.extend(["-i", config.identity_file])
            ssh_command.extend([f"{config.user}@{config.host}", remote_command])

            import subprocess

            process = subprocess.run(ssh_command, capture_output=True, text=True)
            if process.returncode != 0:
                if not silent:
                    self.root.after(0, lambda: self.status_var.set(f"Distance read failed ({process.returncode})"))
                self._ultra_read_in_progress = False
                return

            text = process.stdout.strip().splitlines()
            value = text[-1].strip() if text else "nan"
            try:
                distance = float(value)
            except ValueError:
                distance = float("nan")

            if value == "nan" or distance != distance:
                self.root.after(0, lambda: self.ultra_distance_var.set("No Echo"))
                self.root.after(0, lambda: self.ultra_status_var.set("Unknown"))
                if not silent:
                    self.root.after(0, lambda: self.status_var.set("Ultrasonic no-echo"))
                self._ultra_read_in_progress = False
                return

            threshold = float(self.auto_threshold_var.get())
            is_close = distance < threshold
            self.root.after(0, lambda: self.ultra_distance_var.set(f"{distance:.2f}"))
            self.root.after(0, lambda: self.ultra_status_var.set("Close" if is_close else "Clear"))
            if not silent:
                self.root.after(0, lambda: self.status_var.set("Ultrasonic updated"))
            self._ultra_read_in_progress = False

        threading.Thread(target=worker, daemon=True).start()

    def _servo_angle_var(self, servo: int) -> tk.IntVar:
        if servo == 1:
            return self.camera_angle_var
        elif servo == 2:
            return self.camera_angle2_var
        else:
            return self.camera_angle3_var

    def _servo_pin_var(self, servo: int) -> tk.IntVar:
        if servo == 1:
            return self.camera_servo1_pin_var
        elif servo == 2:
            return self.camera_servo2_pin_var
        else:
            return self.camera_servo3_pin_var

    def _ws_send_command(self, servo: int, angle: float, pin: int, backend: str) -> dict:
        """Send a servo command over WebSocket and return parsed JSON response.

        Returns a dict with server response or an error field.
        """
        if websockets is None:
            return {"error": "websockets package not available"}

        host = self.ws_host_var.get().strip()
        port = int(self.ws_port_var.get())
        token = self.ws_token_var.get().strip()
        uri = f"ws://{host}:{port}"

        async def _send() -> str:
            async with websockets.connect(uri) as ws:
                msg = {"type": "servo", "servo": servo, "angle": angle, "pin": pin, "backend": backend}
                if token:
                    msg["token"] = token
                await ws.send(json.dumps(msg))
                resp = await ws.recv()
                return resp

        try:
            text = asyncio.run(_send())
        except Exception as exc:  # pragma: no cover - runtime network error
            return {"error": str(exc)}

        try:
            return json.loads(text)
        except Exception:
            return {"raw": text}

    def _ros_publish_command(self, servo: int, angle: float, pin: int, backend: str) -> dict:
        """Publish a servo command using ROS2 CLI (fallback) as JSON string on /servo_control.

        Uses `ros2 topic pub -1 /servo_control std_msgs/msg/String "data: '{...}'"`.
        Requires ROS2 `ros2` CLI available on this machine and that ROS network is configured.
        """
        msg = json.dumps({"type": "servo", "servo": servo, "angle": angle, "pin": pin, "backend": backend})

        # Prefer using rclpy to publish directly when available (no external CLI required).
        try:
            import rclpy
            from std_msgs.msg import String
        except Exception:
            rclpy = None

        if rclpy is not None:
            try:
                rclpy.init()
                node = rclpy.create_node("car_controller_gui_pub")
                pub = node.create_publisher(String, "/servo_control", 10)
                m = String()
                m.data = msg
                pub.publish(m)
                # give middleware a moment to send
                try:
                    rclpy.spin_once(node, timeout_sec=0.1)
                except Exception:
                    pass
                node.destroy_node()
                rclpy.shutdown()
                return {"returncode": 0, "stdout": "published via rclpy"}
            except Exception as exc:
                return {"error": f"rclpy publish failed: {exc}"}

        # Fallback to using the ros2 CLI if rclpy is unavailable.
        cmd = [
            "ros2",
            "topic",
            "pub",
            "/servo_control",
            "std_msgs/msg/String",
            f"data: '{msg}'",
            "-1",
        ]
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=8)
            return {"returncode": proc.returncode, "stdout": proc.stdout, "stderr": proc.stderr}
        except FileNotFoundError:
            return {"error": "ros2 CLI not found"}
        except Exception as exc:
            return {"error": str(exc)}

    # Using absolute 0-180 domain for all servos.

    def move_camera(self, delta: int, servo: int = 1) -> None:
        angle_var = self._servo_angle_var(servo)
        target = max(0, min(180, int(angle_var.get()) + int(delta)))
        self.set_camera_angle(target, servo=servo)

    def set_camera_angle(self, angle: int, servo: int = 1) -> None:
        # Use absolute 0..180 for all servos
        angle_var = self._servo_angle_var(servo)
        pin_var = self._servo_pin_var(servo)

        display = max(0, min(180, int(angle)))
        angle_var.set(display)
        servo_angle = display

        pin = int(pin_var.get())
        if pin < 0 or pin > 27:
            self.status_var.set(f"Servo {servo} invalid pin: {pin}")
            return

        config = self._current_config()
        self.status_var.set(f"Setting servo {servo} angle: {servo_angle}")
        backend = self.camera_backend_var.get().strip() or "auto"

        remote_command = (
            "python /home/pi/camera_servo.py --servo %s --pin %s --angle %s --backend %s" % (servo, pin, servo_angle, backend)
        )

        def worker() -> None:
            # Prefer ROS2 publishing when enabled, then WebSocket, then SSH
            if self.ros_enabled_var.get():
                resp = self._ros_publish_command(servo, servo_angle, pin, backend)
                if isinstance(resp, dict) and resp.get("returncode", 1) == 0:
                    message = f"Servo {servo} angle set: {servo_angle} (ros)"
                elif isinstance(resp, dict) and resp.get("error"):
                    message = f"Servo {servo} control failed (ros:{resp})"
                else:
                    message = f"Servo {servo} control unexpected response (ros:{resp})"
            elif self.ws_enabled_var.get():
                # send via websocket to Pi
                resp = self._ws_send_command(servo, servo_angle, pin, backend)
                if isinstance(resp, dict) and resp.get("returncode", 1) == 0:
                    message = f"Servo {servo} angle set: {servo_angle}"
                else:
                    message = f"Servo {servo} control failed (ws:{resp})"
            else:
                code = run_remote_command(
                    config,
                    f"python /home/pi/camera_servo.py --servo {servo} --pin {pin} --angle {servo_angle} --backend {backend}",
                    dry_run=False,
                )
                if code == 0:
                    message = f"Servo {servo} angle set: {servo_angle}"
                else:
                    message = f"Servo {servo} control failed ({code})"
            self.root.after(0, lambda: self.status_var.set(message))

        threading.Thread(target=worker, daemon=True).start()

    def run_servo_self_test(self, servo: int = 1) -> None:
        pin = int(self._servo_pin_var(servo).get())
        if pin < 0 or pin > 27:
            self.status_var.set(f"Servo {servo} invalid pin: {pin}")
            return

        config = self._current_config()
        # absolute-domain tests for all servos
        sequence = [90, 60, 120, 90]
        backend = self.camera_backend_var.get().strip() or "auto"
        self.status_var.set(f"Running servo {servo} self-test...")

        def worker() -> None:
            for index, angle in enumerate(sequence):
                # set display var
                self.root.after(0, lambda a=angle: self._servo_angle_var(servo).set(a))
                if servo == 1:
                    send_angle = int(angle)
                else:
                    send_angle = self._display_to_servo_angle(servo, int(angle))
                # Prefer ROS2 -> WebSocket -> SSH
                if self.ros_enabled_var.get():
                    resp = self._ros_publish_command(servo, send_angle, pin, backend)
                    code = 0 if (isinstance(resp, dict) and resp.get("returncode", 1) == 0) else 1
                elif self.ws_enabled_var.get():
                    resp = self._ws_send_command(servo, send_angle, pin, backend)
                    code = 0 if (isinstance(resp, dict) and resp.get("returncode", 1) == 0) else 1
                else:
                    code = run_remote_command(
                        config,
                        f"python /home/pi/camera_servo.py --servo {servo} --pin {pin} --angle {send_angle} --backend {backend}",
                        dry_run=False,
                    )
                if code != 0:
                    self.root.after(0, lambda c=code: self.status_var.set(f"Servo {servo} self-test failed ({c})"))
                    return
                if index < len(sequence) - 1:
                    time.sleep(0.35)
            self.root.after(0, lambda: self.status_var.set(f"Servo {servo} self-test complete"))

        threading.Thread(target=worker, daemon=True).start()

    def set_center_gui(self, servo: int, var: tk.IntVar) -> None:
        """Set a single servo to the given center angle (used for debug/calibration).

        `servo` is the GUI servo index (1..3 as used in this GUI).
        `var` is the IntVar containing the desired center angle.
        """
        angle = max(0, min(180, int(var.get())))
        pin = int(self._servo_pin_var(servo).get())
        if pin < 0 or pin > 27:
            self.status_var.set(f"Servo {servo} invalid pin: {pin}")
            return

        config = self._current_config()
        backend = self.camera_backend_var.get().strip() or "auto"
        self.status_var.set(f"Setting servo {servo} center to {angle}...")

        def worker() -> None:
            # Prefer ROS2 -> WebSocket -> SSH
            if self.ros_enabled_var.get():
                resp = self._ros_publish_command(servo, angle, pin, backend)
                if isinstance(resp, dict) and resp.get("returncode", 1) == 0:
                    self.root.after(0, lambda: self.status_var.set(f"Servo {servo} center set: {angle}"))
                else:
                    self.root.after(0, lambda: self.status_var.set(f"Set center failed (ros:{resp})"))
            elif self.ws_enabled_var.get():
                resp = self._ws_send_command(servo, angle, pin, backend)
                if isinstance(resp, dict) and resp.get("returncode", 1) == 0:
                    self.root.after(0, lambda: self.status_var.set(f"Servo {servo} center set: {angle}"))
                else:
                    self.root.after(0, lambda: self.status_var.set(f"Set center failed (ws:{resp})"))
            else:
                code = run_remote_command(
                    config,
                    f"python /home/pi/camera_servo.py --servo {servo} --pin {pin} --angle {angle} --backend {backend}",
                    dry_run=False,
                )
                if code == 0:
                    self.root.after(0, lambda: self.status_var.set(f"Servo {servo} center set: {angle}"))
                else:
                    self.root.after(0, lambda: self.status_var.set(f"Set center failed ({code})"))

        threading.Thread(target=worker, daemon=True).start()

    def apply_both_centers_gui(self) -> None:
        """Apply both configured center angles to J1 and J3 (GUI servos 3 and 2)."""
        c_j1 = max(0, min(180, int(self.camera_center_j1_var.get())))
        c_j3 = max(0, min(180, int(self.camera_center_j3_var.get())))
        # set J1 (servo 3) then J3 (servo 2)
        self.set_center_gui(servo=3, var=self.camera_center_j1_var)
        # small delay before second command to avoid overwhelming remote
        def delayed_second() -> None:
            time.sleep(0.15)
            self.set_center_gui(servo=2, var=self.camera_center_j3_var)

        threading.Thread(target=delayed_second, daemon=True).start()

    def auto_correct_camera_gui(self) -> None:
        """Auto-correct camera direction: center J1 (horizontal) and J3 (vertical) to 90 degrees.

        Note: In this GUI J1 is controlled via servo index 3 and J3 via servo index 2.
        """
        # J1 -> servo index 3, J3 -> servo index 2 (matches GUI labels)
        servo_h = 3
        servo_v = 2

        # Use configured center values if available
        angle_h = int(self.camera_center_j1_var.get())
        angle_v = int(self.camera_center_j3_var.get())

        pin_h = int(self._servo_pin_var(servo_h).get())
        pin_v = int(self._servo_pin_var(servo_v).get())
        if pin_h < 0 or pin_h > 27:
            self.status_var.set(f"Servo {servo_h} invalid pin: {pin_h}")
            return
        if pin_v < 0 or pin_v > 27:
            self.status_var.set(f"Servo {servo_v} invalid pin: {pin_v}")
            return

        config = self._current_config()
        backend = self.camera_backend_var.get().strip() or "auto"
        self.status_var.set("Auto-correcting camera (using configured centers)...")

        def worker() -> None:
            # Prefer ROS2 -> WebSocket -> SSH
            if self.ros_enabled_var.get():
                resp_h = self._ros_publish_command(servo_h, angle_h, pin_h, backend)
                code_h = 0 if (isinstance(resp_h, dict) and resp_h.get("returncode", 1) == 0) else 1
                resp_v = self._ros_publish_command(servo_v, angle_v, pin_v, backend)
                code_v = 0 if (isinstance(resp_v, dict) and resp_v.get("returncode", 1) == 0) else 1
            elif self.ws_enabled_var.get():
                resp_h = self._ws_send_command(servo_h, angle_h, pin_h, backend)
                code_h = 0 if (isinstance(resp_h, dict) and resp_h.get("returncode", 1) == 0) else 1
                resp_v = self._ws_send_command(servo_v, angle_v, pin_v, backend)
                code_v = 0 if (isinstance(resp_v, dict) and resp_v.get("returncode", 1) == 0) else 1
            else:
                cmd_h = f"python /home/pi/camera_servo.py --servo {servo_h} --pin {pin_h} --angle {angle_h} --backend {backend}"
                code_h = run_remote_command(config, cmd_h, dry_run=False)
                cmd_v = f"python /home/pi/camera_servo.py --servo {servo_v} --pin {pin_v} --angle {angle_v} --backend {backend}"
                code_v = run_remote_command(config, cmd_v, dry_run=False)

            # Do not modify slider display; keep sliders at their defaults
            # so the user can continue using relative controls even when
            # configured centers are 0.

            if code_h == 0 and code_v == 0:
                message = "Camera auto-correct complete"
            else:
                message = f"Auto-correct failed ({code_h}/{code_v})"
            self.root.after(0, lambda: self.status_var.set(message))

        threading.Thread(target=worker, daemon=True).start()

    def start_camera_stream_service(self) -> None:
        config = self._current_config()
        self.status_var.set("Starting camera stream service...")
        remote_command = "cd /home/pi/mjpg-streamer-master/mjpg-streamer-experimental && sh ./start.sh"

        def worker() -> None:
            code = run_remote_command(config, remote_command, dry_run=False)
            if code == 0:
                message = "Camera stream started"
            else:
                message = f"Camera stream start failed ({code})"
            self.root.after(0, lambda: self.status_var.set(message))

        threading.Thread(target=worker, daemon=True).start()

    def toggle_camera_preview(self) -> None:
        if not self.camera_preview_on_var.get():
            self.camera_preview_label.configure(text="Preview Off", image="")
            self._camera_preview_image = None
            return

        if Image is None or ImageTk is None:
            self.camera_preview_label.configure(text="Install Pillow for preview", image="")
            self.status_var.set("Camera preview unavailable: Pillow not installed")
            self.camera_preview_on_var.set(False)
            return

        self.camera_preview_label.configure(text="Loading...")
        self._schedule_camera_preview()

    def _schedule_camera_preview(self) -> None:
        self.root.after(250, self._camera_preview_tick)

    def _camera_preview_tick(self) -> None:
        if not self.camera_preview_on_var.get() or self._camera_fetch_in_progress:
            if self.camera_preview_on_var.get():
                self._schedule_camera_preview()
            return

        self._camera_fetch_in_progress = True
        url = self.camera_url_var.get().strip()

        def worker() -> None:
            try:
                with urllib.request.urlopen(url, timeout=2.0) as response:
                    data = response.read()
                image = Image.open(io.BytesIO(data)).convert("RGB")
                image.thumbnail((420, 260))
                photo = ImageTk.PhotoImage(image)

                def apply_image() -> None:
                    self._camera_preview_image = photo
                    self.camera_preview_label.configure(image=photo, text="")

                self.root.after(0, apply_image)
            except Exception:
                self.root.after(0, lambda: self.camera_preview_label.configure(text="No camera frame", image=""))
            finally:
                self._camera_fetch_in_progress = False
                if self.camera_preview_on_var.get():
                    self._schedule_camera_preview()

        threading.Thread(target=worker, daemon=True).start()

    def _start_status_ws_client(self):
        def run_ws():
            import asyncio
            import websockets
            import json
            async def listen():
                uri = "ws://" + self.host_var.get() + ":9000"
                while True:
                    try:
                        async with websockets.connect(uri) as ws:
                            while True:
                                msg = await ws.recv()
                                data = json.loads(msg)
                                self.root.after(0, lambda d=data: self._update_rt_status(d))
                    except Exception:
                        time.sleep(2)
            asyncio.run(listen())
        threading.Thread(target=run_ws, daemon=True).start()

    def _update_rt_status(self, data):
        self.rt_direction.set(data.get("direction", "-"))
        self.rt_speed.set(str(data.get("speed", "-")))
        self.rt_left_blocked.set(str(data.get("left_blocked", "-")))
        self.rt_right_blocked.set(str(data.get("right_blocked", "-")))
        self.rt_distance.set(str(data.get("distance", "-")))
        self.rt_too_close.set(str(data.get("too_close", "-")))
        self.rt_obstacle.set(str(data.get("obstacle", "-")))
        emergency_brake = data.get("emergency_brake", False)
        self.rt_emergency_brake.set("ON" if emergency_brake else "OFF")
        self.rt_ultra_servo.set(str(data.get("ultra_servo", "center")))


def main() -> None:
    root = tk.Tk()
    CarControllerGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
