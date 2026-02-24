#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import time
import statistics
import ultrasonic_sensor
import json
import threading
try:
    import websockets
except ImportError:
    websockets = None

def ws_broadcast_status(status):
    # Send status to WebSocket server (localhost:9000)
    try:
        import asyncio
        async def send():
            uri = "ws://127.0.0.1:9000"
            try:
                async with websockets.connect(uri) as ws:
                    await ws.send(json.dumps(status))
            except Exception:
                pass
        threading.Thread(target=lambda: asyncio.run(send()), daemon=True).start()
    except Exception:
        pass
try:
    import RPi.GPIO as GPIO
except ImportError:
    # Try PyRoboSim's GPIO, else use a minimal mock for simulation
    try:
        from pyrobosim.utils.gpio import SimulatedGPIO as GPIO
    except ImportError:
        class MockGPIO:
            BCM = OUT = IN = HIGH = LOW = None
            def setmode(self, *a, **kw): pass
            def setwarnings(self, *a, **kw): pass
            def setup(self, *a, **kw): pass
            def output(self, *a, **kw): pass
            def input(self, *a, **kw): return 1
            def PWM(self, *a, **kw):
                class PWM:
                    def start(self, *a, **kw): pass
                    def stop(self, *a, **kw): pass
                return PWM()
            def cleanup(self, *a, **kw): pass
        GPIO = MockGPIO()

IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13
ECHO_PIN = 0
TRIG_PIN = 1
AVOID_LEFT = 12
AVOID_RIGHT = 17
# Default servo pin for ultrasonic mount (J1). Can be overridden with --servo-pin
DEFAULT_ULTRA_SERVO_PIN = 23


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--threshold-cm', type=float, default=30.0)
    parser.add_argument('--speed', type=int, default=45)
    parser.add_argument('--loop-seconds', type=float, default=0.05)
    parser.add_argument('--back-seconds', type=float, default=0.20)
    parser.add_argument('--turn-seconds', type=float, default=0.30)
    parser.add_argument('--turn-repeat-limit', type=int, default=3, help='Number of repeated left/right turns before backward escape')
    return parser

def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


def setup(speed):
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(ENA, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(ENB, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(TRIG_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ECHO_PIN, GPIO.IN)
    GPIO.setup(AVOID_LEFT, GPIO.IN)
    GPIO.setup(AVOID_RIGHT, GPIO.IN)

    pwm_ena = GPIO.PWM(ENA, 2000)
    pwm_enb = GPIO.PWM(ENB, 2000)
    pwm_ena.start(speed)
    pwm_enb.start(speed)
    return pwm_ena, pwm_enb


def set_motor(a1, a2, b1, b2):
    GPIO.output(IN1, GPIO.HIGH if a1 else GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH if a2 else GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH if b1 else GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH if b2 else GPIO.LOW)


def move_forward():
    set_motor(1, 0, 1, 0)


def move_backward():
    set_motor(0, 1, 0, 1)


def turn_left():
    set_motor(0, 1, 1, 0)


def turn_right():
    set_motor(1, 0, 0, 1)


def brake():
    set_motor(0, 0, 0, 0)


def is_blocked_left():
    return GPIO.input(AVOID_LEFT) == GPIO.LOW


def is_blocked_right():
    return GPIO.input(AVOID_RIGHT) == GPIO.LOW


def measure_distance_cm(timeout_seconds=0.03):
        """Return the median of several ultrasonic sensor readings for robustness."""
        readings = []
        samples = 5
        delay = 0.03
        for _ in range(samples):
            d = ultrasonic_sensor.get_distance_cm()
            if d is not None and 2 < d < 400:
                readings.append(d)
            time.sleep(delay)
        if readings:
            return statistics.median(readings)
        return None


def choose_turn(left_blocked, right_blocked, prefer_left):
    if not left_blocked and right_blocked:
        return 'left'
    if left_blocked and not right_blocked:
        return 'right'
    if not left_blocked and not right_blocked:
        return 'left' if prefer_left else 'right'
    return 'right'


try:
    import camera_servo
except Exception:  # pragma: no cover
    camera_servo = None


def median(values):
    if not values:
        return None
    return statistics.median(values)


def sample_distance(samples=3, timeout_seconds=0.03, inter_delay=0.02):
    vals = []
    for _ in range(max(1, int(samples))):
        v = measure_distance_cm(timeout_seconds=timeout_seconds)
        if v is not None:
            vals.append(v)
        time.sleep(inter_delay)
    return median(vals) if vals else None


def sweep_distances(servo_pin, center_angle=90, delta_deg=40, samples=3, hold_s=0.12):
    """Move the ultrasonic-mounted servo and take samples at left/center/right.

    Returns a dict: {'left': float|None, 'center': float|None, 'right': float|None}
    If servo control isn't available or fails, returns None.
    """
    if camera_servo is None:
        return None

    angles = {
        'left': clamp(center_angle + delta_deg, 0, 180),
        'center': clamp(center_angle, 0, 180),
        'right': clamp(center_angle - delta_deg, 0, 180),
    }

    results = {}
    for name, ang in angles.items():
        try:
            camera_servo.set_servo_angle(servo_pin, ang, 'auto', hold_s)
        except Exception:
            return None

        # take a few quick samples and return median
        val = sample_distance(samples=samples)
        results[name] = val

    # try to recentre sensor to center
    try:
        camera_servo.set_servo_angle(servo_pin, center_angle, 'auto', hold_s)
    except Exception:
        pass

    return results


def choose_direction_fused(left_blocked, right_blocked, sweep, threshold_cm, prefer_left):
    """Return 'left'|'right'|'both' where 'both' means no clear side."""
    if left_blocked and right_blocked:
        return 'both'
    if left_blocked:
        return 'right'
    if right_blocked:
        return 'left'

    # If no sweep results available, fall back to simple toggle logic
    if not sweep:
        return choose_turn(left_blocked, right_blocked, prefer_left)

    l = sweep.get('left')
    r = sweep.get('right')
    # If both unreadable, fallback
    if l is None and r is None:
        return choose_turn(left_blocked, right_blocked, prefer_left)

    # Treat None as very small distance
    l_val = l if l is not None else 0.0
    r_val = r if r is not None else 0.0

    # if both sides below threshold, consider both blocked
    if l_val < threshold_cm and r_val < threshold_cm:
        return 'both'

    # choose the side with larger clearance; tie-break with prefer_left
    if abs(l_val - r_val) < 1e-6:
        return 'left' if prefer_left else 'right'
    return 'left' if l_val > r_val else 'right'


def main():
    stuck_counter = 0
    stuck_threshold = 10  # Number of cycles with no progress before stuck
    last_distance = None
    last_left_blocked = None
    last_right_blocked = None
    stuck_triggered = False
    args = build_parser().parse_args()
    normal_speed = 30
    slow_speed = 15
    speed = normal_speed
    loop_seconds = max(0.02, float(args.loop_seconds))
    threshold_cm = max(10.0, float(args.threshold_cm))
    back_seconds = max(0.05, float(args.back_seconds))
    turn_seconds = max(0.05, float(args.turn_seconds))

    pwm_ena, pwm_enb = setup(speed)
    prefer_left = True

    try:
        # Track repeated turns
        stuck_threshold = 10  # Number of cycles with no progress before stuck
        turn_count = 0
        # Move backward after this many repeated left/right turns (corner escape)
        turn_repeat_limit = getattr(args, 'turn_repeat_limit', 3)
        sweep_enabled = True  # Enable sweep logic

        prev_distance = None
        emergency_brake_triggered = False

        while True:
            left_blocked = is_blocked_left()
            right_blocked = is_blocked_right()
            distance = measure_distance_cm()
            too_close = distance is not None and distance < threshold_cm

            # Stuck detection: if distance and IR sensors unchanged for stuck_threshold cycles
            if (distance == last_distance and left_blocked == last_left_blocked and right_blocked == last_right_blocked):
                stuck_counter += 1
            else:
                stuck_counter = 0
            last_distance = distance
            last_left_blocked = left_blocked
            last_right_blocked = right_blocked

            stuck_triggered = stuck_counter >= stuck_threshold
            # If stuck, sweep with ultrasonic sensor to find the best way out
            if stuck_triggered:
                print("[STUCK] Car appears stuck. Sweeping to find way out.")
                brake()
                ws_broadcast_status({"direction": "brake", "speed": speed, "left_blocked": left_blocked, "right_blocked": right_blocked, "distance": distance, "threshold_cm": threshold_cm, "too_close": too_close, "emergency_brake": False, "obstacle": True, "stuck": True, "ultra_servo": "sweeping"})
                time.sleep(0.1)
                sweep = None
                if camera_servo is not None:
                    sweep = sweep_distances(DEFAULT_ULTRA_SERVO_PIN, center_angle=90, delta_deg=40, samples=3, hold_s=0.12)
                    print(f"[STUCK] Sweep results: {sweep}")
                if sweep:
                    fused_dir = choose_direction_fused(left_blocked, right_blocked, sweep, threshold_cm, prefer_left)
                    print(f"[STUCK] Fused direction: {fused_dir}")
                    if fused_dir == 'left':
                        turn_left()
                        ws_broadcast_status({"direction": "left", "speed": speed, "left_blocked": left_blocked, "right_blocked": right_blocked, "distance": distance, "threshold_cm": threshold_cm, "too_close": too_close, "emergency_brake": False, "obstacle": True, "stuck": True, "ultra_servo": "left"})
                        time.sleep(turn_seconds)
                        brake()
                        ws_broadcast_status({"direction": "brake", "speed": speed, "left_blocked": left_blocked, "right_blocked": right_blocked, "distance": distance, "threshold_cm": threshold_cm, "too_close": too_close, "emergency_brake": False, "obstacle": True, "stuck": True, "ultra_servo": "center"})
                    elif fused_dir == 'right':
                        turn_right()
                        ws_broadcast_status({"direction": "right", "speed": speed, "left_blocked": left_blocked, "right_blocked": right_blocked, "distance": distance, "threshold_cm": threshold_cm, "too_close": too_close, "emergency_brake": False, "obstacle": True, "stuck": True, "ultra_servo": "right"})
                        time.sleep(turn_seconds)
                        brake()
                        ws_broadcast_status({"direction": "brake", "speed": speed, "left_blocked": left_blocked, "right_blocked": right_blocked, "distance": distance, "threshold_cm": threshold_cm, "too_close": too_close, "emergency_brake": False, "obstacle": True, "stuck": True, "ultra_servo": "center"})
                    else:
                        move_backward()
                        ws_broadcast_status({"direction": "backward", "speed": speed, "left_blocked": left_blocked, "right_blocked": right_blocked, "distance": distance, "threshold_cm": threshold_cm, "too_close": too_close, "emergency_brake": False, "obstacle": True, "stuck": True, "ultra_servo": "center"})
                        time.sleep(back_seconds)
                        brake()
                        ws_broadcast_status({"direction": "brake", "speed": speed, "left_blocked": left_blocked, "right_blocked": right_blocked, "distance": distance, "threshold_cm": threshold_cm, "too_close": too_close, "emergency_brake": False, "obstacle": True, "stuck": True, "ultra_servo": "center"})
                stuck_counter = 0
                continue



            # Emergency reverse if ultrasonic sensor detects very close object (<10cm)
            if distance is not None and distance < 10.0:
                print("[ULTRA-EMERGENCY] Ultrasonic sensor detected object <10cm! Immediate brake and reverse.")
                speed = slow_speed
                pwm_ena.ChangeDutyCycle(speed)
                pwm_enb.ChangeDutyCycle(speed)
                brake()
                ws_broadcast_status({"direction": "brake", "speed": speed, "left_blocked": left_blocked, "right_blocked": right_blocked, "distance": distance, "threshold_cm": threshold_cm, "too_close": too_close, "emergency_brake": True, "obstacle": True})
                time.sleep(0.05)
                move_backward()
                ws_broadcast_status({"direction": "backward", "speed": speed, "left_blocked": left_blocked, "right_blocked": right_blocked, "distance": distance, "threshold_cm": threshold_cm, "too_close": too_close, "emergency_brake": True, "obstacle": True})
                time.sleep(back_seconds)
                brake()
                ws_broadcast_status({"direction": "brake", "speed": speed, "left_blocked": left_blocked, "right_blocked": right_blocked, "distance": distance, "threshold_cm": threshold_cm, "too_close": too_close, "emergency_brake": True, "obstacle": True})
                time.sleep(0.05)
                continue

            # Aggressive reaction to side IR sensors
            if left_blocked or right_blocked:
                print("[AGGRESSIVE] Side IR sensor triggered! Immediate brake and reverse.")
                speed = slow_speed
                pwm_ena.ChangeDutyCycle(speed)
                pwm_enb.ChangeDutyCycle(speed)
                brake()
                ws_broadcast_status({"direction": "brake", "speed": speed, "left_blocked": left_blocked, "right_blocked": right_blocked, "distance": distance, "threshold_cm": threshold_cm, "too_close": too_close, "emergency_brake": False, "obstacle": True})
                time.sleep(0.05)
                move_backward()
                ws_broadcast_status({"direction": "backward", "speed": speed, "left_blocked": left_blocked, "right_blocked": right_blocked, "distance": distance, "threshold_cm": threshold_cm, "too_close": too_close, "emergency_brake": False, "obstacle": True})
                time.sleep(back_seconds)
                brake()
                ws_broadcast_status({"direction": "brake", "speed": speed, "left_blocked": left_blocked, "right_blocked": right_blocked, "distance": distance, "threshold_cm": threshold_cm, "too_close": too_close, "emergency_brake": False, "obstacle": True})
                time.sleep(0.05)
                continue

            # Dynamic speed adjustment
            if too_close:
                speed = slow_speed
            else:
                speed = normal_speed
            pwm_ena.ChangeDutyCycle(speed)
            pwm_enb.ChangeDutyCycle(speed)

            # Emergency brake: stop and reverse if distance drops rapidly
            emergency_brake = False
            if prev_distance is not None and distance is not None:
                if prev_distance > threshold_cm and distance < (threshold_cm * 0.5):
                    print(f"[EMERGENCY BRAKE] Distance dropped from {prev_distance:.2f} to {distance:.2f}")
                    emergency_brake = True

            prev_distance = distance

            # Debug output
            print(f"[DEBUG] left_blocked={left_blocked}, right_blocked={right_blocked}, distance={distance}, threshold_cm={threshold_cm}, too_close={too_close}, emergency_brake={emergency_brake}, speed={speed}")

            # Broadcast status
            status = {
                "direction": "stopped",  # Will update below
                "speed": speed,
                "left_blocked": left_blocked,
                "right_blocked": right_blocked,
                "distance": distance,
                "threshold_cm": threshold_cm,
                "too_close": too_close,
                "emergency_brake": emergency_brake
            }

            stuck = False
            if emergency_brake:
                brake()
                status["direction"] = "brake"
                ws_broadcast_status(status)
                time.sleep(0.05)
                move_backward()
                status["direction"] = "backward"
                ws_broadcast_status(status)
                time.sleep(back_seconds)
                brake()
                status["direction"] = "brake"
                ws_broadcast_status(status)
                time.sleep(0.05)
                continue

            if too_close or left_blocked or right_blocked:
                status["obstacle"] = True
                print("[DEBUG] Obstacle detected: ", end="")
                if too_close:
                    print(f"distance {distance:.2f} < threshold {threshold_cm}", end="; ")
                if left_blocked:
                    print("left blocked", end="; ")
                if right_blocked:
                    print("right blocked", end="; ")
                print()
                brake()
                status["direction"] = "brake"
                ws_broadcast_status(status)
                time.sleep(0.05)

                if left_blocked and right_blocked:
                    print("[DEBUG] Both sides blocked, moving backward.")
                    move_backward()
                    status["direction"] = "backward"
                    ws_broadcast_status(status)
                    time.sleep(back_seconds)
                    brake()
                    status["direction"] = "brake"
                    ws_broadcast_status(status)
                    time.sleep(0.05)
                    stuck = True

                # Always sweep with ultrasonic sensor before turning
                sweep = None
                if camera_servo is not None:
                    print("[DEBUG] Performing ultrasonic sweep.")
                    sweep = sweep_distances(DEFAULT_ULTRA_SERVO_PIN, center_angle=90, delta_deg=40, samples=3, hold_s=0.12)
                    print(f"[DEBUG] Sweep results: {sweep}")

                if sweep:
                    fused_dir = choose_direction_fused(left_blocked, right_blocked, sweep, threshold_cm, prefer_left)
                    print(f"[DEBUG] Fused direction: {fused_dir}")
                    if fused_dir == 'left':
                        turn_left()
                        status["direction"] = "left"
                    elif fused_dir == 'right':
                        turn_right()
                        status["direction"] = "right"
                    else:
                        move_backward()
                        status["direction"] = "backward"
                    ws_broadcast_status(status)
                    time.sleep(turn_seconds)
                    brake()
                    status["direction"] = "brake"
                    ws_broadcast_status(status)
                    continue  # Skip normal turn logic

                turn = choose_turn(left_blocked, right_blocked, prefer_left)
                print(f"[DEBUG] Chosen turn: {turn}")
                prefer_left = not prefer_left

                # Count repeated turns
                if turn == last_turn:
                    turn_count += 1
                else:
                    turn_count = 1
                last_turn = turn

                # Corner escape: If stuck turning left/right, try backward
                if turn_count >= turn_repeat_limit:
                    print(f"[CORNER ESCAPE] {turn_count} repeated turns. Moving backward to escape corner.")
                    move_backward()
                    status["direction"] = "backward"
                    status["corner"] = True
                    ws_broadcast_status(status)
                    time.sleep(back_seconds)
                    brake()
                    status["direction"] = "brake"
                    status["corner"] = True
                    ws_broadcast_status(status)
                    time.sleep(0.05)
                    turn_count = 0  # Reset after backward
                    stuck = True

                if turn == 'left':
                    turn_left()
                    status["direction"] = "left"
                else:
                    turn_right()
                    status["direction"] = "right"
                ws_broadcast_status(status)
                time.sleep(turn_seconds)
                brake()
                status["direction"] = "brake"
                ws_broadcast_status(status)
            else:
                move_forward()
                status["direction"] = "forward"
                turn_count = 0  # Reset if moving forward
                ws_broadcast_status(status)

            time.sleep(loop_seconds)
    except KeyboardInterrupt:
        pass
    finally:
        brake()
        pwm_ena.stop()
        pwm_enb.stop()
        try:
            ultrasonic_sensor.cleanup()
        except Exception:
            pass
        GPIO.cleanup()


if __name__ == '__main__':
    main()
