#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import time
import statistics
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
DEFAULT_ULTRA_SERVO_PIN = 16


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--threshold-cm', type=float, default=30.0)
    parser.add_argument('--speed', type=int, default=45)
    parser.add_argument('--loop-seconds', type=float, default=0.05)
    parser.add_argument('--back-seconds', type=float, default=0.20)
    parser.add_argument('--turn-seconds', type=float, default=0.30)
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
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(TRIG_PIN, GPIO.LOW)

    start_wait = time.time()
    while GPIO.input(ECHO_PIN) == GPIO.LOW:
        if time.time() - start_wait > timeout_seconds:
            return None

    pulse_start = time.time()
    while GPIO.input(ECHO_PIN) == GPIO.HIGH:
        if time.time() - pulse_start > timeout_seconds:
            return None

    pulse_end = time.time()
    return (pulse_end - pulse_start) * 17150.0


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
    args = build_parser().parse_args()
    speed = max(20, min(100, int(args.speed)))
    loop_seconds = max(0.02, float(args.loop_seconds))
    threshold_cm = max(10.0, float(args.threshold_cm))
    back_seconds = max(0.05, float(args.back_seconds))
    turn_seconds = max(0.05, float(args.turn_seconds))

    pwm_ena, pwm_enb = setup(speed)
    prefer_left = True

    try:
        # Track repeated turns
        last_turn = None
        turn_count = 0
        turn_repeat_limit = 3  # Move backward after 3 repeated turns
        sweep_enabled = True  # Enable sweep logic

        while True:
            left_blocked = is_blocked_left()
            right_blocked = is_blocked_right()
            distance = measure_distance_cm()
            too_close = distance is not None and distance < threshold_cm

            stuck = False
            if too_close or left_blocked or right_blocked:
                brake()
                time.sleep(0.05)

                if left_blocked and right_blocked:
                    move_backward()
                    time.sleep(back_seconds)
                    brake()
                    time.sleep(0.05)
                    stuck = True

                turn = choose_turn(left_blocked, right_blocked, prefer_left)
                prefer_left = not prefer_left

                # Count repeated turns
                if turn == last_turn:
                    turn_count += 1
                else:
                    turn_count = 1
                last_turn = turn

                # If stuck turning, try backward
                if turn_count >= turn_repeat_limit:
                    move_backward()
                    time.sleep(back_seconds)
                    brake()
                    time.sleep(0.05)
                    turn_count = 0  # Reset after backward
                    stuck = True

                # Sweep ultrasonic sensor if stuck
                if sweep_enabled and stuck and camera_servo is not None:
                    sweep = sweep_distances(DEFAULT_ULTRA_SERVO_PIN, center_angle=90, delta_deg=40, samples=3, hold_s=0.12)
                    if sweep:
                        fused_dir = choose_direction_fused(left_blocked, right_blocked, sweep, threshold_cm, prefer_left)
                        if fused_dir == 'left':
                            turn_left()
                        elif fused_dir == 'right':
                            turn_right()
                        else:
                            move_backward()
                        time.sleep(turn_seconds)
                        brake()
                        continue  # Skip normal turn logic

                if turn == 'left':
                    turn_left()
                else:
                    turn_right()
                time.sleep(turn_seconds)
                brake()
            else:
                move_forward()
                turn_count = 0  # Reset if moving forward

            time.sleep(loop_seconds)
    except KeyboardInterrupt:
        pass
    finally:
        brake()
        pwm_ena.stop()
        pwm_enb.stop()
        GPIO.cleanup()


if __name__ == '__main__':
    main()
