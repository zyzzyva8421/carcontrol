#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import sys
import time

# Prefer gpiozero + pigpio pin factory when available to reduce servo jitter.
try:
    from gpiozero import Device
    from gpiozero.pins.pigpio import PiGPIOFactory
    Device.pin_factory = PiGPIOFactory()
except Exception:
    # pigpio or gpiozero not available; fall back to whatever backend is installed
    pass

DEFAULT_SERVO_PINS = {
    1: 16,  # J1: Ultrasonic (IO23)
    2: 23,  # J2: Camera Pan (IO11)
    3: 21,   # J3: Camera Tilt (IO9)
}


def clamp(value, low, high):
    return max(low, min(high, value))


def angle_to_duty(angle):
    return 2.5 + 10.0 * angle / 180.0


def angle_to_pulse_us(angle):
    return int(500 + (2000.0 * angle / 180.0))


def _set_servo_pigpio(pin, angle, hold_s):
    import pigpio

    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon not connected")

    pulse = angle_to_pulse_us(angle)
    pi.set_mode(pin, pigpio.OUTPUT)
    pi.set_servo_pulsewidth(pin, pulse)
    time.sleep(hold_s)
    pi.set_servo_pulsewidth(pin, 0)
    pi.stop()


def _set_servo_gpiozero(pin, angle, hold_s):
    from gpiozero import AngularServo

    servo = AngularServo(
        pin,
        min_angle=0,
        max_angle=180,
        min_pulse_width=0.0005,
        max_pulse_width=0.0025,
        frame_width=0.02,
    )
    try:
        servo.angle = angle
        time.sleep(hold_s)
        servo.detach()
    finally:
        servo.close()


def _set_servo_rpigpio(pin, angle, hold_s):
    import RPi.GPIO as GPIO

    duty = angle_to_duty(angle)
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(pin, GPIO.OUT)

    pwm = GPIO.PWM(pin, 50)
    pwm.start(0)
    try:
        for _ in range(15):
            pwm.ChangeDutyCycle(duty)
            time.sleep(0.02)
        time.sleep(max(0.0, hold_s - 0.30))
        pwm.ChangeDutyCycle(0)
        time.sleep(0.05)
    finally:
        pwm.stop()
        GPIO.cleanup(pin)


def set_servo_angle(pin, angle, backend, hold_s):
    if backend == "pigpio":
        _set_servo_pigpio(pin, angle, hold_s)
        return "pigpio"
    if backend == "gpiozero":
        _set_servo_gpiozero(pin, angle, hold_s)
        return "gpiozero"
    if backend == "rpi":
        _set_servo_rpigpio(pin, angle, hold_s)
        return "rpi"

    failures = []
    for candidate in ("pigpio", "gpiozero", "rpi"):
        try:
            set_servo_angle(pin, angle, candidate, hold_s)
            return candidate
        except Exception as exc:
            failures.append("%s:%s" % (candidate, exc))

    raise RuntimeError("all backends failed: %s" % ", ".join(failures))


def auto_correct_camera(backend="auto", hold_ms=300, pins=None, horizontal_servo=1, vertical_servo=3):
    """Auto-correct camera direction by centering horizontal and vertical servos.

    - `horizontal_servo` and `vertical_servo` are the servo indices (1..3).
    - By default this will set both servos to 90 degrees (straight ahead).
    - `pins` may be provided to override `DEFAULT_SERVO_PINS`.
    - Returns a dict with results for each servo.
    """
    pins = pins if pins is not None else DEFAULT_SERVO_PINS

    if horizontal_servo not in pins or vertical_servo not in pins:
        raise ValueError("servo index not found in pins mapping")

    horiz_pin = pins[horizontal_servo]
    vert_pin = pins[vertical_servo]

    angle = 90.0
    hold_s = max(0.05, float(hold_ms) / 1000.0)

    result = {}
    used_h = set_servo_angle(horiz_pin, angle, backend, hold_s)
    result["horizontal"] = {"servo": horizontal_servo, "pin": horiz_pin, "angle": angle, "backend": used_h}

    used_v = set_servo_angle(vert_pin, angle, backend, hold_s)
    result["vertical"] = {"servo": vertical_servo, "pin": vert_pin, "angle": angle, "backend": used_v}

    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--angle", type=float, help="Servo angle in degrees (0-180)")
    parser.add_argument("--servo", type=int, choices=[1, 2, 3], default=1, help="Servo channel index (1, 2, or 3)")
    parser.add_argument("--pin", type=int, default=None, help="BCM GPIO pin override")
    parser.add_argument("--backend", choices=["auto", "pigpio", "gpiozero", "rpi"], default="auto", help="PWM backend")
    parser.add_argument("--hold-ms", type=int, default=300, help="Hold PWM for this many milliseconds")
    parser.add_argument("--center", action="store_true", help="Center camera servos J1 (horizontal) and J3 (vertical)")
    args = parser.parse_args()
    if args.center:
        try:
            results = auto_correct_camera(args.backend, args.hold_ms)
            sys.stdout.write("ok center results=%s\n" % (results,))
            sys.stderr.write("[DEBUG] centered camera: %s\n" % (results,))
            return
        except Exception as exc:
            sys.stderr.write("[ERROR] center failed: %s\n" % (exc,))
            raise SystemExit(1)

    if args.angle is None:
        parser.error("--angle is required unless --center is used")

    angle = clamp(args.angle, 0.0, 180.0)
    pin = args.pin if args.pin is not None else DEFAULT_SERVO_PINS[args.servo]

    sys.stderr.write("[DEBUG] Requested servo: %s, pin: %s, angle: %s, backend: %s, hold_ms: %s\n" % (args.servo, pin, angle, args.backend, args.hold_ms))

    if pin < 0 or pin > 27:
        sys.stderr.write("[ERROR] Invalid BCM pin: %s\n" % (pin,))
        raise SystemExit("Invalid BCM pin: %s" % (pin,))

    hold_s = max(0.05, float(args.hold_ms) / 1000.0)
    try:
        used = set_servo_angle(pin, angle, args.backend, hold_s)
        sys.stderr.write("[DEBUG] ok backend=%s pin=%s angle=%s\n" % (used, pin, angle))
        sys.stdout.write("ok backend=%s pin=%s angle=%s\n" % (used, pin, angle))
    except Exception as exc:
        sys.stderr.write("[ERROR] servo control failed: %s\n" % (exc,))
        sys.stderr.write("servo control failed: %s\n" % (exc,))
        raise SystemExit(1)


if __name__ == "__main__":
    main()
