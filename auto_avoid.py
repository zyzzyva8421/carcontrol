#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import time
import RPi.GPIO as GPIO

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


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--threshold-cm', type=float, default=30.0)
    parser.add_argument('--speed', type=int, default=45)
    parser.add_argument('--loop-seconds', type=float, default=0.05)
    parser.add_argument('--back-seconds', type=float, default=0.20)
    parser.add_argument('--turn-seconds', type=float, default=0.30)
    return parser


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
        while True:
            left_blocked = is_blocked_left()
            right_blocked = is_blocked_right()
            distance = measure_distance_cm()
            too_close = distance is not None and distance < threshold_cm

            if too_close or left_blocked or right_blocked:
                brake()
                time.sleep(0.05)

                if left_blocked and right_blocked:
                    move_backward()
                    time.sleep(back_seconds)
                    brake()
                    time.sleep(0.05)

                turn = choose_turn(left_blocked, right_blocked, prefer_left)
                prefer_left = not prefer_left
                if turn == 'left':
                    turn_left()
                else:
                    turn_right()
                time.sleep(turn_seconds)
                brake()
            else:
                move_forward()

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
