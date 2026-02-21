# ultrasonic_sensor.py
# Simple HC-SR04 ultrasonic sensor reading for Raspberry Pi
# Usage: import ultrasonic_sensor; dist = ultrasonic_sensor.get_distance_cm()

import RPi.GPIO as GPIO
import time

TRIG_PIN = 1  # Update as needed
ECHO_PIN = 0  # Update as needed

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

def get_distance_cm():
    # Send 10us pulse to trigger
    GPIO.output(TRIG_PIN, False)
    time.sleep(0.02)
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.000015)
    GPIO.output(TRIG_PIN, False)

    timeout = 0.03  # 30ms timeout for echo
    start = time.time()
    # Wait for echo to go high
    while GPIO.input(ECHO_PIN) == 0:
        if time.time() - start > timeout:
            return None
    pulse_start = time.time()
    # Wait for echo to go low
    while GPIO.input(ECHO_PIN) == 1:
        if time.time() - pulse_start > timeout:
            return None
    pulse_end = time.time()
    distance = (pulse_end - pulse_start) * 17150.0  # cm
    if distance <= 2 or distance >= 400:
        return None
    return distance

def cleanup():
    GPIO.cleanup()
