#!/usr/bin/env python3
"""
GPIO Motor Safety Init

Sets all motor control pins to OUTPUT LOW at boot to prevent uncontrolled
motion from floating inputs. The L298N ENA/ENB pins are often pulled HIGH
by default jumpers on breakout boards — floating IN pins then drive motors.

Run as a systemd oneshot service before robot bringup. Do NOT call
GPIO.cleanup() here — pins must stay as OUTPUT LOW until motor_controller
takes over.
"""
import RPi.GPIO as GPIO

# BCM pin numbers: ENABLE_A, IN1, IN2, ENABLE_B, IN3, IN4
MOTOR_PINS = [17, 27, 22, 13, 19, 26]

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

for pin in MOTOR_PINS:
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

print("Motor GPIO pins set to OUTPUT LOW")
