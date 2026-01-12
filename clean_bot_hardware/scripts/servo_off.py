#!/usr/bin/env python3
from gpiozero import Servo
from time import sleep

# GPIO 18 (Pin 12) is the hardware PWM pin on the Pi
servo = Servo(18)

try:
    # Set to min position (Off)
    servo.min()
    sleep(1.0)
    # Important: Detach or set to None to stop the PWM signal and jitter
    servo.value = None
except KeyboardInterrupt:
    pass
