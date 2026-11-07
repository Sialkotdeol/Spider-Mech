#!/usr/bin/env python3
"""
Move one servo straight from the Pi, with no server involved.

Stop uvicorn first (Ctrl+C), or the two will fight over the I2C bus.

    cd ~/Desktop/spider_mech
    .venv/bin/python servo_test.py 0x40 1      # L1 femur (channel 1)
    .venv/bin/python servo_test.py 0x40 0      # L1 coxa, for comparison
    .venv/bin/python servo_test.py 0x41 1      # R1 femur (channel 17)

Channel numbers in config.py: 0-15 are board 0x40 pins 0-15,
16-31 are board 0x41 pins 0-15 (so channel 17 = 0x41 pin 1).
"""
import sys
import time

import board
import busio
from adafruit_pca9685 import PCA9685

MIN_US, MAX_US, FREQ = 500, 2500, 50


def duty(angle):
    pulse = MIN_US + angle / 180.0 * (MAX_US - MIN_US)
    return int(pulse / (1_000_000 / FREQ) * 0xFFFF)


address = int(sys.argv[1], 0) if len(sys.argv) > 1 else 0x40
pin = int(sys.argv[2]) if len(sys.argv) > 2 else 1

pca = PCA9685(busio.I2C(board.SCL, board.SDA), address=address)
pca.frequency = FREQ
print(f"board 0x{address:02X} pin {pin}: prescale {pca.prescale_reg} "
      f"(121 = 50 Hz), frequency {pca.frequency:.1f} Hz")

channel = pca.channels[pin]
try:
    for cycle in range(3):
        for angle in (60, 120):
            channel.duty_cycle = duty(angle)
            print(f"  -> servo {angle} deg")
            time.sleep(2)
    channel.duty_cycle = duty(90)
    print("  -> servo 90 deg (centre)")
    time.sleep(1.5)
finally:
    channel.duty_cycle = 0      # release the servo
    print("done - servo released")
