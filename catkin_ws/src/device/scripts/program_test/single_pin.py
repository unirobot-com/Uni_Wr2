#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time

hall_pin = 16
count = 0
GPIO.setmode(GPIO.BCM)
GPIO.setup(hall_pin, GPIO.IN)

def callback(channel):
    global count
    count += 1
    print(count)

# GPIO.add_event_detect(hall_pin, GPIO.FALLING, callback=callback)
GPIO.add_event_detect(hall_pin, GPIO.BOTH, callback=callback)

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup()