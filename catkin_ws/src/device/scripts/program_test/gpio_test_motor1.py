#!/usr/bin/env python
# -*- coding:utf-8 -*-

from gpiozero import Button
from signal import pause

encoder_pinA = Button(16)
encoder_pinB = Button(13)

encoder_position = 0

def update_position():
    global encoder_position
    if encoder_pinA.is_pressed and encoder_pinB.is_pressed:
        encoder_position += 1
    elif not encoder_pinA.is_pressed and encoder_pinB.is_pressed:
        encoder_position -= 1

encoder_pinA.when_pressed = update_position
encoder_pinA.when_released = update_position
encoder_pinB.when_pressed = update_position
encoder_pinB.when_released = update_position

try:
    pause()
except KeyboardInterrupt:
    pass

print("Final Encoder Position: {}".format(encoder_position))
