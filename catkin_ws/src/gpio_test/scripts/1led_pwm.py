#!/usr/bin/env python
# -*- coding:utf-8 -*-

from gpiozero import PWMLED
from time import sleep
from enum import Enum


class LEDModule():
    def __init__(self):
        self.led_green = PWMLED(18)
        self.led_blue  = PWMLED(13)
        self.led_red   = PWMLED(12)
        self.led_value = 1.0

def main():
    node = LEDModule()


if __name__ == '__main__':
    main()

