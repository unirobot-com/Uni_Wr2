#!/usr/bin/env python
# -*- coding:utf-8 -*-

from gpiozero import Button, PWMOutputDevice
from signal import pause
import threading
from time import sleep

class EncoderMotorHandler:
    def __init__(self, pin_a, pin_b, motor_pin1, motor_pin2):
        self.pin_a = Button(pin_a, pull_up=False)
        self.pin_b = Button(pin_b, pull_up=False)
        self.position = 0
        self.motor = PWMOutputDevice(motor_pin1)
        self.motor2 = PWMOutputDevice(motor_pin2)

        self.pin_a.when_pressed = self.handle_movement
        self.pin_a.when_released = self.handle_movement
        self.pin_b.when_pressed = self.handle_movement
        self.pin_b.when_released = self.handle_movement

    def handle_movement(self):
        a_pressed = self.pin_a.is_pressed
        b_pressed = self.pin_b.is_pressed
        if a_pressed and not b_pressed:
            self.position += 1
        elif b_pressed and not a_pressed:
            self.position -= 1

        print(f"Current position: {self.position}")

    def run(self):
        print("Monitoring encoder movement. Press CTRL+C to exit.")
        threading.Thread(target=self.run_motor).start()
        pause()

    def run_motor(self):
        while True:
            self.motor.value = 0.5  # 控制电机转速
            self.motor2.value = 0.5  # 控制电机转速
            sleep(0.1)  # 添加延迟

# 使用示例
if __name__ == "__main__":
    encoder_motor_handler = EncoderMotorHandler(16, 13, 5, 6)
    encoder_motor_handler.run()
