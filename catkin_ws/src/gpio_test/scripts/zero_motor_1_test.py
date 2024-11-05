#!/usr/bin/env python
# -*- coding:utf-8 -*-

from gpiozero import Button, PWMOutputDevice
from signal import pause

class EncoderMotorHandler:
    def __init__(self, pin_a, pin_b, pin_pwm_forward, pin_pwm_reverse):
        # 编码器的初始化
        self.pin_a = Button(pin_a, pull_up=False) # 假设需要下拉到GND
        self.pin_b = Button(pin_b, pull_up=False) # 假设需要下拉到GND
        self.position = 0

        # 正转和反转PWM电机的初始化
        self.motor_pwm_forward = PWMOutputDevice(pin_pwm_forward)
        self.motor_pwm_reverse = PWMOutputDevice(pin_pwm_reverse)

        # 使用硬件中断来处理编码器的信号变化
        self.pin_a.when_pressed = self.handle_movement
        self.pin_a.when_released = self.handle_movement
        self.pin_b.when_pressed = self.handle_movement
        self.pin_b.when_released = self.handle_movement

    def set_motor_speed(self, speed, direction):
        if direction == 1:
            self.motor_pwm_forward.value = speed
            self.motor_pwm_reverse.value = 0
        elif direction == -1:
            self.motor_pwm_forward.value = 0
            self.motor_pwm_reverse.value = speed

    def handle_movement(self):
        a_pressed = self.pin_a.is_pressed
        b_pressed = self.pin_b.is_pressed
        if a_pressed and not b_pressed:
            self.position += 1
        elif b_pressed and not a_pressed:
            self.position -= 1

        print(f"Current position: {self.position}")

    def run(self):
        print("Monitoring encoder movement and controlling motor. Press CTRL+C to exit.")
        pause()

# 使用示例
if __name__ == "__main__":
    # 假设正转PWM输出为GPIO 18，反转PWM输出为GPIO 23
    encoder_motor_handler = EncoderMotorHandler(16, 13, 5, 6)
    # 设置中等速度和正转方向
    encoder_motor_handler.set_motor_speed(0.2, 1)
    encoder_motor_handler.run()