#!/usr/bin/env python
# -*- coding:utf-8 -*-


from gpiozero import Button, PWMOutputDevice
from signal import pause
import threading

class EncoderMotorHandler:
    def __init__(self, pin_a, pin_b, pin_pwm_forward, pin_pwm_reverse):
        self.pin_a = Button(pin_a, pull_up=False)
        self.pin_b = Button(pin_b, pull_up=False)
        self.position = 0
        self.motor_pwm_forward = PWMOutputDevice(pin_pwm_forward)
        self.motor_pwm_reverse = PWMOutputDevice(pin_pwm_reverse)

        self.pin_a.when_pressed = self.handle_movement
        self.pin_a.when_released = self.handle_movement
        self.pin_b.when_pressed = self.handle_movement
        self.pin_b.when_released = self.handle_movement

        self.lock = threading.Lock()

    def set_motor_speed(self, speed, direction):
        if direction == 1:
            self.motor_pwm_forward.value = speed
            self.motor_pwm_reverse.value = 0
        elif direction == -1:
            self.motor_pwm_forward.value = 0
            self.motor_pwm_reverse.value = speed

    def handle_movement(self):
        with self.lock:
            a_pressed = self.pin_a.is_pressed
            b_pressed = self.pin_b.is_pressed
            if a_pressed and not b_pressed:
                self.position += 1
            elif b_pressed and not a_pressed:
                self.position -= 1

            print(f"Current position: {self.position}")

    def run(self):
        print("Monitoring encoder movement and controlling motor. Press CTRL+C to exit.")
        # threading.Thread(target=self.run_motor).start()
        pause()

    def run_motor(self):
        while True:
            with self.lock:
                # 在这里添加控制电机的代码
                pass

# 使用示例
if __name__ == "__main__":
    encoder_motor_handler = EncoderMotorHandler(16, 13, 5, 6)
    encoder_motor_handler.set_motor_speed(0.2, 1)
    encoder_motor_handler.run()

