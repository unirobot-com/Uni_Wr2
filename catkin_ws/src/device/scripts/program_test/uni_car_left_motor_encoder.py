#!/usr/bin/env python

import pigpio
import time

class MotorControl:
    def __init__(self, encoder_pinA, encoder_pinB, pwm_pin1, pwm_pin2):
        self.pi = pigpio.pi()
        self.encoder_pinA = encoder_pinA
        self.encoder_pinB = encoder_pinB
        self.gpio_pwm1 = pwm_pin1
        self.gpio_pwm2 = pwm_pin2
        self.encoder_position = 0

        self.pi.set_mode(self.gpio_pwm1, pigpio.OUTPUT)
        self.pi.set_mode(self.gpio_pwm2, pigpio.OUTPUT)
        self.pi.set_PWM_frequency(self.gpio_pwm1, 1000)
        self.pi.set_PWM_frequency(self.gpio_pwm2, 1000)

        self.encoder_last_stateA = self.pi.read(self.encoder_pinA)
        self.encoder_last_stateB = self.pi.read(self.encoder_pinB)
        
        self.cbA = self.pi.callback(self.encoder_pinA, pigpio.EITHER_EDGE, self.encoder_callbackA)
        self.cbB = self.pi.callback(self.encoder_pinB, pigpio.EITHER_EDGE, self.encoder_callbackB)
    
    def encoder_callbackA(self, gpio, level, tick):
        if level == 1 and self.encoder_last_stateA == 1 and self.encoder_last_stateB == 1:
            self.encoder_position -= 1
        elif level == 0 and self.encoder_last_stateA == 0 and self.encoder_last_stateB == 1:
            self.encoder_position += 1
        self.encoder_last_stateB = level

    def encoder_callbackB(self, gpio, level, tick):
        if level == 1 and self.encoder_last_stateA == 1 and self.encoder_last_stateB == 1:
            self.encoder_position += 1
        elif level == 0 and self.encoder_last_stateA == 0 and self.encoder_last_stateB == 1:
            self.encoder_position -= 1
        self.encoder_last_stateB = level

    def set_motor_speed(self, speed):
        duty_cycle = int(abs(speed) * 255)  # 将速度映射到PWM占空比范围内
        if speed >= 0:
            self.pi.set_PWM_dutycycle(self.gpio_pwm1, duty_cycle)
            self.pi.set_PWM_dutycycle(self.gpio_pwm2, 0)
        else:
            self.pi.set_PWM_dutycycle(self.gpio_pwm2, duty_cycle)
            self.pi.set_PWM_dutycycle(self.gpio_pwm1, 0)

    def stop_motor(self):
        self.pi.set_PWM_dutycycle(self.gpio_pwm1, 0)
        self.pi.set_PWM_dutycycle(self.gpio_pwm2, 0)

    def run(self):
        try:
            self.set_motor_speed(0)
            while True:
                print(self.encoder_position)
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.stop_motor()
            self.cbA.cancel()
            self.cbB.cancel()
            self.pi.stop()

def main():
    left_motor = MotorControl(20, 19, 12, 18)
    left_motor.run()

if __name__ == '__main__':
    main()
