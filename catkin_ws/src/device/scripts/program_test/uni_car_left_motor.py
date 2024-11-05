#!/usr/bin/env python

import pigpio

pi = pigpio.pi()

gpio_pin = 12  # 这里假设你选择的GPIO引脚是18
pi.set_mode(gpio_pin, pigpio.OUTPUT)
pi.set_PWM_frequency(gpio_pin, 1000)  # 设置PWM频率为1kHz，可以根据需要调整

def set_motor_speed(speed):
    duty_cycle = int(255 * speed)  # 根据需要将速度映射到PWM占空比范围内
    pi.set_PWM_dutycycle(gpio_pin, duty_cycle)


set_motor_speed(0.0)

pi.stop()