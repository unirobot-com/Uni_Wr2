#!/usr/bin/env python
# -*- coding:utf-8 -*-

import pigpio
import time
from simple_pid import PID

# 初始化pigpio
pi = pigpio.pi()

# 定义电机引脚
motor_pin1 = 5
motor_pin2 = 6

# 设置电机引脚为输出
pi.set_mode(motor_pin1, pigpio.OUTPUT)
pi.set_mode(motor_pin2, pigpio.OUTPUT)

# 设置PWM
pwm_pin = 5
pi.set_mode(pwm_pin, pigpio.OUTPUT)

# 初始化PID控制器
pid = PID(1, 0.1, 0.05, setpoint=0)  # 设置PID参数和目标位置

try:
    while True:
        # 获取当前位置（假设为编码器读数）
        current_position = 0  # 这里需要替换为实际获取的位置值

        # 计算PID输出
        output = pid(current_position)

        # 控制电机转动
        if output > 0:
            pi.write(motor_pin1, 1)
            pi.write(motor_pin2, 0)
        else:
            pi.write(motor_pin1, 0)
            pi.write(motor_pin2, 1)
        
        # 设置PWM占空比
        pi.set_PWM_dutycycle(pwm_pin, abs(output))

        # 实时记录位置
        print("Current Position: {}".format(current_position))
        time.sleep(0.1)

except KeyboardInterrupt:
    pi.set_PWM_dutycycle(pwm_pin, 0)  # 停止PWM
    pi.stop()
