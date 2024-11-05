#!/usr/bin/env python
# -*- coding:utf-8 -*-

import pigpio
import time

# 初始化pigpio
pi = pigpio.pi()

# 定义编码器引脚
encoder_pin1 = 16
1
# 设置编码器引脚为输入
pi.set_mode(encoder_pin1, pigpio.INPUT)

# 初始化编码器状态
encoder_position = 0

def encoder_callback(gpio, level, tick):
    global encoder_position
    if level == 1:
        encoder_position += 1
    else:
        encoder_position -= 1

# 设置编码器引脚的下降沿触发硬件中断检测
cb = pi.callback(encoder_pin1, pigpio.FALLING_EDGE, encoder_callback)

try:
    while True:
        # 实时读取编码器位置
        print("Encoder Position: {}".format(encoder_position))
        time.sleep(0.1)

except KeyboardInterrupt:
    cb.cancel()
    pi.stop()
