#!/usr/bin/env python
# -*- coding:utf-8 -*-

import pigpio
import time

# 初始化pigpio
pi = pigpio.pi()

# 定义编码器引脚
encoder_pinA = 13
encoder_pinB = 16

# 设置编码器引脚为输入
pi.set_mode(encoder_pinA, pigpio.INPUT)
pi.set_mode(encoder_pinB, pigpio.INPUT)

# pi.set_pull_up_down(encoder_pinA, pigpio.PUD_DOWN)
# pi.set_pull_up_down(encoder_pinB, pigpio.PUD_DOWN)
# 初始化编码器状态
encoder_position = 0
encoder_last_stateA = pi.read(encoder_pinA)
encoder_last_stateB = pi.read(encoder_pinB)

def encoder_callbackA(gpio, level, tick):
    global encoder_position, encoder_last_stateA, encoder_last_stateB
    if level == 1 and encoder_last_stateA == 0 and encoder_last_stateB == 0:
        encoder_position += 1
    elif level == 0 and encoder_last_stateA == 1 and encoder_last_stateB == 0:
        encoder_position -= 1
    encoder_last_stateA = level

def encoder_callbackB(gpio, level, tick):
    global encoder_position, encoder_last_stateA, encoder_last_stateB
    if level == 1 and encoder_last_stateA == 1 and encoder_last_stateB == 1:
        encoder_position += 1
    elif level == 0 and encoder_last_stateA == 0 and encoder_last_stateB == 1:
        encoder_position -= 1
    encoder_last_stateB = level

# def encoder_callbackB(gpio, level, tick):
#     global encoder_position, encoder_last_stateA, encoder_last_stateB
#     if level == 1 and encoder_last_stateA == 1 and encoder_last_stateB == 1:
#         encoder_position += 1
#     elif level == 0 and encoder_last_stateA == 0 and encoder_last_stateB == 1:
#         encoder_position -= 1
#     encoder_last_stateB = level

# 设置编码器引脚的中断检测
# cbA = pi.callback(encoder_pinA, pigpio.EITHER_EDGE, encoder_callbackA) #不行
# cbA = pi.callback(encoder_pinA, pigpio.FALLING_EDGE, encoder_callbackA) #不行
cbB = pi.callback(encoder_pinB, pigpio.EITHER_EDGE, encoder_callbackB)  
# cbB = pi.callback(encoder_pinB, pigpio.FALLING_EDGE, encoder_callbackB)  

try:
    while True:
        # 实时读取编码器位置
        print("Encoder Position: {}".format(encoder_position))
        time.sleep(0.1)

except KeyboardInterrupt:
    # cbA.cancel()
    cbB.cancel()
    pi.stop()
