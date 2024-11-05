#!/usr/bin/env python
# -*- coding:utf-8 -*-

from gpiozero import LED # 引入LED类
from time import sleep

red = LED(18) # 声明LED对象red，该对象连接至树莓派GPIO17引脚

while True: # 使下面程序持续循环运行
    try:
        red.on() # 点亮LED
        sleep(1) # 延时1秒
        red.off() # 熄灭LED
        sleep(1) # 延时1秒
    except KeyboardInterrupt:
        break
    
