#!/usr/bin/env python

import pigpio
import time
import rospy
from std_msgs.msg import Float32, Float32MultiArray

class MotorControl:
    def __init__(self, encoder_pinA, encoder_pinB, pwm_pin1, pwm_pin2, ppr, wheel_diameter, gear_ratio):
        self.encoder_pinA = encoder_pinA                           #定义编码器A相引脚
        self.encoder_pinB = encoder_pinB                           #定义编码器B相引脚
        self.gpio_pwm1 = pwm_pin1                                  #定义编码电机的电机引脚1
        self.gpio_pwm2 = pwm_pin2                                  #定义编码电机的电机引脚2
        self.encoder_position = 0                                  #定义保存编码器A，B相的数值变量
        self.encoder_last_position = 0                             #定义保存编码器A，B相上一次数值的变量
        self.target_speed = 0.0                                    #定义编码电机要达到的目标速度的变量
        self.current_speed = 0                                     #定义编码电机当前速度的变量
        self.current_speed_pub = Float32()

        #设定编码器引脚模式，电机引脚模式，外部中断函数
        self.pi = pigpio.pi()
        self.pi.set_mode(self.gpio_pwm1, pigpio.OUTPUT)            #设定编码电机引脚1为输出模式
        self.pi.set_mode(self.gpio_pwm2, pigpio.OUTPUT)            #设定编码电机引脚2为输出模式
        self.pi.set_PWM_frequency(self.gpio_pwm1, 1000)            #定义编码电机引脚1的频率
        self.pi.set_PWM_frequency(self.gpio_pwm2, 1000)            #定义编码电机引脚2的频率
        self.encoder_last_stateA = self.pi.read(self.encoder_pinA) #获取编码器A相引脚的状态
        self.encoder_last_stateB = self.pi.read(self.encoder_pinB) #获取编码器B相引脚的状态
        self.cbA = self.pi.callback(self.encoder_pinA, pigpio.EITHER_EDGE, self.encoder_callbackA)
        self.cbB = self.pi.callback(self.encoder_pinB, pigpio.EITHER_EDGE, self.encoder_callbackB)

        self.ppr = ppr                                                #设定编码电机线圈数
        self.gear_ratio = gear_ratio                                  #保存减速电机减速比
        self.calculate_pid_rate = 50.0                                  #设定程序运行频率
        self.calculate_pid_period = 1.0 / 50.0                          #计算pid程序周期
        self.wheel_circumference = wheel_diameter * 3.14159 / 100.0            #轮子周长
        self.Kp = 1700                                                          #Kp的值
        self.Ki = 120                                                           #Ki的值
        self.Kd = 20                                                            #Kd的值
        self.bias = 0.0                                                    #保存本次误差
        self.last_bias = 0.0                                               #保存上次误差
        self.motor_pwm = 0.0                                         #保存电机pwm输出结果
        self.prev_error = 0.0                                        #保存上上次记录的误差
        self.factor = 1.27273                  #比例因子，实现编码器一圈理论值和实际值达到一致
        self.encoder_theoretical = self.ppr * 2 * self.gear_ratio    #计算编码器一圈理论值
        self.encoder_actual = self.encoder_theoretical / self.factor #计算编码器一圈实际值


        rospy.init_node('uni_car_left_wheel_node', anonymous=True)
        self.PID_DEBUG = False

        if self.PID_DEBUG:
            print("=====================================================")
            print("wheel_circumference:", self.wheel_circumference)
            print("encoder_actual:", self.encoder_actual)
            print("=====================================================")

        print('\033[32m[Left motor start]\033[0m')

    def set_left_pid_callback(self, msg):
        if len(msg.data) == 3:
            self.Kp = msg.data[0]
            self.Ki = msg.data[1]
            self.Kd = msg.data[2]
            print(self.Kp, self.Ki, self.Kd)
        else:
            print("error")

    #Get the speed of the left wheel target by subscribing to the topic
    def speed_callback(self, data):
        self.target_speed = data.data


    #Encoder A phase external interrupt function.
    #Used to calculate the numerical value triggered by the external interrupt of encoder A phase.
    def encoder_callbackA(self, gpio, level, tick):
        if level == 1 and self.encoder_last_stateA == 1 and self.encoder_last_stateB == 1:
            self.encoder_position -= 1
        elif level == 0 and self.encoder_last_stateA == 0 and self.encoder_last_stateB == 1:
            self.encoder_position += 1
        self.encoder_last_stateB = level


    #Encoder B phase external interrupt function.
    #Used to calculate the numerical value triggered by the external interrupt of encoder B phase.
    def encoder_callbackB(self, gpio, level, tick):
        if level == 1 and self.encoder_last_stateA == 1 and self.encoder_last_stateB == 1:
            self.encoder_position += 1
        elif level == 0 and self.encoder_last_stateA == 0 and self.encoder_last_stateB == 1:
            self.encoder_position -= 1
        self.encoder_last_stateB = level

    #Keep the wheels in a stopped state
    def stop_motor(self):
        self.pi.set_PWM_dutycycle(self.gpio_pwm1, 0)
        self.pi.set_PWM_dutycycle(self.gpio_pwm2, 0)


    def run(self):
        try:
            rate = rospy.Rate(50)
            speed_count = 0
            while not rospy.is_shutdown():
                speed_count = speed_count + 1
                if speed_count >=2:
                    speed_count = 0
                    print("encoder_l:", self.encoder_position)
                rate.sleep()
            self.stop_motor()
            self.cbA.cancel()
            self.cbB.cancel()
            self.pi.stop()
            print("left_wheel program exit")

        except KeyboardInterrupt:
            self.stop_motor()
            self.cbA.cancel()
            self.cbB.cancel()
            self.pi.stop()
            print("wheel_left program exit")


def main():
    motor = MotorControl(16, 13, 5, 6, 7, 5, 100)
    motor.run()

if __name__ == '__main__':
    main()
