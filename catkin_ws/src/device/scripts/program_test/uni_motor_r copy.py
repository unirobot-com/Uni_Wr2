#!/usr/bin/env python

import pigpio
import time
# import rospy
# from std_msgs.msg import Float32

class MotorControl:
    def __init__(self, encoder_pinA, encoder_pinB, pwm_pin1, pwm_pin2, ppr, wheel_diameter, gear_ratio):
        # self.node_name = rospy.init_node('uni_right_wheel_node', anonymous = False)

        self.pi = pigpio.pi()
        self.encoder_pinA = encoder_pinA
        self.encoder_pinB = encoder_pinB
        self.gpio_pwm1 = pwm_pin1
        self.gpio_pwm2 = pwm_pin2
        self.encoder_position = 0
        self.encoder_last_position = 0
        self.target_speed = 0.0
        self.current_speed = 0

        self.pi.set_mode(self.gpio_pwm1, pigpio.OUTPUT)
        self.pi.set_mode(self.gpio_pwm2, pigpio.OUTPUT)
        self.pi.set_PWM_frequency(self.gpio_pwm1, 1000)
        self.pi.set_PWM_frequency(self.gpio_pwm2, 1000)

        self.encoder_last_stateA = self.pi.read(self.encoder_pinA)
        self.encoder_last_stateB = self.pi.read(self.encoder_pinB)
        
        self.cbA = self.pi.callback(self.encoder_pinA, pigpio.EITHER_EDGE, self.encoder_callbackA)
        self.cbB = self.pi.callback(self.encoder_pinB, pigpio.EITHER_EDGE, self.encoder_callbackB)

        self.ppr = ppr
        self.gear_ratio = gear_ratio
        self.wheel_circumference = wheel_diameter * 3.14159 / 100.0

        self.calculate_motor_pid_rate = 50.0
        self.calculate_motor_pid_period = 1.0 / self.calculate_motor_pid_rate

        self.Kp = 1700
        self.Ki = 120
        self.Kd = 20
        self.bias = 0.0
        self.last_bias = 0.0
        self.motor_pwm = 0.0
        self.prev_error = 0
        self.once_cycle_pulse_coefficient = 1.27273
        self.once_cycle_pulse_count = (self.ppr * 2 * self.gear_ratio) / self.once_cycle_pulse_coefficient

        print("=====================================================")
        print("wheel_circumference:", self.wheel_circumference)
        print("once_cycle_pulse_count:", self.once_cycle_pulse_count)
        print("calculate_motor_pid_period:", self.calculate_motor_pid_period)
        print("=====================================================")

        # time.sleep(5)

        # self.uni_right_wheel_speed_sub = rospy.Subscriber('/uni_wheel/right', Float32, self.wheel_speed_callback)

    # def wheel_speed_callback(self, data):
    #     rospy.loginfo("receive data: %f", data.data)
    #     self.target_speed = data.data
    #     print("self.target_speed:", self.target_speed)

    """
    Encoder A phase external interrupt function.
    Used to calculate the numerical value 
    triggered by the external interrupt of encoder A phase.
    """
    def encoder_callbackA(self, gpio, level, tick):
        if level == 1 and self.encoder_last_stateA == 1 and self.encoder_last_stateB == 1:
            self.encoder_position -= 1
        elif level == 0 and self.encoder_last_stateA == 0 and self.encoder_last_stateB == 1:
            self.encoder_position += 1
        self.encoder_last_stateB = level

    """
    Encoder B phase external interrupt function.
    Used to calculate the numerical value 
    triggered by the external interrupt of encoder B phase.
    """
    def encoder_callbackB(self, gpio, level, tick):
        if level == 1 and self.encoder_last_stateA == 1 and self.encoder_last_stateB == 1:
            self.encoder_position += 1
        elif level == 0 and self.encoder_last_stateA == 0 and self.encoder_last_stateB == 1:
            self.encoder_position -= 1
        self.encoder_last_stateB = level

    """
    Calculate the current speed of the wheel 
    by reading the values of encoder A and B phases.
    return speed.(m/s)
    """
    def calculate_speed(self):
        delta_position = -self.encoder_position + self.encoder_last_position
        self.current_speed = ((delta_position / self.once_cycle_pulse_count) * self.wheel_circumference ) / self.calculate_motor_pid_period
        self.encoder_last_position = self.encoder_position
        return self.current_speed

    """
    PWM motor speed control function.
    Range 0~255.
    """
    def set_motor_speed(self, speed):
        duty_cycle = int(abs(speed)) # 将速度映射到PWM占空比范围内
        if duty_cycle >= 255:
            duty_cycle = 255
        if speed >= 0:
            self.pi.set_PWM_dutycycle(self.gpio_pwm1, duty_cycle)
            self.pi.set_PWM_dutycycle(self.gpio_pwm2, 0)
        else:
            self.pi.set_PWM_dutycycle(self.gpio_pwm2, duty_cycle)
            self.pi.set_PWM_dutycycle(self.gpio_pwm1, 0)

    """
    wheel stop.
    """
    def stop_motor(self):
        self.pi.set_PWM_dutycycle(self.gpio_pwm1, 0)
        self.pi.set_PWM_dutycycle(self.gpio_pwm2, 0)

    # 对抗机器人pid参数调节 self.Ki = 200
    def calculate_pid_section(self, current_speed, target_speed):
        self.bias = current_speed - target_speed
        kp_value = self.bias - self.last_bias
        self.motor_pwm -= self.Kp * kp_value + self.Ki * self.bias
        self.last_bias = self.bias
        return self.motor_pwm

    """
    PID output calculation.
    """
    def calculate_pid(self, current_speed, target_speed):
        self.bias = current_speed - target_speed
        kp_value = self.bias - self.last_bias
        kd_value = self.bias - 2 * self.last_bias + self.prev_error  # 添加微分项
        self.motor_pwm -= self.Kp * kp_value + self.Ki * self.bias + self.Kd * kd_value
        self.prev_error = self.bias
        self.last_bias = self.bias
        return self.motor_pwm


    def run(self):
        try:
            # rate = rospy.Rate(50)
            # while not rospy.is_shutdown():
            while True:
                currrent_speed = self.calculate_speed()
                output = self.calculate_pid(currrent_speed, self.target_speed)
                # output = self.calculate_pid_section(currrent_speed, self.target_speed)
                self.set_motor_speed(output)
                # num_str = "{:.3f}".format(currrent_speed)
                # integer_part, decimal_part = num_str.split('.')
                # print("current_speed:", integer_part + '.' + decimal_part, "                   target:", self.target_speed)
                time.sleep(self.calculate_motor_pid_period)
                # rate.sleep()
            self.stop_motor()
            self.cbA.cancel()
            self.cbB.cancel()
            self.pi.stop()
            print("exit")

        except KeyboardInterrupt:
            self.stop_motor()
            self.cbA.cancel()
            self.cbB.cancel()
            self.pi.stop()
            print("exit")


def main():
    left_motor = MotorControl(20, 19, 12, 18, 7, 5, 100)
    left_motor.target_speed = 0.08  # 设置目标速度为0.05 m/s
    left_motor.run()

if __name__ == '__main__':
    main()
