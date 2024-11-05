#!/usr/bin/env python

import pigpio
import time
import rospy
import math
import tf
from std_msgs.msg import Float32, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Quaternion, Vector3, Polygon
from turtlesim.msg import Pose

class MotorControl:
    def __init__(self, encoder_pin1A, encoder_pin2B, pwm_pin1, pwm_pin2,
                encoder_pin3A, encoder_pin4B, pwm_pin3, pwm_pin4, ppr, wheel_diameter, wheel_length, gear_ratio):
        #==============================================================================================
        self.encoder_pin1A = encoder_pin1A                          #定义编码器A相引脚
        self.encoder_pin2B = encoder_pin2B                          #定义编码器B相引脚
        self.gpio_pwm1 = pwm_pin1                                   #定义编码电机的电机引脚1
        self.gpio_pwm2 = pwm_pin2                                   #定义编码电机的电机引脚2
        self.encoder_position1 = 0                                  #定义保存编码器A，B相的数值变量
        self.encoder_last_position1 = 0                             #定义保存编码器A，B相上一次数值的变量
        self.target_speed1 = 0.0                                    #定义编码电机要达到的目标速度的变量
        self.current_speed1 = 0                                     #定义编码电机当前速度的变量
        self.current_speed_pub1 = Float32()

        #设定编码器引脚模式，电机引脚模式，外部中断函数
        self.pi = pigpio.pi()
        self.pi.set_mode(self.gpio_pwm1, pigpio.OUTPUT)            #设定编码电机引脚1为输出模式
        self.pi.set_mode(self.gpio_pwm2, pigpio.OUTPUT)            #设定编码电机引脚2为输出模式
        self.pi.set_PWM_frequency(self.gpio_pwm1, 1000)            #定义编码电机引脚1的频率
        self.pi.set_PWM_frequency(self.gpio_pwm2, 1000)            #定义编码电机引脚2的频率
        self.encoder_last_state1A = self.pi.read(self.encoder_pin1A) #获取编码器A相引脚的状态
        self.encoder_last_state1B = self.pi.read(self.encoder_pin2B) #获取编码器B相引脚的状态
        self.cb1A = self.pi.callback(self.encoder_pin1A, pigpio.EITHER_EDGE, self.encoder_callback1A)
        self.cb1B = self.pi.callback(self.encoder_pin2B, pigpio.EITHER_EDGE, self.encoder_callback1B)


        #==============================================================================================
        self.encoder_pin3A = encoder_pin3A                           #定义编码器A相引脚
        self.encoder_pin4B = encoder_pin4B                           #定义编码器B相引脚
        self.gpio_pwm3 = pwm_pin3                                   #定义编码电机的电机引脚1
        self.gpio_pwm4 = pwm_pin4                                   #定义编码电机的电机引脚2
        self.encoder_position2 = 0                                  #定义保存编码器A，B相的数值变量
        self.encoder_last_position2 = 0                             #定义保存编码器A，B相上一次数值的变量
        self.target_speed2 = 0.0                                    #定义编码电机要达到的目标速度的变量
        self.current_speed2 = 0                                     #定义编码电机当前速度的变量
        self.current_speed_pub2 = Float32()

        #设定编码器引脚模式，电机引脚模式，外部中断函数
        # self.pi = pigpio.pi()
        self.pi.set_mode(self.gpio_pwm3, pigpio.OUTPUT)            #设定编码电机引脚1为输出模式
        self.pi.set_mode(self.gpio_pwm4, pigpio.OUTPUT)            #设定编码电机引脚2为输出模式
        self.pi.set_PWM_frequency(self.gpio_pwm3, 1000)            #定义编码电机引脚1的频率
        self.pi.set_PWM_frequency(self.gpio_pwm4, 1000)            #定义编码电机引脚2的频率
        self.encoder_last_state2A = self.pi.read(self.encoder_pin3A) #获取编码器A相引脚的状态
        self.encoder_last_state2B = self.pi.read(self.encoder_pin4B) #获取编码器B相引脚的状态
        self.cb2A = self.pi.callback(self.encoder_pin3A, pigpio.EITHER_EDGE, self.encoder_callback2A)
        self.cb2B = self.pi.callback(self.encoder_pin4B, pigpio.EITHER_EDGE, self.encoder_callback2B)
        #==============================================================================================


        self.wheel_distance = wheel_length
        self.ppr = ppr                                                #设定编码电机线圈数
        self.gear_ratio = gear_ratio                                  #保存减速电机减速比
        self.calculate_pid_rate = 50.0                                  #设定程序运行频率
        self.calculate_pid_period = 1.0 / 50.0                          #计算pid程序周期
        self.wheel_circumference = wheel_diameter * 3.14159 / 100.0            #轮子周长
        self.Kp1 = 1700                                                          #Kp的值
        self.Ki1 = 120                                                           #Ki的值
        self.Kd1 = 20.0011                                                            #Kd的值
        self.bias1 = 0.0                                                    #保存本次误差
        self.last_bias1 = 0.0                                               #保存上次误差
        self.motor_pwm1 = 0.0                                         #保存电机pwm输出结果
        self.prev_error1 = 0.0                                        #保存上上次记录的误差


        self.Kp2 = 1700                                                          #Kp的值
        self.Ki2 = 120                                                           #Ki的值
        self.Kd2 = 20                                                            #Kd的值
        self.bias2 = 0.0                                                    #保存本次误差
        self.last_bias2 = 0.0                                               #保存上次误差
        self.motor_pwm2 = 0.0                                         #保存电机pwm输出结果
        self.prev_error2 = 0.0                                        #保存上上次记录的误差


        self.factor = 1.27273                  #比例因子，实现编码器一圈理论值和实际值达到一致
        self.encoder_theoretical = self.ppr * 2 * self.gear_ratio    #计算编码器一圈理论值
        # self.encoder_actual = self.encoder_theoretical / self.factor #计算编码器一圈实际值
        self.encoder_actual = 1050 #计算编码器一圈实际值
        # self.encoder_actual_left = 1050  #实际测试得到的左轮编码器一圈数值  
        # self.encoder_actual_right = 700  #实际测试得到的右轮编码器一圈数值

        self.x_old = 0
        self.y_old = 0
        self.x_new = 0
        self.y_new = 0
        self.theta_old = 0
        self.x_position = 0.0
        self.y_position = 0.0

        self.L = 0.075
        self.encoder_dismeter = 1100
        # self.wheel_diameter = 0.048

        self.encoder1_prev = 0
        self.encoder2_prev = 0
        self.encoder_ticks_per_rev = 1100
        self.base_width = 0.079
        self.yaw = 0.0
        self.turn_constant = 1.0
        self.theta0 = 0.0
        self.theta_new = 0.0

        self.speed_count = 0

        rospy.init_node('listener', anonymous=True)
        cmd_subscriber = rospy.Subscriber("cmd_vel", Twist, self.vel_callback)

        self.left_wheel_odom = Float32()
        self.left_wheel_speed = Float32()
        self.left_encoder_position = Float32()
        self.left_wheel_pid = Float32MultiArray()
        self.left_wheel_odom_pub = rospy.Publisher('/uni_wheel/left_wheel/get_odom', Float32, queue_size = 1)
        self.left_wheel_speed_pub = rospy.Publisher('/uni_wheel/left_wheel/get_speed', Float32, queue_size = 1)
        self.left_encoder_position_pub = rospy.Publisher('/uni_wheel/left_wheel/get_encoder', Float32, queue_size = 1)
        self.left_wheel_get_pid_pub = rospy.Publisher('/uni_wheel/left_wheel/get_pid', Float32MultiArray, queue_size = 1)
        self.left_wheel_pid_change_sub = rospy.Subscriber('/uni_wheel/left_wheel/set_wheel_pid', Float32MultiArray, self.set_left_pid_callback)


        self.right_wheel_odom = Float32()
        self.right_wheel_speed = Float32()
        self.right_encoder_position = Float32()
        self.right_wheel_pid = Float32MultiArray()
        self.right_wheel_odom_pub = rospy.Publisher('/uni_wheel/right_wheel/get_odom', Float32, queue_size = 1)
        self.right_wheel_speed_pub = rospy.Publisher('/uni_wheel/right_wheel/get_speed', Float32, queue_size = 1)
        self.right_encoder_position_pub = rospy.Publisher('/uni_wheel/right_wheel/get_encoder', Float32, queue_size = 1)
        self.right_wheel_get_pid_pub = rospy.Publisher('/uni_wheel/right_wheel/get_pid', Float32MultiArray, queue_size = 1)
        self.right_wheel_pid_change_sub = rospy.Subscriber('/uni_wheel/right_wheel/set_wheel_pid', Float32MultiArray, self.set_right_pid_callback)


        self.PID_DEBUG = False

        self.time_prev = rospy.Time.now()
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.br = tf.TransformBroadcaster()
        

        if self.PID_DEBUG:
            print("=====================================================")
            print("wheel_circumference:", self.wheel_circumference)
            print("encoder_actual_left:", self.encoder_actual_left)
            print("encoder_actual_right:", self.encoder_actual_right)
            print("=====================================================")
        print('\033[32m[Left motor start]\033[0m')

        self.wheel_diameter = 0.048
        self.wheel_base = 0.075
        self.encoder_counts_per_rev = 1100
        self.x = 0
        self.y = 0
        self.theta = 0
        self.last_delta = 0.0
        self.is_moving = False
        self.encoder_last_1 = 0
        self.encoder_last_2 = 0
        self.turn_left_value = 0.13
        self.turn_right_value = 0.13
        self.teleop_keyboard_speed = Twist()

    def set_left_pid_callback(self, msg):
        if len(msg.data) == 3:
            self.Kp1 = msg.data[0]
            self.Ki1 = msg.data[1]
            self.Kd1 = msg.data[2]
            print(self.Kp1, self.Ki1, self.Kd1)
        else:
            print("error")

    def set_right_pid_callback(self, msg):
        if len(msg.data) == 3:
            self.Kp2 = msg.data[0]
            self.Ki2 = msg.data[1]
            self.Kd2 = msg.data[2]
            print(self.Kp2, self.Ki2, self.Kd2)
        else:
            print("error")

    def vel_callback(self, msg):
        if msg.linear.x == 0 and msg.angular.z == 0:
            self.is_moving = False
        else:
            self.is_moving = True
        self.teleop_keyboard_speed = msg
        self.set_vel(msg.linear.x, msg.angular.z)


    def set_vel(self, vx, w):
        self.car_vx = vx
        self.car_vw = w
        angular_speed_in_radians = w * 2 * 3.14159
        self.left_speed = -vx + (angular_speed_in_radians * self.wheel_distance / 2.0)
        self.right_speed = vx + (angular_speed_in_radians * self.wheel_distance / 2.0)
        self.target_speed1 = self.left_speed
        self.target_speed2 = self.right_speed


    #Encoder A phase external interrupt function.
    #Used to calculate the numerical value triggered by the external interrupt of encoder A phase.
    def encoder_callback1A(self, gpio, level, tick):
        if level == 1 and self.encoder_last_state1A == 1 and self.encoder_last_state1B == 1:
            self.encoder_position1 -= 1
        elif level == 0 and self.encoder_last_state1A == 0 and self.encoder_last_state1B == 1:
            self.encoder_position1 += 1
        self.encoder_last_state1B = level


    #Encoder B phase external interrupt function.
    #Used to calculate the numerical value triggered by the external interrupt of encoder B phase.
    def encoder_callback1B(self, gpio, level, tick):
        if level == 1 and self.encoder_last_state1A == 1 and self.encoder_last_state1B == 1:
            self.encoder_position1 += 1
        elif level == 0 and self.encoder_last_state1A == 0 and self.encoder_last_state1B == 1:
            self.encoder_position1 -= 1
        self.encoder_last_state1B = level


    #Encoder A phase external interrupt function.
    #Used to calculate the numerical value triggered by the external interrupt of encoder A phase.
    def encoder_callback2A(self, gpio, level, tick):
        if level == 1 and self.encoder_last_state2A == 1 and self.encoder_last_state2B == 1:
            self.encoder_position2 -= 1
        elif level == 0 and self.encoder_last_state2A == 0 and self.encoder_last_state2B == 1:
            self.encoder_position2 += 1
        self.encoder_last_state2B = level


    #Encoder B phase external interrupt function.
    #Used to calculate the numerical value triggered by the external interrupt of encoder B phase.
    def encoder_callback2B(self, gpio, level, tick):
        if level == 1 and self.encoder_last_state2A == 1 and self.encoder_last_state2B == 1:
            self.encoder_position2 += 1
        elif level == 0 and self.encoder_last_state2A == 0 and self.encoder_last_state2B == 1:
            self.encoder_position2 -= 1
        self.encoder_last_state2B = level


    #Calculate the current speed of the wheel by reading the values of encoder A and B phases.return speed.(m/s)
    def calculate_speed(self):
        #calculate left wheel current speed
        delta_position1 = -self.encoder_position1 + self.encoder_last_position1
        round1 = delta_position1 / self.encoder_actual
        odoms1 = round1 * self.wheel_circumference
        self.current_speed1 = odoms1 / self.calculate_pid_period
        self.encoder_last_position1 = self.encoder_position1
        #calculate right wheel current speed
        delta_position2 = -self.encoder_position2 + self.encoder_last_position2
        round2 = delta_position2 / self.encoder_actual
        odoms2 = round2 * self.wheel_circumference
        self.current_speed2 = odoms2 / self.calculate_pid_period
        self.encoder_last_position2 = self.encoder_position2

        self.left_wheel_odom = -self.encoder_position1 / self.encoder_actual * self.wheel_circumference
        self.right_wheel_odom = -self.encoder_position2 / self.encoder_actual * self.wheel_circumference
        self.left_wheel_speed = self.current_speed1
        self.right_wheel_speed = self.current_speed2
        self.left_encoder_position = -self.encoder_position1
        self.right_encoder_position = -self.encoder_position2
        self.left_wheel_pid.data = [self.Kp1, self.Ki1, self.Kd1]
        self.right_wheel_pid.data = [self.Kp2, self.Ki2, self.Kd2]
        # print(self.current_speed1, self.current_speed2)
        return self.current_speed1, self.current_speed2



    #PWM motor speed control function.Range 0~255.
    def set_motor_speed(self, speed1, speed2):
        #set left wheel pwm
        duty_cycle1 = int(abs(speed1))
        if duty_cycle1 >= 255:
            duty_cycle1 = 255
        if speed1 > 0:
            self.pi.set_PWM_dutycycle(self.gpio_pwm1, duty_cycle1)
            self.pi.set_PWM_dutycycle(self.gpio_pwm2, 0)
        elif speed1 == 0:
            self.pi.set_PWM_dutycycle(self.gpio_pwm1, 0)
            self.pi.set_PWM_dutycycle(self.gpio_pwm2, 0)
            self.pi.set_mode(self.gpio_pwm1, pigpio.OUTPUT)
            self.pi.write(self.gpio_pwm1, 0)
            self.pi.set_mode(self.gpio_pwm2, pigpio.OUTPUT)
            self.pi.write(self.gpio_pwm2, 0)
        else:
            self.pi.set_PWM_dutycycle(self.gpio_pwm2, duty_cycle1)
            self.pi.set_PWM_dutycycle(self.gpio_pwm1, 0)
        #set right wheel pwm
        duty_cycle2 = int(abs(speed2))
        if duty_cycle2 >= 255:
            duty_cycle2 = 255
        if speed2 >= 0:
            self.pi.set_PWM_dutycycle(self.gpio_pwm3, duty_cycle2)
            self.pi.set_PWM_dutycycle(self.gpio_pwm4, 0)
        elif speed2 == 0:
            self.pi.set_PWM_dutycycle(self.gpio_pwm3, 0)
            self.pi.set_PWM_dutycycle(self.gpio_pwm4, 0)
            self.pi.set_mode(self.gpio_pwm3, pigpio.OUTPUT)
            self.pi.write(self.gpio_pwm3, 0)
            self.pi.set_mode(self.gpio_pwm4, pigpio.OUTPUT)
            self.pi.write(self.gpio_pwm4, 0)
        else:
            self.pi.set_PWM_dutycycle(self.gpio_pwm4, duty_cycle2)
            self.pi.set_PWM_dutycycle(self.gpio_pwm3, 0)


    #Keep the wheels in a stopped state
    def stop_motor(self):
        self.pi.set_PWM_dutycycle(self.gpio_pwm1, 0)
        self.pi.set_PWM_dutycycle(self.gpio_pwm2, 0)
        self.pi.set_PWM_dutycycle(self.gpio_pwm3, 0)
        self.pi.set_PWM_dutycycle(self.gpio_pwm4, 0)

    #Calculate the motor PWM output value by using PID to read the values of phases A and B from the encoder
    def calculate_pid(self, current_speed1, current_speed2, target_speed1, target_speed2):
        #calculate left wheel output_pwm
        self.bias1 = current_speed1 - target_speed1
        kp_value1 = self.bias1 - self.last_bias1
        kd_value1 = self.bias1 - 2 * self.last_bias1 + self.prev_error1
        self.motor_pwm1 -= self.Kp1 * kp_value1 + self.Ki1 * self.bias1 + self.Kd1 * kd_value1
        self.prev_error1 = self.bias1
        self.last_bias1 = self.bias1
        #calculate right wheel output_pwm
        self.bias2 = current_speed2 - target_speed2
        kp_value2 = self.bias2 - self.last_bias2
        kd_value2 = self.bias2 - 2 * self.last_bias2 + self.prev_error2
        self.motor_pwm2 -= self.Kp2 * kp_value2 + self.Ki2 * self.bias2 + self.Kd2 * kd_value2
        self.prev_error2 = self.bias2
        self.last_bias2 = self.bias2
        #return double wheel output_pwm
        return self.motor_pwm1, self.motor_pwm2


    def calculate_update2(self, enc_left, enc_right):
        # print("wheel_left:", enc_left, "wheel_right:", enc_right)
        if self.is_moving:
            # 计算左右轮子转动的距离
            # print("wheel_left:", enc_left, "wheel_right:", enc_right)
            # print(enc_left, enc_right)
            dist_left = (-enc_left + self.encoder_last_1) / 1100 * self.wheel_diameter * math.pi
            dist_right =(-enc_right+ self.encoder_last_2) / 1100 * self.wheel_diameter * math.pi

            dist_turn_left = enc_left / 1100 * self.wheel_diameter * math.pi
            dist_turn_right = enc_right / 1100 * self.wheel_diameter * math.pi
            # 计算小车移动的里程
            dist = (-dist_left + dist_right) / 2
            # print("----------------------------------------------------")
            # print("dist_left:", dist_left, "dist_right:", dist_right)
            # print("dist_turn_left:", dist_turn_left, "dist_turn_right:", dist_turn_right)
            # print(dist)

            # 计算小车转动的角度
            if self.teleop_keyboard_speed.angular.z > 0:
                theta = ((-dist_turn_right - dist_turn_left) / self.base_width) / 1.0 #1.09  #1.13
                # theta = ((-dist_turn_right - dist_turn_left) / self.base_width) / 1.0 #1.09  #1.13
                # theta = (-dist_turn_right / (self.base_width - 0.009) - dist_turn_left / self.base_width) / 1.13 #1.09  #1.13
            elif self.teleop_keyboard_speed.angular.z < 0:
                theta = ((-dist_turn_right - dist_turn_left) / self.base_width) / 1.0 #1.18
                # theta = ((-dist_turn_right - dist_turn_left) / self.base_width) / 1.3 #1.18
                # theta = (-dist_turn_right / (self.base_width - 0.009) - dist_turn_left / self.base_width) / 1.13 #1.09  #1.13
            else:
                theta = ((-dist_turn_right - dist_turn_left) / self.base_width) / 1.0
            # 更新小车在ROS机器人坐标系中的位置和角度
            self.x += dist * math.cos(self.theta_new)
            self.y += dist * math.sin(self.theta_new)
            self.theta_new = self.theta0 + theta
            # print("theta:", self.theta_new)

            self.encoder_last_1 = enc_left
            self.encoder_last_2 = enc_right

        # 发布里程计信息
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        quat = Quaternion()
        quat.z = math.sin(self.theta_new / 2)
        quat.w = math.cos(self.theta_new / 2)
        odom_msg.pose.pose.orientation = quat
        self.odom_pub.publish(odom_msg)

        # 发布tf关系
        self.br.sendTransform((self.x, self.y, 0),
                              tf.transformations.quaternion_from_euler(0, 0, self.theta_new),
                              rospy.Time.now(),
                              "base_link",
                              "odom")

        return self.x, self.y, self.theta


    def calculate_odometry(self, left_encoder, right_encoder, last_left_encoder, last_right_encoder, last_pose):
        # 计算编码器变化量
        delta_theta_left = left_encoder - last_left_encoder
        delta_theta_right = right_encoder - last_right_encoder

        # 转换为线性位移
        delta_s_left = 0.048 * delta_theta_left
        delta_s_right = 0.048 * delta_theta_right

        # 计算总位移和旋转角度
        delta_s = (delta_s_left + delta_s_right) / 2.0
        delta_theta = (delta_s_right - delta_s_left) / 0.075

        # 更新小车位姿
        new_x = last_pose.x + delta_s * math.cos(last_pose.theta + delta_theta / 2.0)
        new_y = last_pose.y + delta_s * math.sin(last_pose.theta + delta_theta / 2.0)
        new_theta = last_pose.theta + delta_theta

        return Pose(new_x, new_y, new_theta, 0, 0)


    def run(self):
        global_footprint_publisher = rospy.Publisher('global_costmap/footprint', Polygon, queue_size = 10)
        local_footprint_publisher = rospy.Publisher('local_costmap/footprint', Polygon, queue_size = 10)
        plg = Polygon([Point(-0.023, -0.049, 0), Point(-0.023, 0.049, 0), Point(0.109, 0.049, 0), Point(0.109, -0.049, 0)])
        try:
            rate = rospy.Rate(50)
            self.speed_count = 0
            pose = Pose()
            last_left_encoder = 0.0
            last_right_encoder = 0.0
            while not rospy.is_shutdown():
                global_footprint_publisher.publish(plg)
                local_footprint_publisher.publish(plg)
                current_speed1, current_speed2 = self.calculate_speed()
                output1, output2 = self.calculate_pid(current_speed1, current_speed2, self.target_speed1, self.target_speed2)

                self.set_motor_speed(output1, output2)
                self.speed_count = self.speed_count + 1
                if self.speed_count >=5:
                    self.speed_count = 0
                    self.left_wheel_odom_pub.publish(self.left_wheel_odom)
                    self.right_wheel_odom_pub.publish(self.right_wheel_odom)
                    self.left_wheel_speed_pub.publish(self.left_wheel_speed)
                    self.right_wheel_speed_pub.publish(self.right_wheel_speed)
                    self.left_encoder_position_pub.publish(self.left_encoder_position)
                    self.right_encoder_position_pub.publish(self.right_encoder_position)
                    self.left_wheel_get_pid_pub.publish(self.left_wheel_pid)
                    self.right_wheel_get_pid_pub.publish(self.right_wheel_pid)
                x, y, theta = self.calculate_update2(self.encoder_position1, self.encoder_position2)
                # pos = self.calculate_odometry(self.encoder_position1, self.encoder_position2, last_left_encoder, last_right_encoder, pose)
                # last_left_encoder = self.encoder_position1
                # last_right_encoder = self.encoder_position2
                # print(pose)
                rate.sleep()
            self.stop_motor()
            self.cb1A.cancel()
            self.cb1B.cancel()
            self.cb2A.cancel()
            self.cb2B.cancel()
            self.pi.stop()
            print("left_wheel program exit")

        except KeyboardInterrupt:
            self.stop_motor()
            self.cb1A.cancel()
            self.cb1B.cancel()
            self.cb2A.cancel()
            self.cb2B.cancel()
            self.pi.stop()
            print("wheel_left program exit")


def main():
    # motor = MotorControl(16, 13, 5, 6, 20, 19, 12, 18, 7, 4.6, 0.075, 100)
    motor = MotorControl(16, 13, 5, 6, 20, 19, 12, 18, 7, 4.6, 0.075, 100)
    motor.run()

if __name__ == '__main__':
    main()
