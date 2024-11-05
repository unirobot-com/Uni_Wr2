#!/usr/bin/env python

import tf
import rospy
from math import pi, sin, cos
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3, Polygon

class CarControl():
    def __init__(self):
        rospy.init_node('uni_diffwheel_node', anonymous = False)
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.wheel_distance = rospy.get_param('~wheel_distance', 0.09)
        self.cmd_subscriber = rospy.Subscriber('/cmd_vel', Twist, self.vel_callback)

        self.data0 = 0.0
        self.data1 = 0.0

        self.yaw = 0
        self.car_vx = 0.0
        self.car_vy = 0.0
        self.car_vw = 0.0
        self.right_speed = Float32()
        self.left_speed_pub = rospy.Publisher('/uni_wheel/set_left_speed', Float32, queue_size = 1)
        self.right_speed_pub = rospy.Publisher('/uni_wheel/set_right_speed', Float32, queue_size = 1)
        # self.get_left_seed_sub = rospy.Subscriber('/uni_wheel/get_left_speed', Float32, self.get_leftspeed_callback)
        # self.get_right_seed_sub = rospy.Subscriber('/uni_wheel/get_right_speed', Float32, self.get_rightspeed_callback)

    # def get_leftspeed_callback(self, msg):
    #     self.left_speed = msg.data

    # def get_rightspeed_callback(self, msg):
    #     self.right_speed = msg.data

    def vel_callback(self, msg):
        self.set_vel(msg.linear.x, msg.angular.z)

    def set_vel(self, vx, w):
        self.car_vx = vx
        self.car_vw = w
        angular_speed_in_radians = w * 2 * 3.14159
        self.left_speed = -vx + (angular_speed_in_radians * self.wheel_distance / 2.0)
        self.right_speed = vx + (angular_speed_in_radians * self.wheel_distance / 2.0)
        self.left_speed_pub.publish(self.left_speed)
        self.right_speed_pub.publish(self.right_speed)
        # print("vx:", vx, "w:", w, "left_speed:", self.left_speed.data, "right_speed:", self.right_speed.data)

def main():
    robot = CarControl()
    print('\033[32m[Uni car start]\033[0m')
    tf_listener = tf.TransformListener()
    rate = rospy.Rate(20)

    odom_publisher = rospy.Publisher("odom", Odometry, queue_size = 1)
    global_footprint_publisher = rospy.Publisher('global_costmap/footprint', Polygon, queue_size = 10)
    local_footprint_publisher = rospy.Publisher('local_costmap/footprint', Polygon, queue_size = 10)
    # plg = Polygon([Point(-0.109, -0.051, 0), Point(-0.109, 0.051, 0), Point(0.023, 0.051, 0), Point(0.023, -0.051, 0)])
    plg = Polygon([Point(-0.076, -0.038, 0), Point(-0.076, 0.038, 0), Point(0.001, 0.038, 0), Point(0.001, -0.038, 0)])
    while not rospy.is_shutdown():
        global_footprint_publisher.publish(plg)
        local_footprint_publisher.publish(plg)
        try:
            (trans,rot) = tf_listener.lookupTransform(robot.base_frame, robot.odom_frame, rospy.Time(0))
            robot.yaw = tf.transformations.euler_from_quaternion(rot)[2]
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = robot.odom_frame
            odom.child_frame_id = robot.base_frame
            odom.pose.pose = Pose(Point(*trans), Quaternion(*rot))
            odom.twist.twist = Twist(Vector3(robot.car_vx, 0, 0), Vector3(0, 0, robot.car_vw))
            odom_publisher.publish(odom)
        except Exception as e:
            pass
        rate.sleep()
    robot.set_vel(0, 0)

if __name__ == '__main__':
    main()