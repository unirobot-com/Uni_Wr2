#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def callback(data):
    rospy.loginfo("Received a scan with %d points.", len(data.ranges))

def listener():
    rospy.init_node('laser_scan_listener', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()