#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf.transformations

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    br = tf2_ros.TransformBroadcaster()
    while not rospy.is_shutdown():
        ts = TransformStamped()
        ts.header.stamp = rospy.Time.now()
        ts.header.frame_id = 'base_link'
        ts.child_frame_id = 'laser'
        ts.transform.translation.x = 0.1
        ts.transform.translation.y = 0.0
        ts.transform.translation.z = 0.2
        qtn = tf.transformations.quaternion_from_euler(0, 0, 0)
        ts.transform.rotation.x = qtn[0]
        ts.transform.rotation.y = qtn[1]
        ts.transform.rotation.z = qtn[2]
        ts.transform.rotation.w = qtn[3]
        br.sendTransform(ts)