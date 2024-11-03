#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf.transformations import quaternion_from_euler

def publish_transform():
    rospy.init_node('tf_publisher_map_to_odom')

    # 创建一个TF发布者
    br = tf2_ros.TransformBroadcaster()

    # 设置变换信息
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "odom"
    # 设置位置
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    # 设置旋转，顺时针旋转180度
    # q = quaternion_from_euler(0, 0, 3.14159265359)  # 180度对应的弧度值
    q = quaternion_from_euler(0, 0, 0)  # 180度对应的弧度值
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    # 发布TF变换
    rate = rospy.Rate(20.0)  # 10Hz
    while not rospy.is_shutdown():
        # 更新时间戳
        t.header.stamp = rospy.Time.now()
        br.sendTransform(t)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_transform()
    except rospy.ROSInterruptException:
        pass
