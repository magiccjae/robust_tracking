#!/usr/bin/env python

import rospy, tf
from sensor_msgs.msg import Imu
import numpy as np

def quaternion_callback(msg):
    quaternion = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w)

    # Use ROS tf to convert to Euler angles from quaternion
    euler = tf.transformations.euler_from_quaternion(quaternion)

    tracker_roll = euler[0]*180/np.pi
    tracker_pitch = euler[1]*180/np.pi
    tracker_yaw = euler[2]*180/np.pi
    print tracker_roll, tracker_pitch, tracker_yaw

def listener():
    rospy.init_node('quaternion_to_euler', anonymous=True)
    rospy.Subscriber('/gimbal_cam/raw_imu', Imu, quaternion_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
