#!/usr/bin/env python

import rospy, tf
from visual_mtt.msg import MultiTrack
from geometry_msgs.msg import Twist
from hector_uav_msgs.msg import Altimeter
from sensor_msgs.msg import Imu
from math import *
import numpy as np

# global variables
pixel_x = 0
pixel_y = 0
altitude = 0
phi = 0
theta = 0

x_resolution = 1280
y_resolution = 720
x_half = 1280/2
y_half = 720/2

# focal length calculated by f = M/(2*tan(v/2)) equation from UAV book p.228 and Horizontal and Vertical FOV.
focal_length = 763
hov = 100.0*pi/180.0
vev = 56.25*pi/180.0

def pixel_callback(data):
    global pixel_x
    global pixel_y
    # need to subtract the half size of image width and height so that the center of image plane is (0,0)
    pixel_x = data.tracks[0].meanx-x_half
    pixel_y = data.tracks[0].meany-y_half
    # print "pixel", pixel_x, pixel_y
    # When RRANSAC doesn't have any tracks pixel location, don't move rotor
    if len(data.tracks)==0:
        pixel_x = 0
        pixel_y = 0

def alt_callback(data):
    global altitude
    altitude = data.altitude
    # print "altitude", altitude

def imu_callback(msg):
    global phi
    global theta

    quaternion = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w)

    # Use ROS tf to convert to Euler angles from quaternion
    euler = tf.transformations.euler_from_quaternion(quaternion)

    # roll, pitch, yaw of the quadrotor in XYZ frame. Gazebo's default frame is XYZ.
    phi = euler[0] #*180/np.pi
    theta = euler[1] #*180/np.pi
    psi = euler[2] #*180/np.pi
    # print phi, theta, psi

def calculate_command():
    global pixel_x
    global pixel_y
    global altitude
    global focal_length
    global phi
    global theta

    # print "pixel_x, pixel_y: ", pixel_x, pixel_y

    # angle between target and -z-axis of the rotor in roll axis
    x_angle = phi-pixel_x*hov/x_resolution
    py = altitude*tan(x_angle)
    y_angle = theta+pixel_y*vev/y_resolution
    px = -altitude*tan(y_angle)

    # print "px, py: ", px, py
    scale_factor = 0.2
    # py is basically vx. py is vy with scale factor
    vx = scale_factor*px
    vy = scale_factor*py
    print "vx, vy: ", vx, vy
    return (vx, vy)

def listener():
    rospy.init_node('relative_tracking', anonymous=True)
    rospy.Subscriber("visual_mtt/rransac_tracks", MultiTrack, pixel_callback)
    rospy.Subscriber("hector/altimeter", Altimeter, alt_callback)
    rospy.Subscriber("hector/raw_imu", Imu, imu_callback)

    pub = rospy.Publisher("hector/cmd_vel", Twist, queue_size=1)
    r = rospy.Rate(30) # 20hz

    while not rospy.is_shutdown():
        (vx, vy) = calculate_command()
        vx = saturate(vx, 2)
        vy = saturate(vy, 2)
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        pub.publish(msg)
        r.sleep()

def saturate(v,limit):
    if v>limit:
        v = limit
    elif v<-limit:
        v = -limit
    return v

if __name__ == '__main__':
    listener()
