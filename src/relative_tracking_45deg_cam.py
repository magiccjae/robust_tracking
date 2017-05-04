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
y_resolution = 720.0
x_half = 1280/2
y_half = 720/2

# focal length calculated by f = M/(2*tan(v/2)) equation from UAV book p.228 and Horizontal and Vertical FOV.
focal_length = 763
hov = 100.0*pi/180.0
vev = 56.25*pi/180.0

def pixel_callback(data):
    global pixel_x
    global pixel_y
    global focal_length

    if len(data.tracks)==0:
        # When RRANSAC doesn't have any tracks pixel location, don't move rotor
        pixel_x = 0
        pixel_y = 0
        focal_length = 0
    else:
        # need to subtract the half size of image width and height so that the center of image plane is (0,0)
        pixel_x = data.tracks[0].meanx-x_half
        pixel_y = data.tracks[0].meany-y_half
        focal_length = 763

    # print "pixel", pixel_x, pixel_y

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

    # line of sight vector in optical frame
    los = np.matrix([[pixel_x],[pixel_y],[focal_length]])
    # print los

    # rotation matrix from optical fram to camera frame
    R_o_to_c = np.matrix([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    # rotation matrix from camera frame to body frame
    R_c_to_b = np.matrix([[cos(pi/4), 0, sin(pi/4)], [0, 1, 0], [-sin(pi/4), 0, cos(pi/4)]])
    # rotation matrix from body frame to vehicle-1 frame
    R_b_to_v1 = rot_b_to_v1(phi,theta)

    # line of sight vector in vehicle-1 frame
    los_v1 = R_b_to_v1*R_c_to_b*R_o_to_c*los
    # print los_v1

    # second element of this vector indicates whether the target is left or right of the rotor
    if los_v1.item(1)>=10:
        wz = 0.001*los_v1.item(1)
    elif los_v1.item(1)<=-10:
        wz = 0.001*los_v1.item(1)
    else:
        wz = 0

    vx = 0
    if focal_length!=0:
        y_angle = theta+pixel_y*vev/y_resolution
        px = -altitude*tan(y_angle)
        scale_factor = 0.2
        # py is basically vx. py is vy with scale factor
        vx = scale_factor*px

    print "vx, wz: ", vx, wz
    return (vx, wz)

def rot_b_to_v1(phi,theta):
    R_v2_to_v1 = np.matrix([[cos(theta), 0, sin(theta)], [0, 1, 0], [-sin(theta), 0, cos(theta)]])
    R_b_to_v2 = np.matrix([[1, 0, 0], [0, cos(phi), -sin(phi)], [0, sin(phi), cos(phi)]])
    return R_v2_to_v1*R_b_to_v2

def listener():
    rospy.init_node('relative_tracking', anonymous=True)
    rospy.Subscriber("visual_mtt/rransac_tracks", MultiTrack, pixel_callback)
    rospy.Subscriber("hector/altimeter", Altimeter, alt_callback)
    rospy.Subscriber("hector/raw_imu", Imu, imu_callback)

    pub = rospy.Publisher("hector/cmd_vel", Twist, queue_size=1)
    r = rospy.Rate(30) # 20hz

    while not rospy.is_shutdown():
        (vx, wz) = calculate_command()
        vx = saturate(vx, 2)
        wz = saturate(wz, pi/8)
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
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
