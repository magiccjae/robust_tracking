#!/usr/bin/env python

import rospy
from visual_mtt.msg import MultiTrack
from geometry_msgs.msg import Twist
from hector_uav_msgs.msg import Altimeter
from math import *
import numpy as np

pixel_x = 0
pixel_y = 0
altitude = 0

x_resolution = 1280/2
y_resolution = 720/2

# focal length calculated by f = M/(2*tan(v/2)) equation from UAV book p.228
focal_length = 763

# constant parameter to push the target into the center of image
alpha = 1

# rotation matrix from image frame to quadrotor body frame
Rot_o_to_b = np.array([[0, -1, 0],[-1, 0, 0],[0, 0, -1]])

def pixel_callback(data):
    global pixel_x
    global pixel_y
    # need to subtract the half size of image width and height so that the center of image plane is (0,0)
    pixel_x = data.tracks[0].meanx-x_resolution
    pixel_y = data.tracks[0].meany-y_resolution
    # print "pixel", pixel_x, pixel_y

def alt_callback(data):
    global altitude
    altitude = data.altitude
    # print "altitude", altitude

def calculate_command():
    global pixel_x
    global pixel_Y
    global altitude
    global focal_length
    u = pixel_x
    v = pixel_y
    z = altitude
    fl = focal_length
    z_sq = pow(z,2)
    u_sq = pow(u,2)
    v_sq = pow(v,2)
    fl_sq = pow(fl,2)

    denominator = fl_sq + u_sq*z_sq + v_sq*z_sq

    # pseudo inverse of image jacobian
    L_dagger = np.array([[-z*(fl_sq+u_sq*z_sq)/(fl*denominator), -u*v*z*z_sq/(fl*denominator)],
                         [-u*v*z*z_sq/(fl*denominator), -z*(fl_sq+v_sq*z_sq)/(fl*denominator)],
                         [v*z_sq/denominator, -u*z_sq/denominator]])
    s_dot = np.array([[-alpha*u],[-alpha*v]])

    # V_c contains x, y translational velocity and rotational velocity about z-axis which is yaw rate
    V_c = np.dot(L_dagger,s_dot)
    V_c = np.dot(Rot_o_to_b, V_c)

    # slow down the rotor
    scale_factor = 10.0
    V_c = V_c/scale_factor

    velocity_limit = 1.5
    vx = V_c[0]
    if vx > velocity_limit:
        vx = velocity_limit
    elif vx < -velocity_limit:
        vx = -velocity_limit
    vy = V_c[1]
    if vy > velocity_limit:
        vy = velocity_limit
    elif vy < -velocity_limit:
        vy = -velocity_limit
    yaw_rate = V_c[2]

    print vx, vy, yaw_rate
    return  vx, vy, yaw_rate

def listener():
    rospy.init_node('custom_listener', anonymous=True)
    rospy.Subscriber("rransac_tracks", MultiTrack, pixel_callback)
    rospy.Subscriber("hector/altimeter", Altimeter, alt_callback)
    pub = rospy.Publisher("hector/cmd_vel", Twist, queue_size=1)
    r = rospy.Rate(20) # 20hz

    while not rospy.is_shutdown():
        (vx, vy, yaw_rate) = calculate_command()
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = yaw_rate
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    listener()
