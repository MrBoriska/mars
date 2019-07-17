#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

import time

from math import sin, cos, atan2, pow, sqrt, pi, fabs

params = {
    "max_lin_speed": 0.8,
    "max_lin_accel": 0.1 * 1,
    "max_ang_speed": 0.5, # * 1.33
    "max_ang_accel": 0.2 * 1,
    "rate_hz": 50
}

curr_pos = {
    "object_pos": {"x":0,"y":0,"alfa":0},
    "robots_pos": [
        {"x":0,"y":0.5,"alfa":0},
        {"x":0,"y":-0.5,"alfa":0}
    ]
}

def main():
    rospy.init_node('sgur', anonymous=True)

    vel_pub = rospy.Publisher('youbot1/cmd_vel', Twist, queue_size=10)


    pp = raw_input("press the Enter to continue...")


    vel_msg = Twist()
    V=0.5
    R=0.80
    omega = V/R

    vel_msg.linear.x = V
    vel_msg.angular.z = omega

    rospy.loginfo("V=%s W=%s" % (vel_msg.linear.x,vel_msg.angular.z))


    rospy.loginfo("Start")
    vel_pub.publish(vel_msg)
    time.sleep(5)


    vel_msg.angular.z = 0
    vel_msg.linear.x = 0
    vel_pub.publish(vel_msg)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("rospy.ROSInterruptException")
