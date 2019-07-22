#!/usr/bin/env python
#
#   Simple node for slippage detection
#   Subscribed: /odom, /odometry/filtered
#   Publishing: /is_slippage <std_msgs::Bool>
#

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


def compare(odom_1, odom_2):
    # todo: need tuning this params
    dx = 0.1
    dy = 0.1
    dw = 0.1
    
    if abs(odom_1.twist.twist.linear.x - odom_2.twist.twist.linear.x) > dx:
        return False
    if abs(odom_1.twist.twist.linear.y - odom_2.twist.twist.linear.y) > dy:
        return False
    if abs(odom_1.twist.twist.angular.z - odom_2.twist.twist.angular.z) > dw:
        return False
    return True

def cb_odom_by_wheel(odom_msg):
    odom_by_wheel = odom_msg

def cb_odom_by_pos(odom_msg):
    odom_by_pos = odom_msg

if __name__ == '__main__':
    rospy.init_node('tester', anonymous=True)
    is_slippage_pubr = rospy.Publisher('/is_slippage', Bool)
    w_odom_subr = rospy.Subscriber("/odom", Odometry, cb_odom_by_wheel)
    p_odom_subr = rospy.Subscriber("/odom/filtered", Odometry, cb_odom_by_pos)

    odom_by_pos = Odometry()
    odom_by_wheel = Odometry()

    rate = rospy.Rate(30)
    while rospy.is_shutdown():
        iss_msg = Bool()
        
        if compare(odom_by_pos, odom_by_wheel):
            iss_msg.data = False
        else:
            iss_msg.data = True

        is_slippage_pubr.publish(iss_msg)

        rate.sleep()