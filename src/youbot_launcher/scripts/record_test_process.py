#!/usr/bin/env python

import rospy
import rosbag
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

def cbJS(j_s):
    bag.write('/joint_states', j_s, j_s.header.stamp)

def cbODOM(odom):
    bag.write('/youbot1/odom', odom, odom.header.stamp)

def cbTwist(twist):
    bag.write('/youbot1/cmd_vel', twist, twist.header.stamp)

def listenJS():
    rospy.Subscriber("/joint_states", JointState, cbJS)
    rospy.Subscriber("/youbot1/odom", Odometry, cbODOM)
    rospy.Subscriber("/youbot1/cmd_vel", Twist, cbTwist)
    #rospy.spin()
    #rospy.spin()

def close():
    vel_msg.linear.x = 0
    vel_pub.publish(vel_msg)


def init_twist_msg():
    vel_msg.linear.x = 0
    #Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0


def move():
    #Receiveing the user's input
    print("Let's move your robot")
    speed = 0.8
    accel = 0.01
    distance = 1

    r = rospy.Rate(10)

    init_twist_msg()
    vel_msg.linear.x = 0

    #Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    #Loop to move the turtle in an specified distance
    while (current_distance < distance and not rospy.is_shutdown()):
        #Publish the velocity
        vel_pub.publish(vel_msg)

        #Takes actual time to velocity calculus
        t1=rospy.Time.now().to_sec()
        vel_msg.linear.x += accel * (t1 - t0)
        if vel_msg.linear.x > speed:
            vel_msg.linear.x = speed
        #Calculates distancePoseStamped
        current_distance = speed*(t1-t0)
        r.sleep()

    #After the loop, stops the robot
    vel_msg.linear.x = 0
    #Force the robot to stop
    vel_pub.publish(vel_msg)

if __name__ == '__main__':
    rospy.init_node('tester', anonymous=True)
    vel_pub = rospy.Publisher('/youbot1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    bag = rosbag.Bag("/home/mrboriska/mars/src/youbot_remote/bags/fullout_with_accel/080_001.bag", 'w')

    js_subr = rospy.Subscriber("/joint_states", JointState, cbJS)

    move()
    close()

    rospy.sleep(1.0)
    js_subr.unregister()
    bag.close()

    print("recording success")
