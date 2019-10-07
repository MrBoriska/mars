#!/usr/bin/env python

import rospy
import rosbag
from geometry_msgs.msg import Twist
from brics_actuator.msg import JointPositions, JointValue
from math import sin, cos, atan2, pow, sqrt, pi, fabs


def createGripperPositionCommand(newPos, t):
    msg = JointPositions()
    joint = JointValue(
        timeStamp=t,
        value=newPos,
        unit="m",
        joint_uri="gripper_finger_joint_l"
    )
    msg.positions.append(joint)
    joint.joint_uri = "gripper_finger_joint_r"
    msg.positions.append(joint)

    return msg

def createArmPositionCommand(newArmPosition, t):
    msg = JointPositions()
    
    for i, val in enumerate(newArmPosition, 1):
        msg.positions.append(JointValue(
            timeStamp=t,
            value=val*pi/180,
            unit="rad",
            joint_uri="arm_joint_%s" % (i,)
        ))
    
    return msg




def main():

    rosbag_file = None
    #rosbag_file = "dance_from_js.bag"

    rospy.init_node('youbot_dancer')

    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    arm_pub = rospy.Publisher('/arm_1/arm_controller/position_command', JointPositions, queue_size=1)
    gripper_pub = rospy.Publisher('/arm_1/gripper_controller/position_command', JointPositions, queue_size=1)
    

    if rosbag_file is not None:
        bag = rosbag.Bag(rosbag_file, 'r')

    def sendControlMsgsFromGoal(goal):
        if "base" in goal.keys():
            twist = Twist()
            twist.linear.x = goal["base"]["x"]
            twist.linear.y = goal["base"]["y"]
            twist.angular.z = goal["base"]["w"]
            vel_pub.publish(twist)

        if "arm" in goal.keys():
            msg = createArmPositionCommand(goal["arm"], rospy.Time.now())
            arm_pub.publish(msg)

        if "gripper" in goal.keys():
            msg = createGripperPositionCommand(goal["gripper"], rospy.Time.now())
            gripper_pub.publish(msg)

        rospy.sleep(goal["t"])
    

    path = [
        { #   --------------- 1 -----------------
            "t":3,
            "base": {"x":0.0,"y":0.05,"w":0.0},
            "arm":[169,60,-140,100,167],
            "gripper":0.011
        },{
            "t":3,
            #"base": {"x":0.0,"y":0.05,"w":0.0},
            "arm":[-45,7,-7,7,7]
        },{
            "t":3,
            "base": {"x":0.0,"y":-0.05,"w":0.0},
            "arm":[-45,60,-140,145,100]
        },{
            "t":3,
            #"base": {"x":-0.05,"y":-0.05,"w":0.0},
            "arm":[-45,7,-7,7,7]
        },{
            "t":3,
            "base": {"x":0.00,"y":0.00,"w":0.0},
            "arm":[-45,60,-140,145,167]
        },{ # --------------- 2 -----------------
            "t":6,
            "base": {"x":0.05,"y":0.05,"w":0.5},
            "arm":[169,60,-140,100,100],
            "gripper":0.011
        },{ # --------------- 3 -----------------
            "t":4,
            "base": {"x":0.0,"y":0.1,"w":0.3},
            "arm":[169,60,-100,100,167],
            "gripper":0.011
        },{
            "t":3,
            "arm":[7,60,-100,100,167],
            "gripper":0.0
        },{
            "t":3,
            "arm":[169,60,-100,100,167],
            "gripper":0.011
        },{
            "t":3,
            "arm":[7,60,-100,100,167],
            "gripper":0.0
        },{
            "t":5,
            "arm":[169,60,-140,100,167],
            "gripper":0.011
        },{ # --------------- 4 -----------------
            "t":3,
            "base": {"x":-0.1,"y":0,"w":0},
            "arm":[7,7,-7,7,7],
            "gripper":0.0
        },{
            "t":10,
            "base": {"x":0.0,"y":0.0,"w":0.0},
            "arm":[7,7,-7,7,7],
            "gripper":0.0
        }
    ]

    while not rospy.is_shutdown():

        try:
            print(path)
        except NameError:
            continue

        rospy.loginfo("Start")

        t_ = None

        if rosbag_file is not None:
            for topic, bag_msg, t in bag.read_messages(topics=['joint_states', 'odom']):
                if t_ is None:
                    t_ = t
                if topic == 'joint_states':
                    msg = JointPositions()
                    for i, joint_uri in enumerate(bag_msg.name):
                        if joint_uri.startswith('arm'):
                            msg.positions.append(JointValue(
                                timeStamp=t,
                                value=bag_msg.position[i],
                                unit="rad",
                                joint_uri=joint_uri
                            ))
                            arm_pub.publish(msg)

                        if joint_uri.startswith('gripper'):
                            msg.positions.append(JointValue(
                                timeStamp=t,
                                value=bag_msg.position[i],
                                unit="m",
                                joint_uri=joint_uri
                            ))
                            gripper_pub.publish(msg)
                    
                            
                
                if topic == 'odom':
                    twist = Twist()
                    twist.linear.x = bag_msg.twist.twist.linear.x
                    twist.linear.y = bag_msg.twist.twist.linear.y
                    twist.angular.z = bag_msg.twist.twist.angular.z
                    vel_pub.publish(twist)

                # this fucking feature. Replace in the future! 
                rospy.sleep(t-t_)
                t_ = t

        else:
            for goal in path:
                sendControlMsgsFromGoal(goal)

        rospy.loginfo("Finish")
    
    sendControlMsgsFromGoal({
        "t":1,
        "base": {"x":0.0,"y":0.0,"w":0.0},
        "arm":[7,7,-7,7,7],
        "gripper":0.0
    })


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("rospy.ROSInterruptException")
