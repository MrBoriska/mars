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

    rospy.init_node('youbot_dancer_by_step')

    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    arm_pub = rospy.Publisher('arm_1/arm_controller/position_command', JointPositions, queue_size=1)
    gripper_pub = rospy.Publisher('arm_1/gripper_controller/position_command', JointPositions, queue_size=1)
    
    
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
    

    prepare_commands = [
        { # -- open gripper --
            "gripper":0.05,
            "arm":[50,7,-7,7,7],
            "base": {"x":0.0,"y":0.0,"w":0.0}
        },
        { # -- close gripper --
            "gripper":0.05
        }
    ]

    path = [
        { # -- first position (red color)
            "t":3,
            "arm":[50,7,-7,20,107]
        },{ # -- second position (green color)
            "t":3,
            "arm":[169,60,-140,100,165],
        }
    ]

    rospy.loginfo("Start")

    """
    print("Prepare commands process...")

    for goal in prepare_commands:
        raw_input("Press any button for continue...")
        sendControlMsgsFromGoal(goal)
        rospy.sleep(1)
    """
    print("Main commands process...")
    while not rospy.is_shutdown():
        for goal in path:
            raw_input("Press any button for continue...")
            sendControlMsgsFromGoal(goal)
            rospy.sleep(1)
    
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
