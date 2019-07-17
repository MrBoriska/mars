#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

from math import sin, cos, atan2, pow, sqrt, pi, fabs

params = {
    "max_lin_speed": 0.2,
    "max_lin_accel": 0.2,
    "max_ang_speed": 0.5,
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

    vel_pubs = [
        rospy.Publisher('youbot1/cmd_vel', Twist, queue_size=10),
        rospy.Publisher('youbot2/cmd_vel', Twist, queue_size=10)
    ]

    path = [
        #{"x":0,"y":0},
        #{"x":1,"y":0},
        #{"x":0,"y":0, "alfa": 0}
        {"x":0,"y":0},
        {"x":1,"y":0, "alfa": 180},
        {"x":0,"y":0, "alfa": 0}
    ]

    while not rospy.is_shutdown():

        try:
            print(path)
        except NameError:
            continue

        rospy.loginfo("Start")

        # steps path
        prev_angle = curr_pos["object_pos"]["alfa"]*pi/180;
        for i in range(1):
            for goal in path:
                vector = [curr_pos["object_pos"]["x"] - goal["x"], curr_pos["object_pos"]["y"] - goal["y"]]
                if (vector[0] != 0 or vector[1] != 0):
                    angle = atan2(vector[1],vector[0])
                    distance = sqrt(pow(vector[0], 2) + pow(vector[1], 2))

                    # for minimum rotation
                    angle *= 180/pi
                    if angle > 90:
                        angle = angle - 180
                        distance = -distance
                    elif angle < -90:
                        angle = angle + 180
                        distance = -distance
                    angle *= pi/180

                    #rotate for move to goal
                    motion("r", angle-prev_angle, vel_pubs)
                    prev_angle = angle

                    #move to goal
                    motion("m", distance, vel_pubs)

                # if need orienation in position
                try:
                    angle = goal["alfa"]*pi/180
                    motion("r", angle-prev_angle, vel_pubs)
                    prev_angle = angle
                except KeyError:
                    pass

                # paranoyia mod (for odometry)
                curr_pos["object_pos"].update(goal)

        del path

        rospy.loginfo("Finish")

def update_curr_pos(d_a, d_d):
    for pos in curr_pos["robots_pos"]:
        pos["x"] += d_d*cos(pos["alfa"])
        pos["y"] += d_d*sin(pos["alfa"])
        pos["alfa"] += d_a

    curr_pos["object_pos"]["x"] += d_d*cos(pos["alfa"])
    curr_pos["object_pos"]["y"] += d_d*sin(pos["alfa"])
    curr_pos["object_pos"]["alfa"] += 0 #todo: need change to future


def motion(t, v, vel_pubs):
    if v == 0: return

    speed = {
        "r": params["max_ang_speed"],
        "m": params["max_lin_speed"]
    }[t]

    if v < 0: speed = -speed;

    rospy.loginfo(
        "%s to %s with speed %s" % (
            {"r":"Rotate","m":"Move"}[t],
            {"r":v*180/pi,"m":v}[t],
            speed
        )
    )

    # for slip
    k = [1] * len(vel_pubs)
    k[0:2] = {"r":[1,1.333],"m":[1,1]}[t]

    vel_msgs = []
    for i in range(len(vel_pubs)):
        vel_msgs.append(Twist())
        if t == "r":
            vel_msgs[i].angular.z = speed * k[i]
        elif t == "m":
            vel_msgs[i].linear.x = speed * k[i]

    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_v = 0
    prev_v = 0

    rate = rospy.Rate(50)
    #Loop to move the turtle in an specified distance
    while((v < 0 and current_v > v) or (v > 0 and current_v < v)):
        #Publish the velocity
        for i, vel_pub in enumerate(vel_pubs):
            vel_pub.publish(vel_msgs[i])
        #Takes actual time to velocity calculus
        t1=rospy.Time.now().to_sec()
        #Calculates new value (in time)
        current_v = speed*(t1-t0)

        #Simple odometry:
        if t == "r":
            update_curr_pos(current_v-prev_v, 0)
        elif t == "m":
            update_curr_pos(0, current_v-prev_v)
        prev_v = current_v

        rate.sleep()

    #Force the robot to stop
    for i, vel_pub in enumerate(vel_pubs):
        vel_msgs[i].angular.z = 0
        vel_msgs[i].linear.x = 0
        vel_pub.publish(vel_msgs[i])



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("rospy.ROSInterruptException")
