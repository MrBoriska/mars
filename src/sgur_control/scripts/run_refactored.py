#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

from math import sin, cos, atan2, pow, sqrt, pi, fabs

params = {
    "max_lin_speed": 0.6,
    "max_lin_accel": 0.9 * 1,
    "max_ang_speed": 1, # * 1.33
    "max_ang_accel": 0.5 * 1,
    "rate_hz": 50
}

curr_pos = {
    "object_pos": {"x":0,"y":0,"alfa":0},
    "robots_pos": [
        {"x":0,"y":0.5,"alfa":0},
        {"x":0,"y":-0.5,"alfa":0}
    ]
}

class Motion(object):

    v_max = 0
    a_max = 0
    dist = 0
    type_ = "m"
    vel_pubs = []
    t = 0
    tk=0

    def __init__(self, type_, d, vel_pubs):
        if d == 0: return

        if type_ == "r":
            self.v_max = params["max_ang_speed"]
            self.a_max = params["max_ang_accel"]
        else:
            self.v_max = params["max_lin_speed"]
            self.a_max = params["max_lin_accel"]
        self.type_ = type_
        self.dist = d
        self.vel_pubs = vel_pubs
        self.tk = self.get_tk()

        if self.dist < 0:
            speed = -self.v_max
        else:
            speed = self.v_max

        rospy.loginfo(
            "%s to %s with speed %s" % (
                {"r":"Rotate","m":"Move"}[type_],
                {"r":d*180/pi,"m":d}[type_],
                speed
            )
        )

        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        self.t = 0
        current_d = 0
        prev_d = 0

        rate = rospy.Rate(params["rate_hz"])
        # Loop to move the robot in an specified distance
        while((d < 0 and current_d > d) or (d > 0 and current_d < d)):
            if rospy.is_shutdown(): return

            #Takes actual time to velocity calculus
            self.t = rospy.Time.now().to_sec() - t0
            vel_msgs = self.get_vel_msgs()

            #Publish the velocity
            for i, vel_pub in enumerate(self.vel_pubs):
                vel_pub.publish(vel_msgs[i])

            #Calculates new dist (in time)
            current_d = self.get_current_val()

            #Simple odometry:
            if type_ == "r":
                update_curr_pos(current_d-prev_d, 0)
            elif type_ == "m":
                update_curr_pos(0, current_d-prev_d)

            prev_d = current_d

            rate.sleep()

        #Force the robot to stop
        if vel_msgs:
            for i, vel_pub in enumerate(self.vel_pubs):
                vel_msgs[i].angular.z = 0
                vel_msgs[i].linear.x = 0
                vel_pub.publish(vel_msgs[i])



    def get_tk(self):
        dk = fabs(self.dist)
        tr = self.v_max/self.a_max
        dr = self.a_max*(tr**2)/2
        if dr > dk/2:
            dr = dk/2
            tr = sqrt(2*dr/self.a_max)

        return tr + (dk-2*dr)/self.v_max + tr

    def get_current_val(self):

        tr = self.v_max/self.a_max
        if tr > self.tk/2: tr = self.tk/2


        dr = (self.a_max*((tr)**2))/2
        if self.t < tr:
            # accel
            current_d = (self.a_max*(self.t**2))/2
        elif self.t >= self.tk:
            # stop
            current_d = self.dist
        elif self.t >= self.tk-tr:
            # decel
            v_max_ = self.a_max*tr
            current_d = dr + v_max_*((self.tk-tr)-tr) + v_max_*(self.t-(self.tk-tr)) - (self.a_max*((self.t-(self.tk-tr))**2))/2
        else:
            # normal
            current_d = dr + self.v_max*(self.t-tr)

        if self.dist < 0 and self.dist != current_d: current_d = -current_d

        rospy.loginfo("curr_d=%s and t=%s tk=%s tr=%s " % (current_d,self.t,self.tk,tr))

        return current_d

    def get_vel_msgs(self):

        tr = self.v_max/self.a_max
        if tr > self.tk/2: tr = self.tk/2

        if self.t < tr:
            # accel
            speed = self.a_max*self.t
        elif self.t >= self.tk:
            # stop
            speed = 0
        elif self.t >= self.tk-tr:
            # decel
            speed = self.a_max*tr - self.a_max*(self.t-(self.tk-tr))
        else:
            # normal
            speed = self.v_max

        if self.dist<0: speed = -speed

        rospy.loginfo("curr_speed %s " % (speed,))

        vel_msgs = []
        for i in range(len(self.vel_pubs)):
            vel_msgs.append(Twist())
            if self.type_ == "r":
                vel_msgs[i].angular.z = speed
            elif self.type_ == "m":
                vel_msgs[i].linear.x = speed

        return vel_msgs


def main():
    rospy.init_node('sgur', anonymous=True)

    vel_pubs = [
        rospy.Publisher('youbot1/cmd_vel', Twist, queue_size=10),
        rospy.Publisher('youbot2/cmd_vel', Twist, queue_size=10)
    ]

    path = [
        {"x":0,"y":0},
        {"x":1.5,"y":0}#,
        #{"x":0,"y":0}
    ]

    pp = raw_input("press the Enter to continue...")

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
                    Motion("r", angle-prev_angle, vel_pubs)
                    prev_angle = angle

                    #move to goal
                    Motion("m", distance, vel_pubs)

                # if need orienation in position
                try:
                    angle = goal["alfa"]*pi/180
                    Motion("r", angle-prev_angle, vel_pubs)
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


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("rospy.ROSInterruptException")
