#!/usr/bin/env python
#
#   
#

import rospy
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import tf
from math import pi, fabs, cos, sin
from Tkinter import Tk, Label
import threading
#from youbot_launcher.msg import FrictionJointState

# Simple class for vectors math
class Vec(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def rot(self, a):
        return Vec(self.x*cos(a)-self.y*sin(a), self.x*sin(a)+self.y*cos(a))
    def __abs__(self):
        return (self.x*self.x + self.y*self.y) ** 0.5
    def __add__(self, v):
        return Vec(self.x + v.x, self.y + v.y)
    def __sub__(self, v):
        return Vec(self.x - v.x, self.y - v.y)
    def __mul__(self, v):
        return self.x*v.x + self.y*v.y
    def __div__(self, a):
        return Vec(self.x/a, self.y/a)

def cb_odom(odom_msg):
    OdomMsg = odom_msg

def cb_jsf(js_msg):
    JSFMsg = js_msg

def cb_slippage(is_slippage_msg):
    IsSlipMsg = is_slippage_msg    

def detector():
    if IsSlipMsg.data:
        return
    
    # Rc vector
    w = OdomMsg.twist.twist.angular.z
    V = Vec(OdomMsg.twist.twist.linear.x, OdomMsg.twist.twist.linear.y)
    Rc = V.rot(pi/2)/w
    
    global new_friction_states
    new_friction_states = list()

    #Names iterator of wheel tf frames
    names_iter = iter(JSFMsg.name)
    for x in JSFMsg.effort:

        # Getting wheel position
        try:
            (trans, rot) = tf_listener.lookupTransform(robot_frame, next(names_iter), rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        # Create radius-vector for wheel position
        rb = Vec(trans[0], trans[1])

        # Radius-vector from wheel to ICC
        Rw = Rc - rb

        # Angle of direction force of the friction
        cos_alfa = (Rw*Rc)/(fabs(Rw) * fabs(Rc))
        
        # Force of normal reaction
        N = gravity*robot_mass/4 #need dynamic model... maybe

        # Goal!
        mu = x/(cos_alfa*wheel_radius*N)
        
        mu_ros_data = Float64()
        mu_ros_data.data = mu
        new_friction_states.append(mu_ros_data)
    
    friction_states = new_friction_states
    #Print Mu value
    mu_label.config(text='%.2f,%.2f,%.2f,%.2f' % tuple([x.data for x in friction_states]))
    mu_label.pack()

if __name__ == '__main__':
    rospy.init_node('mu_detector_node', anonymous=True)
    js_subr = rospy.Subscriber("/joint_states", JointState, cb_jsf)
    odom_subr = rospy.Subscriber("/odometry/filtered", Odometry, cb_odom)
    slippage_subr = rospy.Subscriber("/is_slippage", Bool, cb_slippage)
    #friction_js_pub = rospy.Publisher("/friction_joint_states", FrictionJointState, queue_size=10)

    robot_frame = rospy.get_param("~robot_frame", default="base_footprint")
    wheel_radius = rospy.get_param("~wheel_radius", default=0.0475)
    gravity = rospy.get_param("~gravity", default=9.81)
    robot_mass = rospy.get_param("~robot_mass", default=20+5.3+5)

    OdomMsg = Odometry()
    JSFMsg = JointState()
    IsSlipMsg = Bool()

    tf_listener = tf.TransformListener()

    # create window
    root = Tk()
    root.title("Mu value detected:")
    root.geometry("500x500+100+100")
    mu_label = Label(root, text="unknown")
    mu_label.config(font=('times', 48, 'bold'))
    mu_label.pack()
    # run window process
    root.update()


    # Run detection process
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.spin()
        root.update()

        if OdomMsg != Odometry() and JSFMsg != JointState():
            detector()
            friction_js_pub.publish(friction_states)

        rate.sleep()

    #root.mainloop()
    root.destroy()
