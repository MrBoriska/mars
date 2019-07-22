#!/usr/bin/env python
#
#   
#

import rospy
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

from Tkinter import Tk, Label


class Vec(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def __abs__(self):
        return (self.x*self.x + self.y*self.y) ** 0.5
    def __add__(self, v):
        return Vec(self.x + v.x, self.y + v.y)
    def __sub__(self, v):
        return Vec(self.x - v.x, self.y - v.y)
    def __mul__(self, v):
        return self.x*v.x + self.y*v.y

def cb_odom(odom_msg):
    OdomMsg = odom_msg

def cb_jsf(js_msg):
    JSFMsg = js_msg

def cb_slippage(is_slippage_msg):
    IsSlipMsg = is_slippage_msg    

def detector():
    if not IsSlipMsg.data:
        return
    
    # todo: need considering y component of velocity too
    v = OdomMsg.twist.twist.linear.x
    w = OdomMsg.twist.twist.angular.z
    Rc = new Vec(v/w, 0)

    #todo: need getting by urdf model and wheel frames
    rb_l = iter([Vec(0.1,0.1),Vec(0.1,0.1),Vec(0.1,0.1),Vec(0.1,0.1)])

    #Calc mu slip parameter(linear moving)
    r = 0.0475
    m = (20+5.3+5)/4 #need dynamic model... maybe
    
    friction_states = list()
    for x in JSFMsg.effort:
        rb = next(rb_l)
        Rw = Rc - rb
        cos_alfa = Rw.dot(Rc)/(abs(Rw) * abs(Rc))
        
        mu = x/(cos_alfa*r*m*9.81)
        
        mu_ros_data = Float64()
        mu_ros_data.data = mu
        friction_states.append(mu_ros_data)
    
    #Print Mu value
    mu_label.config(text='%.2f,%.2f,%.2f,%.2f' % tuple(friction_states))
    mu_label.pack()


filter.t0 = 0
filter.mu = 0
filter.x_cov = 5.0
filter.u_cov = 0.7
filter.x = 0
filter.Eq = filter.x_cov

if __name__ == '__main__':
    rospy.init_node('mu_detector', anonymous=True)
    js_subr = rospy.Subscriber("/joint_states", JointState, cb_jsf)
    odom_subr = rospy.Subscriber("/odometry/filtered", Odometry, cb_odom)
    slippage_subr = rospy.Subscriber("/is_slip", Bool, cb_slippage)
    #friction_js_pub = rospy.Publisher("/friction_joint_states", FrictionJointState, queue_size=10)

    OdomMsg = Odometry()
    JSFMsg = JointState()
    IsSlipMsg = Bool()

    # create window
    root = Tk()
    root.title("Mu value detected:")
    root.geometry("500x500+100+100")
    mu_label = Label(root, text="unknown")
    mu_label.config(font=('times', 48, 'bold'))
    mu_label.pack()
    # run window process
    root.mainloop()

    # Run detection process
    rate = rospy.Rate(10)
    while rospy.is_shutdown():
        #rospy.spin()

        detector()

        #friction_js_pub.publish(friction_states)

        rate.sleep()


