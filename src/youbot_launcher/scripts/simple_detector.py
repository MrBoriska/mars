#!/usr/bin/env python
#
#   Simple node for detection lower threshold of mu coefficient.
#   Subscribed: /joint_states
#   Publishing: /joint_states_filtered
#   Show in new window (Tkinter) value of mu coefficient 
#

import rospy
from copy import deepcopy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

from Tkinter import Tk, Label

def cbJS(js):
    (effort, predict, velocity) = filter(js.effort[1],js.velocity[1],js.header.stamp.to_nsec())
    js.effort = (
        js.effort[0],
        effort,
        js.effort[2],
        js.effort[3]
    )
    js.velocity = (
        js.velocity[0],
        velocity,
        js.velocity[2],
        js.velocity[3]
    )

    jsf_pub.publish(js)



def predictLin(x1, t2):
    # linear extrapolation
    u = (t2-predictLin.t1)*(x1 - predictLin.x0)/(predictLin.t1-predictLin.t0)

    predictLin.x0 = x1
    predictLin.t0 = predictLin.t1
    predictLin.t1 = t2

    #print("predicted: %s",u)

    return u

predictLin.x0 = 0
predictLin.t0 = 0
predictLin.t1 = 0.001

def predictByVelocity(eff1, t2):
    pBV = predictByVelocity

    #Calc velocity control signal (todo: need real signal and to wheel transformation)
    if (abs(eff1) < 0.1):
        v1 = 0
        effort = 0
    else:
        a = 1.0
        vmax = pBV.vgoal / 0.0475 # w = v/r
        if (pBV.v0 != vmax):
            v1 = pBV.v0 + a*(t2-pBV.t1) #trapecial accel model
            if (v1 > vmax):
                v1 = vmax
        else:
            v1 = vmax

        # effort by DPT model
        vxx = 549.7787 * pBV.vgoal/pBV.vmax
        vxx = vxx / 26 # reduction
        effort_start = 0.0827 * 26
        effort = (vxx - v1)*effort_start/vxx

    pBV.v0 = v1
    pBV.t1 = t2
    result = effort - pBV.eff0
    pBV.eff0 = effort

    return result

pBV = predictByVelocity
pBV.vgoal = 0.65
pBV.vmax = 0.8
pBV.eff0 = 0
pBV.v0 = 0
pBV.t1 = 0



def filter(z1, z2, stamp):
    # Error function
    filter.Eq = filter.x_cov*(filter.Eq+filter.u_cov)/(filter.Eq+filter.u_cov+filter.x_cov)
    # Kalman k
    K = filter.Eq/filter.x_cov
    # Model function (for predict topology process)
    #U = predictLin(filter.x, stamp)
    U = predictByVelocity(z1, stamp)
    # Optimal predicted calc
    x = K*z1 + (1-K)*(filter.x + U)


    # velocity by real effort, by DPT model
    #if (z1 > 0.2):
    # todo: need improving correct by voltage
    vxx = 549.7787*pBV.vgoal/pBV.vmax
    vxx = vxx / 26

    #vxx = z2

    effort_start = 0.0827 * 26
    v = vxx*(1-(x/effort_start))
    #if v > z2:
    #    v = z2;

    #Calc mu slip parameter(linear moving)
    r = 0.0475
    m = (20+5.3+5)/4 #need dynamic model... maybe
    mu = x/(r*m*9.81)
    filter.mu = max(mu, filter.mu)
    
    # this mu is real, if slip was detected
    if filter.mu != 0:
        #print filter.mu
        mu_label.config(text='%.2f' % round(filter.mu, 2))
        #mu_label.pack()
    else:
        mu_label.config(text="unknown")

    # Reset mu by timeout (todo: need update by different in real pos and odometry position)
    if (stamp - filter.t0) > 10e-9:
        #filter.mu = 0
        filter.t0 = stamp

    #print("filtered: %s",x)
    filter.x = x
    return (x, U, v)


filter.t0 = 0
filter.mu = 0
filter.x_cov = 5.0
filter.u_cov = 0.7
filter.x = 0
filter.Eq = filter.x_cov

if __name__ == '__main__':
    rospy.init_node('tester', anonymous=True)
    jsf_pub = rospy.Publisher('/joint_states_filtered', JointState, queue_size=10)
    js_subr = rospy.Subscriber("/joint_states", JointState, cbJS)

    # create window
    root = Tk()
    root.title("Mu value detected:")
    root.geometry("500x500+100+100")
    mu_label = Label(root, text="unknown")
    mu_label.config(font=('times', 48, 'bold'))
    mu_label.pack()
    # run window process
    root.mainloop()
    rospy.spin()

