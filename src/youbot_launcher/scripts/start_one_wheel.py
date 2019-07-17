#!/usr/bin/env python

import youbotpy

if __name__ == '__main__':
    #rospy.init_node('simply_detector', anonymous=True)

    base = youbotpy.GetBase()
    base.SetWheelVelocities([0.5, 0.0, 0.0, 0.0])
