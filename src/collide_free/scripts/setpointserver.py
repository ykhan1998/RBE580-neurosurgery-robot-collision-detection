#!/usr/bin/env python3

import rospy
from SetPoint.srv import pose
from collide_detect import *

def set_point_control(req):
    joint_pos = req.pos

    return

def setpoint_server():
    rospy.init_node('setpoint_server')
    s = rospy.Service('setpoint', SetPoint, set_point_control)
    print("set point control server ready")
    rospy.spin()

if __name__ == "__main__":
    setpoint_server()