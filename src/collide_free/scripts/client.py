#!/usr/bin/env python3

import sys
import rospy
from SetPoint.srv import *
from kinematics import *
def setpoint_client(mode):
    rospy.wait_for_service('setpoint')
    if mode == 2:
        entrance = []
        target = []
        entrance.append(float(input("Input desire x entrance position in Z frame\n")))
        entrance.append(float(input("Input desire y entrance position in Z frame\n")))
        entrance.append(float(input("Input desire z entrance position in Z frame\n")))
        entrance.append(1.0)
        target.append(float(input("Input desire x target position in Z frame\n")))
        target.append(float(input("Input desire y target position in Z frame\n")))
        target.append(float(input("Input desire z target position in Z frame\n")))
        target.append(1.0)
        joint_pos, targetFK = IK_Z_tip(entrance,target)
    else:
        entrance = []
        entrance.append(float(input("Input desire x entrance position in Z frame\n")))
        entrance.append(float(input("Input desire y entrance position in Z frame\n")))
        entrance.append(float(input("Input desire z entrance position in Z frame\n")))
        entrance.append(1.0)
        joint_pos = IK_entrance(entrance)
    try:
        set_point_control = rospy.ServiceProxy('setpoint', SetPoint)
        resp1 = set_point_control(joint_pos)
        return resp1.res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "useage rosrun collide_free client <mode> \n Mode 1 for entrance only\n Mode 2 for entrance and target point"

if __name__ == "__main__":
    des_position = []
    if len(sys.argv) == 2:
        mode = int(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)
    if mode != 1 and mode != 2:
        print(usage())
        sys.exit(1)
    print("Moving the robot to the desired position")
    setpoint_client(mode)