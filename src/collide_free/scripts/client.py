#!/usr/bin/env python3

import sys
import rospy
from collide_free.srv import *
from kinematics import *
import matplotlib.pyplot as plt

def setpoint_client():
    rospy.wait_for_service('setpoint')
    mode = int(input("Are we using pre-set input or manual input?\n 0 for pre-set\n 1 for manual\n"))
    if mode == 1:
        entrance = []
        target = []
        xe = float(input("Input desire x entrance position in Z frame in mm\n"))
        entrance.append(xe)
        ye = float(input("Input desire y entrance position in Z frame in mm\n"))
        entrance.append(ye)
        ze = float(input("Input desire z entrance position in Z frame in mm\n"))
        entrance.append(ze)
        entrance.append(1.0)
        entrance = np.array(entrance)
        xt = float(input("Input desire x entrance position in Z frame in mm\n"))
        target.append(xt)
        yt = float(input("Input desire y entrance position in Z frame in mm\n"))
        target.append(yt)
        zt = float(input("Input desire z entrance position in Z frame in mm\n"))
        target.append(zt)
        target.append(1.0)
        target = np.array(target)
    elif mode == 0:
        entrance = np.matrix([1.03888865e+01,1.91675263e+02,3.11775300e+01,1]).T
        target = np.matrix(entrance - np.matrix([4,5,8,0]).T)
    else:
        print("Wrong Input!")
        sys.exit(1)
    joint_pos, targetFK = IK_Z_tip(entrance,target)
    joint_pos = np.array(joint_pos,dtype='float64')
    if joint_pos[3] < -0.6:
        joint_pos[3] = -0.4 
    elif joint_pos[3] > 0.6:
        joint_pos[3] = 0.4
    print(joint_pos)
    print("Moving the robot to the desired position")
    try:
        set_point_control = rospy.ServiceProxy('setpoint', SetPoint)
        resp1 = set_point_control(joint_pos)
        return resp1.wm, resp1.wh, resp1.t
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "useage rosrun collide_free client <mode> \n Mode 1 for entrance only\n Mode 2 for entrance and target point"

if __name__ == "__main__":
    wm, wh, t = setpoint_client()
    t = np.array(t)
    if t.size > 2:
        plt.plot(t, wm, 'r', t, wh,'b')
        plt.ylabel('Collision factor')
        plt.xlabel('time')
        plt.legend(["MRI collision factor", "Head collision factor"], loc ="upper right")
        plt.show()
    else:
        print("The collison factors are")
        print(wm)
        print(wh)
