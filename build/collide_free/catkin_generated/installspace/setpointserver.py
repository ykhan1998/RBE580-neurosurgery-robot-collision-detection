#!/usr/bin/env python3

import rospy
import sys
import time
from SetPoint.srv import *
from collide_detect import *
#import the ambf client handler and connect to the client
from ambf_client import Client
'''
 Joint variables of the robot
   1) AxialHeadTranslation -86 to 0(corresponds to the delta_z_sup)
   2) AxialFeetTranslation -143 to 57(corresponds to the delta_z_inf )
   3) LateralTranslation (ranging from -37.5 to 0 mm)
   the first three degree of freedom of the robot positions the robot's RCM point at a target location
   4) PitchRotation (Ry ranging from -37.2 to 30.6 deg)
   5) YawRotation (Rx ranging from -90deg to 0deg)
   4 and 5 positions the needle or the probe to pass through the burr hole
   6) ProbeRotation (Rz continuous deg)
   7) ProbeInsertion (PI ranging from -40 to 0 mm)'''

def set_point_control(req):
    _client = Client()
    _client.connect()
    base = _client.get_obj_handle('/ambf/env/neuro_robot/base_link')
    base.set_joint_pos(0,0)
    base.set_joint_pos(1,0)
    base.set_joint_pos(2,0)
    base.set_joint_pos(3,0)
    base.set_joint_pos(4,0)
    base.set_joint_pos(5,0)
    base.set_joint_pos(6,0)
    joint_pos = np.array(req.pose,dtype='float64')
    cfm = MRI_collide_detection(joint_pos)
    cfh = Head_collide_detection(joint_pos)
    if cfm[0] == 1 or cfh[0] == 1:
        sys.exit(1)
    wm = np.array([cfm[0]],dtype='float64')
    wh = np.array([cfh[0]],dtype='float64')
    t = np.array([0],dtype='float64')
    # current joint pose
    c_pos = np.array([0,0,0,0,0,0,0])
    # ending joint pose
    e_pos = joint_pos / 1000
    # step joint pose
    s_pos = joint_pos / 100000
    # we will rotate the robot while slowly pull out the needle first
    if not(joint_pos[2]==0 and joint_pos[3]==0 and joint_pos[4]==0 and joint_pos[5]==0):
        dinsert = 40/100000
        while np.mean(abs(e_pos[2:4]-c_pos[2:4])) > 0.0002:
            # first check the next point is safe
            n_pos = c_pos
            n_pos[2] = n_pos[2] + s_pos[2]
            n_pos[3] = n_pos[3] + s_pos[3]
            n_pos[4] = n_pos[4] + dinsert
            n_pos[5] = n_pos[5] + s_pos[5]
            cfmr = MRI_collide_detection(n_pos)
            cfhr = Head_collide_detection(n_pos)
            if not (cfmr[0]==1 or cfhr[0] ==1):
                base.set_joint_pos(2,c_pos[2]+s_pos[2])
                base.set_joint_pos(3,c_pos[3]+s_pos[3])
                base.set_joint_pos(4,c_pos[4]+dinsert)
                base.set_joint_pos(5,c_pos[5]+s_pos[5])
                c_pos[2] = base.get_joint_pos(2)
                c_pos[3] = base.get_joint_pos(3)
                c_pos[4] = base.get_joint_pos(4)
                c_pos[5] = base.get_joint_pos(5)
            elif cfmr[0] == 1 and cfhr[0] != 1:
                base.set_joint_pos(2,c_pos[2]+s_pos[2])
                base.set_joint_pos(3,c_pos[3]+s_pos[3])
                base.set_joint_pos(4,c_pos[4]-dinsert)
                base.set_joint_pos(5,c_pos[5]+s_pos[5])
                c_pos[2] = base.get_joint_pos(2)
                c_pos[3] = base.get_joint_pos(3)
                c_pos[4] = base.get_joint_pos(4)
                c_pos[5] = base.get_joint_pos(5)
            np.append(wm,cfmr[0])
            np.append(wh,cfhr[0])
            np.append(t,t[-1]+0.05)
            time.sleep(0.048)

    dinsert = e_pos[4] - c_pos[4]
    while np.mean(abs(e_pos-c_pos)) > 0.0002:
            # first check the next point is safe
            n_pos = c_pos
            n_pos[0] = n_pos[0] + s_pos[0]
            n_pos[1] = n_pos[1] + s_pos[1]
            n_pos[4] = n_pos[4] + dinsert
            n_pos[6] = n_pos[6] + s_pos[6]
            cfmr = MRI_collide_detection(n_pos)
            cfhr = Head_collide_detection(n_pos)
            if not (cfmr[0]==1 or cfhr[0] ==1):
                base.set_joint_pos(0,c_pos[0]+s_pos[0])
                base.set_joint_pos(1,c_pos[1]+s_pos[1])
                base.set_joint_pos(4,c_pos[4]+dinsert)
                base.set_joint_pos(6,c_pos[6]+s_pos[6])
                c_pos[0] = base.get_joint_pos(0)
                c_pos[1] = base.get_joint_pos(1)
                c_pos[4] = base.get_joint_pos(4)
                c_pos[6] = base.get_joint_pos(6)
                np.append(wm,cfmr[0])
                np.append(wh,cfhr[0])
                np.append(t,t[-1]+0.05)
                time.sleep(0.048)
            else:
                print("Trajectory error")
                sys.exit(1)
    return {'wm': wm, 'wh': wh, 't': t}

def setpoint_server():
    rospy.init_node('setpoint_server')
    s = rospy.Service('setpoint', SetPoint, set_point_control)
    print("set point control server ready")
    rospy.spin()

if __name__ == "__main__":
    setpoint_server()