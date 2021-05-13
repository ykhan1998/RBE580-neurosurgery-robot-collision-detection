#!/usr/bin/env python3

import rospy
import sys
import time
from collide_free.srv import *
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
    time.sleep(1)
    print("Recieved joint positions")
    print(req.pose)
    joint_pos = np.array(req.pose,dtype = np.float32)
    cfm = MRI_collide_detection(joint_pos)
    cfh = Head_collide_detection(joint_pos)
    wm = [cfm[0]]
    wh = [cfh[0]]
    t = [0]
    if cfm[0] == 1 or cfh[0] == 1:
        wm = cfm[1]
        wh = cfh[1]
        print("Destination is not reachable without collision")
        return {'wm': wm, 'wh': wh, 't': t}
    else:
        print("Destination is reachable without collision")
    # current joint pose
    c_pos = np.array([.00000000,.00000000,.00000000,.0000000,.00000000,.00000000,.0000000])
    # ending joint pose
    e_pos = joint_pos
    e_pos[0] = e_pos[0]/1000
    e_pos[1] = e_pos[1]/1000
    e_pos[2] = e_pos[2]
    e_pos[3] = e_pos[3]
    e_pos[4] = -e_pos[4]/1000
    e_pos[6] = -e_pos[6]/1000

    # step joint pose
    s_pos = e_pos / 100
    print(s_pos)
    print("starting")
    # we will rotate the robot while slowly pull out the needle first
    if joint_pos[2]!=0 or joint_pos[3]!=0:
        dinsert = -40/100000
        i = 0
        while i<=100:
            # first check the next point is safe
            n_pos = c_pos
            n_pos[2] = n_pos[2] + s_pos[2]
            n_pos[3] = n_pos[3] + s_pos[3]
            n_pos[4] = n_pos[4] + dinsert
            n_pos[5] = n_pos[5] + s_pos[5]
            cfmr = MRI_collide_detection(n_pos)
            cfhr = Head_collide_detection(n_pos)
            if cfmr[0]!=1 and cfhr[0] != 1:
                base.set_joint_pos(3,n_pos[2])
                base.set_joint_pos(2,n_pos[3])
                base.set_joint_pos(4,n_pos[4])
                base.set_joint_pos(5,n_pos[5])
                c_pos[2] = base.get_joint_pos(2)
                c_pos[3] = base.get_joint_pos(3)
                c_pos[4] = base.get_joint_pos(4)
                c_pos[5] = base.get_joint_pos(5)
            elif cfmr[0] == 1 and cfhr[0] != 1:
                base.set_joint_pos(2,n_pos[2])
                base.set_joint_pos(3,n_pos[3])
                base.set_joint_pos(4,c_pos[4]-dinsert)
                base.set_joint_pos(5,n_pos[5])
                c_pos[2] = base.get_joint_pos(2)
                c_pos[3] = base.get_joint_pos(3)
                c_pos[4] = base.get_joint_pos(4)
                c_pos[5] = base.get_joint_pos(5)
            else:
                print("Trajectory error")
                sys.exit(1)
            wm.append(cfmr[0])
            wh.append(cfhr[0])
            t.append(t[-1]+0.1)
            i = i+1
            time.sleep(0.1)

    dinsert = (e_pos[4] - c_pos[4])/100
    i=0
    while i<=100:
        # first check the next point is safe
        n_pos = c_pos
        n_pos[0] = n_pos[0] + s_pos[0]
        n_pos[1] = n_pos[1] + s_pos[1]
        n_pos[4] = n_pos[4] + dinsert
        n_pos[6] = n_pos[6] + s_pos[6]
        cfmr = MRI_collide_detection(n_pos)
        cfhr = Head_collide_detection(n_pos)
        if cfmr[0]!=1 or cfhr[0] !=1:
            base.set_joint_pos(0,n_pos[0])
            base.set_joint_pos(1,n_pos[1])
            base.set_joint_pos(4,n_pos[4])
            base.set_joint_pos(6,n_pos[6])
            c_pos[0] = base.get_joint_pos(0)
            c_pos[1] = base.get_joint_pos(1)
            c_pos[4] = base.get_joint_pos(4)
            c_pos[6] = base.get_joint_pos(6)
            wm.append(cfmr[0])
            wh.append(cfhr[0])
            t.append(t[-1]+0.1)
            i = i+1
            time.sleep(0.1)
        else:
            print("Trajectory error")
            sys.exit(1)
    wm = np.array(wm, dtype=np.float32)
    wh = np.array(wh, dtype=np.float32)
    t = np.array(t, dtype=np.float32)
    return {'wm': wm, 'wh': wh, 't': t}

def setpoint_server():
    rospy.init_node('setpoint_server')
    s = rospy.Service('setpoint', SetPoint, set_point_control)
    print("set point control server ready")
    rospy.spin()

if __name__ == "__main__":
    setpoint_server()