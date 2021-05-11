#!/usr/bin/env python3
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np
import math as m
import kinematics as kine
import collide_detect as cd
'''
 Joint variables of the robot
   1) AxialHeadTranslation (corresponds to the delta_z_sup)
   2) AxialFeetTranslation (corresponds to the delta_z_inf )
   3) LateralTranslation (ranging from -37.5 to 0 mm)
   the first three degree of freedom of the robot positions the robot's RCM point at a target location
   4) PitchRotation (Ry ranging from -37.2 to 30.6 deg)
   5) YawRotation (Rx ranging from -90deg to 0deg)
   4 and 5 positions the needle or the probe to pass through the burr hole
   6) ProbeRotation (Rz continuous deg)
   7) ProbeInsertion (PI ranging from -40 to 0 mm)
'''

joint_pos = [0,0,0,0,0,0,0]
A,B = kine.FK_Z_tip(joint_pos)
print(A)
print(B)
'''
def talker():
    joint_pos = [0,0,0,0,0,0,0]
    A,B = kine.FK_Z_tip(joint_pos)
    pub = rospy.Publisher('floats', numpy_msg(Floats))
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(A)
        pub.publish(B)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
'''
'''
#import the ambf client handler and connect to the client
from ambf_client import Client
_client = Client()
_client.connect()

names = _client.get_obj_names()
base = _client.get_obj_handle('/ambf/env/neuro_robot/base_link')
joint_pos = base.get_all_joint_pos()
print(joint_pos)
'''