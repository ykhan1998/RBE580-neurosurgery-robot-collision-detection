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
print("The T-Zframe to treatment frame feed back")
print(A)
print("The T-Zframe to RCM frame feed back")
print(B)
C = kine.IK_entrance(B[:,3])
print("The joint settings for treatment with only entrance point known")
print(C)
D,E = kine.IK_Z_tip(A[:,3],(A[:,3]-np.matrix([3,2,10,0]).T))
print("The joint settings for treatment with both entrance point and target point known")
print(D)
print("The T-Zframe to treatment frame for treatment with both entrance point and target point known")
print(E)
F = cd.MRI_collide_detection(C)
print("The collision possibility factor with MRI bore")
print(F)
G = cd.Head_collide_detection(C)
print("The collision possibility factor with patient head")
print(G)



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