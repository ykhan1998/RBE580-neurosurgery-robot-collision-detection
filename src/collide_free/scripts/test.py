#!/usr/bin/env python3
import numpy as np
import kinematics as kine
import collide_detect as cd
'''
 Joint variables of the robot
   1) AxialHeadTranslation -86 to 0(corresponds to the delta_z_sup)
   2) AxialFeetTranslation -143 to 57(corresponds to the delta_z_inf )
   3) AxialFeetTranslation (ranging from -37.5 to 0 mm)
   the first three degree of freedom of the robot positions the robot's RCM point at a target location
   4) PitchRotation (Ry ranging from -37.2 to 30.6 deg)
   5) YawRotation (Rx ranging from -90deg to 0deg)
   4 and 5 positions the needle or the probe to pass through the burr hole
   6) ProbeRotation (Rz continuous deg)
   7) ProbeInsertion (PI ranging from -40 to 0 mm)
'''
mode = int(input("Are we using pre-set input or manual input?\n 0 for pre-set\n 1 for manual\n"))
if mode == 1:
  joint_pos = [.0,.0,.0,.0,.0,.0,.0]
  joint_pos[1] = float(input("Input desired AxialHeadTranslation in mm\n"))
  joint_pos[6] = float(input("Input desired AxialFeetTranslation in mm\n"))
  joint_pos[0] = float(input("Input desired LateralTranslation in mm\n"))
  joint_pos[3] = float(input("Input desired PitchRotation in rads\n"))
  joint_pos[2] = float(input("Input desired YawRotation in rads\n"))
  joint_pos[5] = float(input("Input desired ProbeRotation in rads\n"))
  joint_pos[4] = float(input("Input desired ProbeInsertion in mm\n"))
elif mode == 0:
  joint_pos = [-20,-10,1.0,0.3,0,0,-15]
else:
    print("Wrong Input!")
    sys.exit(1)
A,B = kine.FK_Z_tip(joint_pos)
print("The T-Zframe to treatment frame feed back")
print(A)
print("The T-Zframe to RCM frame feed back")
print(B)
D,E = kine.IK_Z_tip(A[:,3],(A[:,3]-np.matrix([4,5,8,0]).T))
print("The joint settings for treatment with both entrance point and target point known")
print(D)
print("The T-Zframe to treatment frame for treatment with both entrance point and target point known")
print(E)
F = cd.MRI_collide_detection(D)
print("The collision possibility factor with MRI bore")
print(F[0])
print("The collision possibility array with MRI bore")
print(F[1])
G = cd.Head_collide_detection(D)
print("The collision possibility factor with patient head")
print(G[0])
print("The collision possibility array with MRI bore")
print(G[1])