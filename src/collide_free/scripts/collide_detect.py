#!/usr/bin/env python3

import numpy as np
import math as m
import kinematics as kine


# The basic idea of the collision detection is determine how close 
# is the danger point on robots from the MRI or patient head

# With the possibility output, the path planning can figure out wether it is moving close to 
# bondary or leaving. S that it can better determine the movement velocity or torque

# If the feedback is one, there must be a collision; close to one but larger than 0, in danger
# Smaller than 0, safe

# This is a joint base control method, while for C-space, a grid map should be build

# The future extention of this function can be calculating distance from point cloud bondary to point cloud bondary
# and determining the collision possibility factor

# The MRI_collide_detection based on the circle tangent theory.
# In (X,Y) plain of Z-frame, the MRI bore bondary can be considered as a circle,
# While the danger point movement should inside it.
def MRI_collide_detection(joint_pos):
    
    # radius of MRI bore
    radius_bore = 350
    # TODO determine weight of each danger point colliding possibility to total
    w1 = 0.3
    w2 = 0.1
    w3 = 0.25
    w4 = 0.35
    #define clearance distance
    dis_safe = 25 #mm
    # TODO define danger points
    danger_point1 = np.matrix([0,0,0,1]).T
    danger_point2 = np.matrix([0,0,0,1]).T
    danger_point3 = np.matrix([0,0,0,1]).T
    danger_point4 = np.matrix([0,0,0,1]).T
    # Now we assume they are level in x and have sqrt((35**2) - (20**2)) in y
    ozx = 20
    ozy = m.sqrt((35**2) - (20**2))
    # Get the z to treatment T matrix
    zFrameToTreatment, zFrameToRCM = kine.FK_Z_tip(joint_pos)

    # Convert danger points from RCM frame to Z frame
    Z_point1 = np.matmul(zFrameToRCM,danger_point1)
    Z_point2 = np.matmul(zFrameToRCM,danger_point2)
    Z_point3 = np.matmul(zFrameToRCM,danger_point3)
    Z_point4 = np.matmul(zFrameToRCM,danger_point4)

    # calculate the distance from danger point to MRI bore
    disC_p1 = radius_bore - m.sqrt((Z_point1[0] - ozx) ** 2 + (Z_point1[1] - ozy) ** 2)
    disC_p2 = radius_bore - m.sqrt((Z_point2[0] - ozx) ** 2 + (Z_point2[1] - ozy) ** 2)
    disC_p3 = radius_bore - m.sqrt((Z_point3[0] - ozx) ** 2 + (Z_point3[1] - ozy) ** 2)
    disC_p4 = radius_bore - m.sqrt((Z_point4[0] - ozx) ** 2 + (Z_point4[1] - ozy) ** 2)

    # collision factor calculation
    if disC_p1 <= 0 or disC_p2 <= 0 or disC_p3 <= 0 or disC_p4 <= 0:
        collision_factor = 1.
    else:
        collision_factor = (w1*(dis_safe - disC_p1) + w2*(dis_safe - disC_p2) + w3*(dis_safe - disC_p3) + w4*(dis_safe - disC_p4)) / dis_safe

    return collision_factor


# similar to previous one, checking the distance to head. But this time, the calculation should be in 3D
def Head_collide_detection(joint_pos):

    # radius of the head
    radius_head = 200
    # TODO determine weight of each danger point colliding possibility to total
    w1 = 0.3
    w2 = 0.1
    w3 = 0.25
    w4 = 0.35
    #define clearance distance
    dis_safe = 25 #mm
    # TODO define danger points
    danger_point1 = np.matrix([0,0,0,1]).T
    danger_point2 = np.matrix([0,0,0,1]).T
    danger_point3 = np.matrix([0,0,0,1]).T
    danger_point4 = np.matrix([0,0,0,1]).T
    # define the varance from head center to zframe in 3D
    # Now we assume the are level in x and have sqrt((35**2) - (20**2)) in y
    ozx = 0
    ozy = -100
    ozz = -150

    # Get the z to treatment T matrix
    zFrameToTreatment, zFrameToRCM = kine.FK_Z_tip(joint_pos)

    # Convert danger points from RCM frame to Z frame
    Z_point1 = np.matmul(zFrameToRCM,danger_point1)
    Z_point2 = np.matmul(zFrameToRCM,danger_point2)
    Z_point3 = np.matmul(zFrameToRCM,danger_point3)
    Z_point4 = np.matmul(zFrameToRCM,danger_point4)

    # calculate the distance from danger point to MRI bore
    disC_p1 = radius_head - m.sqrt((Z_point1[0] - ozx) ** 2 + (Z_point1[1] - ozy) ** 2 + (Z_point1[2] - ozz) ** 2)
    disC_p2 = radius_head - m.sqrt((Z_point2[0] - ozx) ** 2 + (Z_point2[1] - ozy) ** 2 + (Z_point2[2] - ozz) ** 2)
    disC_p3 = radius_head - m.sqrt((Z_point3[0] - ozx) ** 2 + (Z_point3[1] - ozy) ** 2 + (Z_point3[2] - ozz) ** 2)
    disC_p4 = radius_head - m.sqrt((Z_point4[0] - ozx) ** 2 + (Z_point4[1] - ozy) ** 2 + (Z_point4[2] - ozz) ** 2)

    # collision factor calculation
    if disC_p1 <= 0 or disC_p2 <= 0 or disC_p3 <= 0 or disC_p4 <= 0:
        collision_factor = 1.
    else:
        collision_factor = (w1*(dis_safe - disC_p1) + w2*(dis_safe - disC_p2) + w3*(dis_safe - disC_p3) + w4*(dis_safe - disC_p4)) / dis_safe

    return collision_factor