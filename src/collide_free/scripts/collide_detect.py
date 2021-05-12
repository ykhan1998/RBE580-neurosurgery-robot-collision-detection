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
    # determine weight of each danger point colliding possibility to total
    w = [0.25,0.25,0.20,0.18,0.12]
    #define clearance distance
    dis_safe = 20 #mm
    # RCM rotation matrix
    # Obtain basic Yaw, Pitch, and Roll Rotations
    # TODO:check how the joint inputs are tracked (directions)
    YawRotation = joint_pos[2]
    PitchRotation = joint_pos[3]
    ProbeRotation = joint_pos[5]
    ProbeInsertion = joint_pos[4]
    xDeltaRCM = joint_pos[0]
    # Calculate the XYZ Rotation
    xRotationDueToYawRotationFK = np.matrix([[1, 0, 0],[0, m.cos(YawRotation), -m.sin(YawRotation)],[0, m.sin(YawRotation), m.cos(YawRotation)]])
    yRotationDueToPitchRotationFK = np.matrix([[m.cos(PitchRotation), 0, m.sin(PitchRotation)],[0, 1, 0],[-m.sin(PitchRotation), 0, m.cos(PitchRotation)]])
    zRotationDueToProbeRotationFK = np.matrix([[m.cos(ProbeRotation), -m.sin(ProbeRotation), 0],[m.sin(ProbeRotation), m.cos(ProbeRotation), 0],[0, 0, 1]])
    RCMRotation = np.identity(4)
    RCMRotation[0:3,0:3] = np.matmul(np.matmul(xRotationDueToYawRotationFK,yRotationDueToPitchRotationFK) , zRotationDueToProbeRotationFK)
    RCMRotation2D = np.identity(4)
    RCMRotation2D[0:3,0:3] = np.matmul(xRotationDueToYawRotationFK,yRotationDueToPitchRotationFK)
    # point 1: probe top-base
    danger_point1 = np.matmul(RCMRotation,np.matrix([5,181.31+ProbeInsertion,0,1]).T)
    # point 2: probe top-head
    danger_point2 = np.matmul(RCMRotation,np.matrix([69.924,181.31+ProbeInsertion,0,1]).T)
    # point 3: "The back bone"
    danger_point3 = np.matmul(RCMRotation2D,np.matrix([-76.1,183.07,0,1]).T)
    # point 4: "The butt"
    danger_point4 = np.matrix([-254.0-xDeltaRCM,-77.983,0,1]).T
    # point 5: "The elbow"
    danger_point5 = np.matmul(RCMRotation2D,np.matrix([0,83,0,1]).T)
    danger_point5[0] = danger_point5[0] - 188.132


    # Now we assume they are level in x and have sqrt((35**2) - (20**2)) in y
    ozx = 20
    ozy = m.sqrt((35**2) - (20**2))
    # Get the z to treatment T matrix
    zFrameToTreatment, zFrameToRCM = kine.FK_Z_tip(joint_pos)

    # Convert danger points from RCM frame to Z frame
    Z_points = []
    Z_points.append(np.matmul(zFrameToRCM,danger_point1))
    Z_points.append(np.matmul(zFrameToRCM,danger_point2))
    Z_points.append(np.matmul(zFrameToRCM,danger_point3))
    Z_points.append(np.matmul(zFrameToRCM,danger_point4))
    Z_points.append(np.matmul(zFrameToRCM,danger_point5))


    
    # calculate the distance from danger point to MRI bore
    collision_factor = 0
    collison_array = []
    for idx,points in enumerate(Z_points):
        disC= radius_bore - m.sqrt((points[0] - ozx) ** 2 + (points[1] - ozy) ** 2)
        # collision factor calculation
        if disC <= 0:
            collision_factor = 1
            collison_array.append(collision_factor)
            break
        else:
            collison_array.append((dis_safe - disC)/dis_safe)
            collision_factor = collision_factor + w[idx]*(dis_safe - disC) 
    if collision_factor != 1:
        collision_factor = collision_factor/dis_safe

    return collision_factor, np.array(collison_array)


# similar to previous one, checking the distance to head. But this time, the calculation should be in 3D
def Head_collide_detection(joint_pos):

    # radius of the head
    radius_head = 100
    # determine weight of each danger point colliding possibility to total
    w = [0.3,0.1,0.3,0.1,0.22]
    #define clearance distance
    dis_safe = 25 #mm
    # RCM rotation matrix
    # Obtain basic Yaw, Pitch, and Roll Rotations
    # TODO:check how the joint inputs are tracked (directions)
    YawRotation = joint_pos[2]
    PitchRotation = joint_pos[3]
    ProbeRotation = joint_pos[5]
    ProbeInsertion = joint_pos[4]
    # Calculate the XYZ Rotation
    xRotationDueToYawRotationFK = np.matrix([[1, 0, 0],[0, m.cos(YawRotation), -m.sin(YawRotation)],[0, m.sin(YawRotation), m.cos(YawRotation)]])
    yRotationDueToPitchRotationFK = np.matrix([[m.cos(PitchRotation), 0, m.sin(PitchRotation)],[0, 1, 0],[-m.sin(PitchRotation), 0, m.cos(PitchRotation)]])
    zRotationDueToProbeRotationFK = np.matrix([[m.cos(ProbeRotation), -m.sin(ProbeRotation), 0],[m.sin(ProbeRotation), m.cos(ProbeRotation), 0],[0, 0, 1]])
    RCMRotation = np.identity(4)
    RCMRotation[0:3,0:3] = np.matmul(np.matmul(xRotationDueToYawRotationFK,yRotationDueToPitchRotationFK) , zRotationDueToProbeRotationFK)
    RCMRotation2D = np.identity(4)
    RCMRotation2D[0:3,0:3] = np.matmul(xRotationDueToYawRotationFK,yRotationDueToPitchRotationFK)
    # point 1: probe tip
    # point 2: probe top-base
    danger_point2 = np.matmul(RCMRotation,np.matrix([5,181.31+ProbeInsertion,0,1]).T)
    # point 3: probe top-head
    danger_point3 = np.matmul(RCMRotation,np.matrix([69.924,181.31+ProbeInsertion,0,1]).T)
    # point 4: "The back bone"
    danger_point4 = np.matmul(RCMRotation2D,np.matrix([-76.1,183.07,0,1]).T)
    # point 5: "The back bone bottom"
    danger_point5 = np.matmul(RCMRotation2D,np.matrix([-80.6,73.07,0,1]).T)


    # define the varance from head center to zframe in 3D
    # Now we assume the are level in x and have sqrt((35**2) - (20**2)) in y
    ozx = 0
    ozy = -100
    ozz = -150

    # Get the z to treatment T matrix
    zFrameToTreatment, zFrameToRCM = kine.FK_Z_tip(joint_pos)

    # Convert danger points from RCM frame to Z frame
    Z_points = []
    Z_points.append(zFrameToTreatment[:,3])
    Z_points.append(np.matmul(zFrameToRCM,danger_point2))
    Z_points.append(np.matmul(zFrameToRCM,danger_point3))
    Z_points.append(np.matmul(zFrameToRCM,danger_point4))
    Z_points.append(np.matmul(zFrameToRCM,danger_point5))

    # calculate the distance from danger point to patient head
    collision_factor = 0
    collison_array = []
    for idx,point in enumerate(Z_points):
        disC =  m.sqrt((point[0] - ozx) ** 2 + (point[1] - ozy) ** 2 + (point[2] - ozz) ** 2) - radius_head 
        # collision factor calculation
        if disC <= 0:
            collision_factor = 1
            collison_array.append(collision_factor)
            break
        else:
            collison_array.append((dis_safe - disC)/dis_safe)
            collision_factor = collision_factor + w[idx]*(dis_safe - disC)
    if collision_factor != 1:
        collision_factor = collision_factor/dis_safe

    return collision_factor, np.array(collison_array)