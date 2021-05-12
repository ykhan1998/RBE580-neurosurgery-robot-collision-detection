#!/usr/bin/env python3
import numpy as np
import math as m

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
   7) ProbeInsertion (PI ranging from -40 to 0 mm)
'''


def FK_Z_tip(joint_pos):

    # initial RCM, Z frame variance
    lengthOfAxialTrapezoidSideLink = 60
    widthTrapezoidTop = 30
    initialAxialSeperation = 143
    xInitialRCM = 21.08
    yInitialRCM = 184.7
    zInitialRCM = 69
    robotToRCMOffset = 72.5
    # cannula to treatment, we define this so the robot can compute the cannula length,// C = 5mm
    cannulaToTreatment = 5
    # treatment to tip, where treatment is the piezoelectric element,// A = 10mmtreatment to tip, where treatment is the piezoelectric element,// A = 10mm
    treatmentToTip = 15
    # robot to entry, this allows us to specify how close to the patient the physical robot can be,// B = 5mm
    robotToEntry = 5
    # the robot to treatment distance,// D = 41mm
    robotToTreatmentAtHome = 41

    # If RCM to Z relative pose changed, change the matrix below
    zFrameToRCM = np.matrix([[-1, 0, 0, 0],[0, 0, -1, 0],[0, -1, 0, 0],[0, 0, 0, 1]])

    AxialHeadTranslation = joint_pos[1]  #zsup
    AxialFeetTranslation = joint_pos[6]  #zinf

    # Z position of RCM is solely defined as the midpoint of the axial trapezoid 
    axialTrapezoidMidpoint = (AxialHeadTranslation - AxialFeetTranslation + initialAxialSeperation) / 2
    zDeltaRCM = (AxialFeetTranslation + AxialHeadTranslation) / 2

    # Y position of RCM is found by pythagorean theorem of the axial trapezoid
    yTrapezoidHypotenuseSquared = lengthOfAxialTrapezoidSideLink ** 2
    yTrapezoidSideSquared = (axialTrapezoidMidpoint - widthTrapezoidTop / 2) ** 2
    yTrapezoidInitialSeparationSquared = ((initialAxialSeperation - widthTrapezoidTop) / 2) ** 2
    yDeltaRCM = m.sqrt(yTrapezoidHypotenuseSquared - yTrapezoidSideSquared) - m.sqrt(yTrapezoidHypotenuseSquared - yTrapezoidInitialSeparationSquared)

    # X position of RCM is solely defined as the amount traveled in lateral translation
    xDeltaRCM = joint_pos[0]

    # Obtain basic Yaw, Pitch, and Roll Rotations
    YawRotation = joint_pos[2]
    PitchRotation = joint_pos[3]
    ProbeRotation = joint_pos[5]
    ProbeInsertion = joint_pos[4]

    # Calculate the XYZ Rotation
    xRotationDueToYawRotationFK = np.matrix([[1, 0, 0],[0, m.cos(YawRotation), -m.sin(YawRotation)],[0, m.sin(YawRotation), m.cos(YawRotation)]])
    yRotationDueToPitchRotationFK = np.matrix([[m.cos(PitchRotation), 0, m.sin(PitchRotation)],[0, 1, 0],[-m.sin(PitchRotation), 0, m.cos(PitchRotation)]])
    zRotationDueToProbeRotationFK = np.matrix([[m.cos(ProbeRotation), -m.sin(ProbeRotation), 0],[m.sin(ProbeRotation), m.cos(ProbeRotation), 0],[0, 0, 1]])

    # Calculate the XYZ Translation
    zFrameToRCMRotation = np.identity(4)
    zFrameToRCMRotation[0:3,0:3] = np.matmul(np.matmul(xRotationDueToYawRotationFK,yRotationDueToPitchRotationFK) , zRotationDueToProbeRotationFK)
    zFrameToRCMPrime = np.matrix([[-1, 0, 0, xInitialRCM + xDeltaRCM],[0, 0, -1, yInitialRCM + yDeltaRCM],[0, -1, 0, zInitialRCM + zDeltaRCM],[0, 0, 0, 1]])
    # Now Calculate zFrame to RCM given the calculated values above
    zFrameToRCM = np.matmul(zFrameToRCMPrime,zFrameToRCMRotation)
    # Create RCM to Treatment Matrix
    RCMToTreatment = np.identity(4)
    RCMToTreatment[2][3] = ProbeInsertion + robotToTreatmentAtHome - robotToRCMOffset
    zFrameToTreatment = np.matmul(zFrameToRCM, RCMToTreatment)

    return zFrameToTreatment, zFrameToRCM

'''
This method defines the inverse kinematics for the neurosurgery robot
Given: Vectors for the 3D location of the entry point and target point with respect to the zFrame
Returns: The joint values for the given approach
'''
def IK_Z_tip(entance_posz,target_posz):
    
    #parameter of the robot, change if needed
    lengthOfAxialTrapezoidSideLink = 60
    widthTrapezoidTop = 30
    initialAxialSeperation = 143
    xInitialRCM = 21.08
    yInitialRCM = 184.7
    zInitialRCM = 69
    robotToRCMOffset = 72.5
    cannulaToTreatment = 5
    treatmentToTip = 15
    robotToEntry = 5
    robotToTreatmentAtHome = 41

    # initial RCM, Z frame variance
    zFrameToRCM = np.matrix([[-1, 0, 0, 0],[0, 0, -1, 0],[0, -1, 0, 0],[0, 0, 0, 1]])

    # Get the entry point with respect to the orientation of the zFrame
    rcmToEntry = np.linalg.pinv(zFrameToRCM) * entance_posz

    # Get the target point with respect to the orientation of the zFrame
    rcmToTarget = np.linalg.pinv(zFrameToRCM) * target_posz

    # The yaw and pitch components of the robot rely solely on the entry point's location with respect to the target point
	# This calculation is done with respect to the RCM Orientation
    YawRotation = (3.1415 / 2) + m.atan2((rcmToEntry[2] - rcmToTarget[2]), (rcmToEntry[1] - rcmToTarget[1]))
    PitchRotation = m.atan2((rcmToEntry[0] - rcmToTarget[0]),(rcmToEntry[2] - rcmToTarget[2]))

    # TODO: Add IK for probe Rotation
    ProbeRotation = 0

    # The Translational elements on the Inverse Kinematics rely on the final location of the target point
    XEntry = entance_posz[0]
    YEntry = entance_posz[1]
    ZEntry = entance_posz[2]

    # The Lateral Translation is given by the desired distance in x
    LateralTranslation = XEntry - xInitialRCM - robotToRCMOffset * m.sin(PitchRotation) + robotToEntry * m.sin(PitchRotation)

    # Equations calculated through the symbolic equations for the Forward Kinematics
	# Substituting known values in the FK equations yields the value for Axial Head and Feet
    AxialHeadTranslation = ZEntry - initialAxialSeperation / 2 + widthTrapezoidTop / 2 - zInitialRCM + m.sqrt(8 * YEntry * yInitialRCM - 2 * initialAxialSeperation * widthTrapezoidTop - 4 * YEntry * m.sqrt(-(initialAxialSeperation ** 2) + 2 * initialAxialSeperation * widthTrapezoidTop + 4 * (lengthOfAxialTrapezoidSideLink ** 2) - (widthTrapezoidTop ** 2)) + 4 * yInitialRCM * m.sqrt(-(initialAxialSeperation ** 2) + 2 * initialAxialSeperation * widthTrapezoidTop + 4 * (lengthOfAxialTrapezoidSideLink ** 2) - (widthTrapezoidTop ** 2)) - 4 * (YEntry ** 2) + (initialAxialSeperation ** 2) + (widthTrapezoidTop ** 2) - 4 * (yInitialRCM ** 2) - 4 * (robotToRCMOffset ** 2) * (m.cos(PitchRotation) ** 2) * (m.cos(YawRotation) ** 2) - 4 * (robotToEntry ** 2) * (m.cos(PitchRotation) ** 2) * (m.cos(YawRotation) ** 2) + 8 * robotToRCMOffset * robotToEntry * (m.cos(PitchRotation) ** 2) * (m.cos(YawRotation) ** 2) + 8 * YEntry * robotToRCMOffset * m.cos(PitchRotation) * m.cos(YawRotation) - 8 * YEntry * robotToEntry * m.cos(PitchRotation) * m.cos(YawRotation) - 8 * robotToRCMOffset * yInitialRCM * m.cos(PitchRotation) * m.cos(YawRotation) + 8 * robotToEntry * yInitialRCM * m.cos(PitchRotation) * m.cos(YawRotation) + 4 * robotToRCMOffset * m.cos(PitchRotation) * m.cos(YawRotation) * m.sqrt(-(initialAxialSeperation ** 2) + 2 * initialAxialSeperation * widthTrapezoidTop + 4 * (lengthOfAxialTrapezoidSideLink ** 2) - (widthTrapezoidTop ** 2)) - 4 * robotToEntry * m.cos(PitchRotation) * m.cos(YawRotation) * m.sqrt(-(initialAxialSeperation ** 2) + 2 * initialAxialSeperation * widthTrapezoidTop + 4 * (lengthOfAxialTrapezoidSideLink ** 2) - (widthTrapezoidTop ** 2))) / 2 + robotToRCMOffset * m.cos(PitchRotation) * m.sin(YawRotation) - robotToEntry * m.cos(PitchRotation) * m.sin(YawRotation)
    AxialFeetTranslation = ZEntry + initialAxialSeperation / 2 - widthTrapezoidTop / 2 - zInitialRCM - m.sqrt(8 * YEntry * yInitialRCM - 2 * initialAxialSeperation * widthTrapezoidTop - 4 * YEntry * m.sqrt(-(initialAxialSeperation ** 2) + 2 * initialAxialSeperation * widthTrapezoidTop + 4 * (lengthOfAxialTrapezoidSideLink ** 2) - (widthTrapezoidTop ** 2)) + 4 * yInitialRCM * m.sqrt(-(initialAxialSeperation ** 2) + 2 * initialAxialSeperation * widthTrapezoidTop + 4 * (lengthOfAxialTrapezoidSideLink ** 2) - (widthTrapezoidTop ** 2)) - 4 * (YEntry ** 2) + (initialAxialSeperation ** 2) + (widthTrapezoidTop ** 2) - 4 * (yInitialRCM ** 2) - 4 * (robotToRCMOffset ** 2) * (m.cos(PitchRotation) ** 2) * (m.cos(YawRotation) ** 2) - 4 * (robotToEntry ** 2) * (m.cos(PitchRotation) ** 2) * (m.cos(YawRotation) ** 2) + 8 * robotToRCMOffset * robotToEntry * (m.cos(PitchRotation) ** 2) * (m.cos(YawRotation) ** 2) + 8 * YEntry * robotToRCMOffset * m.cos(PitchRotation) * m.cos(YawRotation) - 8 * YEntry * robotToEntry * m.cos(PitchRotation) * m.cos(YawRotation) - 8 * robotToRCMOffset * yInitialRCM * m.cos(PitchRotation) * m.cos(YawRotation) + 8 * robotToEntry * yInitialRCM * m.cos(PitchRotation) * m.cos(YawRotation) + 4 * robotToRCMOffset * m.cos(PitchRotation) * m.cos(YawRotation) * m.sqrt(-(initialAxialSeperation ** 2) + 2 * initialAxialSeperation * widthTrapezoidTop + 4 * (lengthOfAxialTrapezoidSideLink ** 2) - (widthTrapezoidTop ** 2)) - 4 * robotToEntry * m.cos(PitchRotation) * m.cos(YawRotation) * m.sqrt(-(initialAxialSeperation ** 2) + 2 * initialAxialSeperation * widthTrapezoidTop + 4 * (lengthOfAxialTrapezoidSideLink ** 2) - (widthTrapezoidTop ** 2))) / 2 + robotToRCMOffset * m.cos(PitchRotation) * m.sin(YawRotation) - robotToEntry * m.cos(PitchRotation) * m.sin(YawRotation)
    
    # Probe Insertion is calculate as the distance between the entry point and the target point in 3D space (with considerations for the final treatment zone of the probe)
    ProbeInsertion = m.sqrt((XEntry - target_posz[0]) ** 2 + (YEntry - target_posz[1]) ** 2 + (ZEntry - target_posz[2]) ** 2) - robotToTreatmentAtHome + robotToEntry
    
    # Obtain basic Yaw, Pitch, and Roll Rotations
    xRotationDueToYawRotationFK = np.matrix([[1, 0, 0],[0, m.cos(YawRotation), -m.sin(YawRotation)],[0, m.sin(YawRotation), m.cos(YawRotation)]])
    yRotationDueToPitchRotationFK = np.matrix([[m.cos(PitchRotation), 0, m.sin(PitchRotation)],[0, 1, 0],[-m.sin(PitchRotation), 0, m.cos(PitchRotation)]])
    zRotationDueToProbeRotationFK = np.matrix([[m.cos(ProbeRotation), -m.sin(ProbeRotation), 0],[m.sin(ProbeRotation), m.cos(ProbeRotation), 0],[0, 0, 1]])

    # Calculate the XYZ Rotation
    zFrameToTargetPointFinal = np.identity(4)
    zFrameToTargetPointFinal[0:3,0:3] = np.matmul(np.matmul(xRotationDueToYawRotationFK,yRotationDueToPitchRotationFK) , zRotationDueToProbeRotationFK)
    zFrameToTargetPointFinal = zFrameToRCM * zFrameToTargetPointFinal
    # The X,Y,Z location is already given by the target point
    zFrameToTargetPointFinal[0, 3] = target_posz[0]
    zFrameToTargetPointFinal[1, 3] = target_posz[1]
    zFrameToTargetPointFinal[2, 3] = target_posz[2]

    # Obtain the FK of the target Pose
    target_FK = zFrameToTargetPointFinal

    desired_pos = [LateralTranslation.item(), AxialHeadTranslation.item(), YawRotation, PitchRotation, ProbeInsertion, ProbeRotation, AxialFeetTranslation.item()]
    return desired_pos, target_FK 



'''
def IK_entrance(entance_posz):
    #parameter of the robot, change if needed
    lengthOfAxialTrapezoidSideLink = 60
    widthTrapezoidTop = 30
    initialAxialSeperation = 143
    xInitialRCM = 21.08
    yInitialRCM = 184.7
    zInitialRCM = 69
    robotToRCMOffset = 72.5
    cannulaToTreatment = 5
    treatmentToTip = 15
    robotToEntry = 5
    robotToTreatmentAtHome = 41

    # finding the delta values
    xDeltaRCM = entance_posz[0] - xInitialRCM
    yDeltaRCM = entance_posz[1] - yInitialRCM
    zDeltaRCM = entance_posz[2] - zInitialRCM

    yTrapezoidHypotenuseSquared = lengthOfAxialTrapezoidSideLink ** 2
    yTrapezoidInitialSeparationSquared = ((initialAxialSeperation - widthTrapezoidTop) / 2) ** 2
    axialTrapezoidMidpoint = m.sqrt(-1 * ((m.sqrt(yTrapezoidHypotenuseSquared - yTrapezoidInitialSeparationSquared) ** 2) + 2 * yDeltaRCM * m.sqrt(yTrapezoidHypotenuseSquared - yTrapezoidInitialSeparationSquared) + (yDeltaRCM ** 2) - yTrapezoidHypotenuseSquared)) + widthTrapezoidTop / 2
    
    # Finding the Lateral Translational value which is only dependant on the x offset
    LateralTranslation = xDeltaRCM

    # for the calculation of A * x = B, -->  x = inv(A) * B;
    A = np.matrix([[1, -1],[1, 1]])
    B = np.matrix([0,0]).T

    # Z position of RCM is solely defined as the midpoint of the axial trapezoid
    B[0] = axialTrapezoidMidpoint * 2 - initialAxialSeperation
    B[1] = zDeltaRCM * 2
    x = np.linalg.pinv(A) * B
    AxialHeadTranslation = x[0]
    AxialFeetTranslation = x[1]

    desired_pos = [LateralTranslation.item(), AxialHeadTranslation.item(), 0, 0, 0, 0, AxialFeetTranslation.item()]

    return desired_pos'''