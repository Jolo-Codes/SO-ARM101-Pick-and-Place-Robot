from math import cos

import mujoco
import mujoco.viewer
import numpy as np
import math
import time

#Loads our data from our model
model = mujoco.MjModel.from_xml_path("SO101-Model/scene.xml")
data = mujoco.MjData(model)

def solve_3dof_ik(target_x, target_y, target_z, effector_offset, link1_length, link2_length, shoulder_offset, effector_orientation):
    """
    Returns the joint angles for my arm to travel to given (x, y, z) coordinates

    Parameters
    ----------------
    target_x : float
        The x coordinate the arm is moving to.
    target_y : float
        The y coordinate the arm is moving to.
    target_z : float
        The z coordinate the arm is moving to.
    effector_offset : list[int]
        The vector to the gripper from the wrist.
    link1_length : float
        The length of the arm's first link
    link2_length : float
        The length of the arm's second link
    shoulder_offset: list[int]
        The vector from the arm's origin (middle of the base) to the shoulder
    effector_orientation: float
        The orientation at which an arm grabs an object (phi).

    Returns
    -----------
    Elbow-up angles : tuple[float, float, float, float]
        A solution containing the base, shoulder, elbow, and wrist angles for an elbow-up configuration
    Elbow-down angles : tuple[float, float, float, float]
        A solution containing the base, shoulder, elbow, and wrist angles for an elbow-down configuration
    """

    #Calculates the base rotation and the  horizontal radial distance.
    base_angle = math.atan2(target_y, target_x)
    xy_distance = math.hypot(target_x,target_y)


    #Two helper terms used frequently in my calculations (I am simply making the code more readable here).
    intermediate_term_1 = xy_distance - effector_offset[0] * math.cos(effector_orientation) + effector_offset[1] * math.sin(effector_orientation) - shoulder_offset[0]
    intermediate_term_2 = target_z - effector_offset[0] * math.sin(effector_orientation) - effector_offset[1] * math.cos(effector_orientation) - shoulder_offset[1]

    #Amplitude and phase angles used in trig substitution for solving the inverse Kinematics.
    trig_amplitude = math.sqrt(4 * (intermediate_term_1 ** 2) * (link1_length ** 2) + 4 * (intermediate_term_2 ** 2) * (link1_length ** 2))
    phase_angle = math.atan2(intermediate_term_1, intermediate_term_2)

    #Represents the shoulder angle + alpha.
    intermediate_angle = math.asin((intermediate_term_1 ** 2 + intermediate_term_2 ** 2 + link1_length ** 2 - link2_length ** 2) / trig_amplitude)

    #Two shoulder angles as arcsin has two solutions.
    shoulder_angle_1 = intermediate_angle - phase_angle
    shoulder_angle_2 = math.pi - intermediate_angle - phase_angle

    #Two different elbow angles based on the shoulder angle.
    elbow_angle_1 = calculate_elbow_angle(intermediate_term_2, link1_length, intermediate_term_1, shoulder_angle_1)
    elbow_angle_2 = calculate_elbow_angle(intermediate_term_2, link1_length, intermediate_term_1, shoulder_angle_2)

    #Two different wrist angles based on the elbow angle.
    wrist_angle_1 = effector_orientation - elbow_angle_1 - shoulder_angle_1
    wrist_angle_2 = effector_orientation - elbow_angle_2 - shoulder_angle_2

    return (base_angle, shoulder_angle_1, elbow_angle_1, wrist_angle_1), (base_angle, shoulder_angle_2, elbow_angle_2, wrist_angle_2) # Two IK solutions: elbow-up and elbow-down


#Helper for the elbow calculation as it is large
def calculate_elbow_angle(intermediate_term_2, link1_length, intermediate_term_1, shoulder_angle):
    return math.atan2(intermediate_term_2-link1_length*math.sin(shoulder_angle),intermediate_term_1-link1_length*math.cos(shoulder_angle)) - shoulder_angle

mujoco.viewer.launch()









