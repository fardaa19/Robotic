"""
kr210_kinematics.py

Forward and Inverse Kinematics for the KUKA KR210 6-DOF arm used in the
Udacity RoboND "Pick and Place" project.

This module is intentionally free of any ROS/rospy imports so the math can
be unit-tested on its own (see test_kinematics.py) before it is wired into
the ROS service in IK_server.py.

DH convention: Modified (Craig) DH parameters, same layout used in the
project's kr210.urdf.xacro:

    i   alpha(i-1)   a(i-1)   d(i)    theta(i)
    1   0            0        0.75    q1
    2   -pi/2        0.35     0       q2 - pi/2
    3   0            1.25     0       q3
    4   -pi/2        -0.054   1.50    q4
    5   pi/2          0       0       q5
    6   -pi/2         0       0       q6
    7(gripper) 0       0       0.303  0
"""

import numpy as np

# ----------------------------------------------------------------------
# DH parameter table (meters, radians)
# ----------------------------------------------------------------------
DH = {
    "alpha0": 0.0,       "a0": 0.0,      "d1": 0.75,
    "alpha1": -np.pi / 2, "a1": 0.35,     "d2": 0.0,
    "alpha2": 0.0,        "a2": 1.25,     "d3": 0.0,
    "alpha3": -np.pi / 2, "a3": -0.054,   "d4": 1.50,
    "alpha4": np.pi / 2,  "a4": 0.0,      "d5": 0.0,
    "alpha5": -np.pi / 2, "a5": 0.0,      "d6": 0.0,
    "alpha6": 0.0,        "a6": 0.0,      "d7": 0.303,
}


def dh_transform(alpha, a, d, theta):
    """Homogeneous transform for one modified-DH link."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct,        -st,       0,       a],
        [st * ca,    ct * ca,  -sa,     -sa * d],
        [st * sa,    ct * sa,   ca,      ca * d],
        [0,          0,         0,       1],
    ])


def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# Correction rotation: DH gripper frame -> URDF gripper frame
# (rotate +180 deg about Z, then -90 deg about Y)
R_CORR = rot_z(np.pi).dot(rot_y(-np.pi / 2))


def joint_transforms(q):
    """
    Return the list of individual link transforms T(i-1,i) for i=1..7
    (7th is the fixed gripper offset), given joint angles q = [q1..q6].
    """
    q1, q2, q3, q4, q5, q6 = q
    thetas = [q1, q2 - np.pi / 2, q3, q4, q5, q6, 0.0]
    Ts = []
    for i in range(1, 8):
        alpha = DH[f"alpha{i-1}"]
        a = DH[f"a{i-1}"]
        d = DH[f"d{i}"]
        theta = thetas[i - 1]
        Ts.append(dh_transform(alpha, a, d, theta))
    return Ts


def forward_kinematics(q):
    """
    Compute end-effector position and orientation (URDF frame convention).

    Returns:
        pos   : (3,) numpy array, EE position (x, y, z)
        R_ee  : (3,3) numpy array, EE rotation matrix (URDF frame)
        T0_ee : (4,4) full homogeneous transform, base -> EE (DH frame,
                pre-correction), useful for debugging.
    """
    Ts = joint_transforms(q)
    T0_g = np.eye(4)
    for T in Ts:
        T0_g = T0_g.dot(T)

    T0_ee = T0_g.copy()
    R_ee = T0_g[:3, :3].dot(R_CORR)
    pos = T0_g[:3, 3]
    return pos, R_ee, T0_ee


def rpy_to_rotation(roll, pitch, yaw):
    """Extrinsic X-Y-Z (roll about X, pitch about Y, yaw about Z), URDF/ROS
    convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    return rot_z(yaw).dot(rot_y(pitch)).dot(rot_x(roll))


def inverse_kinematics(px, py, pz, roll, pitch, yaw):
    """
    Analytic (geometric) IK for the KR210.

    Args:
        px, py, pz    : desired end-effector position (base frame)
        roll,pitch,yaw: desired end-effector orientation (URDF/ROS RPY)

    Returns:
        (theta1, theta2, theta3, theta4, theta5, theta6) in radians
    """
    d1 = DH["d1"]
    a1 = DH["a1"]
    a2 = DH["a2"]
    a3 = DH["a3"]
    d4 = DH["d4"]
    d7 = DH["d7"]

    # ---- Desired EE rotation matrix (URDF frame) ----
    R_ee = rpy_to_rotation(roll, pitch, yaw)

    # ---- Wrist center: back out the gripper-length offset ----
    # d7 is applied as a pure translation along frame-6's z-axis in the DH
    # chain (BEFORE the URDF correction rotation is applied), so we must
    # rotate R_ee back into the DH frame first to get the right direction.
    ee_pos = np.array([px, py, pz])
    R0_6_dh = R_ee.dot(R_CORR.T)
    z_axis_dh = R0_6_dh[:, 2]
    wc = ee_pos - d7 * z_axis_dh

    wx, wy, wz = wc

    # ---- theta1: rotate base to face the wrist center ----
    theta1 = np.arctan2(wy, wx)

    # ---- theta2, theta3: planar 2-link geometry (law of cosines) ----
    r = np.sqrt(wx ** 2 + wy ** 2) - a1
    s = wz - d1

    side_a = np.sqrt(a3 ** 2 + d4 ** 2)   # joint3 -> wrist center
    side_b = a2                            # joint2 -> joint3
    side_c = np.sqrt(r ** 2 + s ** 2)      # joint2 -> wrist center

    # clamp for numerical safety (avoid domain errors from float round-off)
    def _acos_clamped(x):
        return np.arccos(np.clip(x, -1.0, 1.0))

    angle_a = _acos_clamped((side_b ** 2 + side_c ** 2 - side_a ** 2) / (2 * side_b * side_c))
    angle_b = _acos_clamped((side_a ** 2 + side_b ** 2 - side_c ** 2) / (2 * side_a * side_b))

    theta2 = np.pi / 2 - angle_a - np.arctan2(s, r)
    theta3 = np.pi / 2 - (angle_b + np.arctan2(-a3, d4))

    # ---- R0_3 at (theta1, theta2, theta3), then R3_6 = R0_3^T * R_ee_dh ----
    thetas = [theta1, theta2 - np.pi / 2, theta3]
    T0_1 = dh_transform(DH["alpha0"], DH["a0"], DH["d1"], thetas[0])
    T1_2 = dh_transform(DH["alpha1"], DH["a1"], DH["d2"], thetas[1])
    T2_3 = dh_transform(DH["alpha2"], DH["a2"], DH["d3"], thetas[2])
    R0_3 = (T0_1.dot(T1_2).dot(T2_3))[:3, :3]

    # R_ee is in URDF frame; convert back to DH frame before removing R0_3
    R_ee_dh = R_ee.dot(R_CORR.T)
    R3_6 = R0_3.T.dot(R_ee_dh)

    # ---- theta4, theta5, theta6 from R3_6 (standard KR210 decomposition) ----
    theta5 = np.arctan2(np.sqrt(R3_6[0, 2] ** 2 + R3_6[2, 2] ** 2), R3_6[1, 2])

    if np.sin(theta5) < 1e-6:
        # wrist singularity: theta4/theta6 not independently observable,
        # split the required rotation arbitrarily between them.
        theta4 = 0.0
        theta6 = np.arctan2(-R3_6[0, 1], R3_6[0, 0])
    else:
        theta4 = np.arctan2(R3_6[2, 2], -R3_6[0, 2])
        theta6 = np.arctan2(-R3_6[1, 1], R3_6[1, 0])

    return theta1, theta2, theta3, theta4, theta5, theta6
