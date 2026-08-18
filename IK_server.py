#!/usr/bin/env python
"""
IK_server.py

ROS service node for the Udacity RoboND "Pick and Place" project.
Receives a list of end-effector poses from the trajectory planner and
returns the corresponding joint-angle trajectory for the KUKA KR210.

All the actual math lives in kr210_kinematics.py (plain numpy, no ROS
dependency) so it can be unit-tested on its own -- see test_kinematics.py.
This file is intentionally a thin ROS wrapper around that module.
"""

import rospy
import tf
from kuka_arm.srv import CalculateIK, CalculateIKResponse
from trajectory_msgs.msg import JointTrajectoryPoint

from kr210_kinematics import inverse_kinematics


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))

    if len(req.poses) < 1:
        rospy.logwarn("No valid poses received")
        return CalculateIKResponse([])

    joint_trajectory_list = []

    for x in range(len(req.poses)):
        pose = req.poses[x]

        px = pose.position.x
        py = pose.position.y
        pz = pose.position.z

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )

        try:
            theta1, theta2, theta3, theta4, theta5, theta6 = inverse_kinematics(
                px, py, pz, roll, pitch, yaw
            )
        except Exception as exc:
            rospy.logerr("IK failed for pose %d: %s" % (x, str(exc)))
            continue

        point = JointTrajectoryPoint()
        point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
        joint_trajectory_list.append(point)

    rospy.loginfo(
        "Returning %s joint trajectory points" % len(joint_trajectory_list)
    )
    return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    rospy.init_node("IK_server")
    s = rospy.Service("calculate_ik", CalculateIK, handle_calculate_IK)
    rospy.loginfo("Ready to receive an IK request")
    rospy.spin()


if __name__ == "__main__":
    IK_server()
