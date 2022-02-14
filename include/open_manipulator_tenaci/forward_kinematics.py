import numpy as np

from open_manipulator_tenaci.kinematics_common import JointAngles, ToolPose
from open_manipulator_tenaci.dh import tf_from_dh


def forward_kinematics(
    joint_angles: JointAngles, print_joint_and_tool_position: bool
) -> ToolPose:
    """
    Compute the forward kinematics for the OpenManipulator-X robot.

    At the moment, this returns float("inf") for 'ToolPose.theta'.
    """

    T01 = tf_from_dh(0, 0, 0, joint_angles.joint1_angle + np.pi / 2.0)
    T12 = tf_from_dh(0.0, 0, 0.077, 0)
    T23 = tf_from_dh(
        np.pi / 2.0, 0.0, 0.0, -joint_angles.joint2_angle + np.pi / 2.0 - 0.1853
    )
    T34 = tf_from_dh(0, 0.13, 0, -joint_angles.joint3_angle - np.pi / 2.0 + 0.1853)
    T45 = tf_from_dh(0, 0.124, 0, -joint_angles.joint4_angle)
    T56 = tf_from_dh(0, 0.126, 0, 0)

    tool_position = (T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ np.array([0, 0, 0, 1]))[:3]

    if print_joint_and_tool_position:
        print("-------------------------------")
        print("Joint 1 pose:", (T01 @ np.array([0, 0, 0, 1]))[:3])
        print("Joint 2 pose:", (T01 @ T12 @ T23 @ np.array([0, 0, 0, 1]))[:3])
        print("Joint 3 pose:", (T01 @ T12 @ T23 @ T34 @ np.array([0, 0, 0, 1]))[:3])
        print(
            "Joint 4 pose:", (T01 @ T12 @ T23 @ T34 @ T45 @ np.array([0, 0, 0, 1]))[:3]
        )
        print("Tool pose:", tool_position[:3])
        print("-------------------------------")

    return ToolPose(tool_position[0], tool_position[1], tool_position[2], float("inf"))
