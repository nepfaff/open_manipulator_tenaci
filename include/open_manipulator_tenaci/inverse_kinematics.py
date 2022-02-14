from enum import Enum
from math import atan, atan2, sqrt, cos, sin
from typing import Tuple, List, Optional

import numpy as np

from open_manipulator_tenaci.kinematics_common import JointAngles, ToolPose


class IKSolutionType(Enum):
    """
    Robot can either reach forward or rotate by 180 degrees and reach backwards
    to reach a desired goal pose. Both of these will have both elbow up and elbow
    down solutions.
    """

    ForwardReach = 1
    BackwardReach = 2


def planar_two_r_inverse_kinematics(
    l1: float, l2: float, x_tool: float, y_tool: float
) -> Tuple[Optional[float], Optional[float], Optional[float], Optional[float]]:
    """
    Standard algorithm for planar 2R inverse kinematics.
    See https://motion.cs.illinois.edu/RoboticSystems/InverseKinematics.html
    (section 2).

    :param l1: Link 1 length.
    :param l2: Link 2 length.
    :param x_tool: Goal x-coordinate of the tool.
    :param y_tool: Goal y-coordinate of the tool.
    :return: A tuple of (first joint angle a, second joint angle a,
        first joint angle b, second joint angle b). Where a and b
        represent elbow up and elbow down solutions.
    """

    c2 = (x_tool ** 2 + y_tool ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    if abs(c2) > 1:
        # No possible solution
        return None, None, None, None

    s2_a = sqrt(1 - c2 ** 2)
    s2_b = -s2_a

    t2_a = atan2(s2_a, c2)
    t2_b = atan2(s2_b, c2)

    k1_a = l1 + l2 * cos(t2_a)
    k1_b = l1 + l2 * cos(t2_b)
    k2_a = l2 * sin(t2_a)
    k2_b = l2 * sin(t2_b)

    theta = atan2(y_tool, x_tool)
    t1_a = theta - atan2(k2_a, k1_a)
    t1_b = theta - atan2(k2_b, k1_b)

    return t1_a, t2_a, t1_b, t2_b


def analytical_inverse_kinematics(tool_pose: ToolPose) -> List[JointAngles]:
    """
    Compute IK for OpenManipulator-X robot.
    """

    # OpenManipulator-X constants
    d1 = 0.077
    l1 = 0.130
    l2 = 0.124
    l3 = 0.126

    def ik_solution_from_ik_angles(
        theta1: float, theta2: float, theta3: float, theta4: float
    ) -> JointAngles:
        """
        Transforms internal IK angles into actual OpenManipulator-X joint angles.
        Returns the joint angles as a 'IKSolution'.
        """

        return JointAngles(
            joint1_angle=theta1 - np.pi / 2.0,
            joint2_angle=-theta2 + (np.pi / 2.0 - atan(0.024 / 0.128)),
            joint3_angle=-theta3 - (np.pi / 2.0 - atan(0.024 / 0.128)),
            joint4_angle=-theta4,
        )

    sols: List[JointAngles] = []
    for solType in IKSolutionType:
        # Subsystem 1 (x-y plane)
        theta1 = atan2(tool_pose.y, tool_pose.x)

        # Subsystem 2 (r-z plane where r is merged x-y plane)
        r = sqrt(tool_pose.x ** 2 + tool_pose.y ** 2)
        z = tool_pose.z - d1

        x = r - l3 * cos(tool_pose.theta)
        y = z - l3 * sin(tool_pose.theta)

        if solType == IKSolutionType.BackwardReach:
            # See https://motion.cs.illinois.edu/RoboticSystems/InverseKinematics.html
            # (section 3.1) for reasoning behind reaching backwards solution
            theta1 = theta1 + np.pi
            x = -x

        theta2_a, theta3_a, theta2_b, theta3_b = planar_two_r_inverse_kinematics(
            l1, l2, x, y
        )
        if theta2_a is None:
            continue

        theta4_a = tool_pose.theta - theta2_a - theta3_a
        theta4_b = tool_pose.theta - theta2_b - theta3_b

        if solType == IKSolutionType.BackwardReach and tool_pose.theta == 0.0:
            # NOTE: This seems to be an edge case. Otherwise angles can't
            # all be zero to achieve ThetaTool = 0. Not sure if there are
            # other cases when this occurs.
            # TODO: Look into this!

            if theta4_a <= np.pi:
                theta4_a = theta4_a + np.pi

            if theta4_b <= np.pi:
                theta4_b = theta4_b + np.pi

        # Elbow up and elbow down solution
        sols.append(ik_solution_from_ik_angles(theta1, theta2_a, theta3_a, theta4_a))
        sols.append(ik_solution_from_ik_angles(theta1, theta2_b, theta3_b, theta4_b))

    return sols
