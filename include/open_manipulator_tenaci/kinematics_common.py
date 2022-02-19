from dataclasses import dataclass
from typing import List, Optional

import numpy as np


@dataclass
class JointAngles:
    """
    Class for grouping OpenManipulator-X joint angles.
    All angles are in radians and in range [-pi, pi].
    """

    joint1_angle: float
    joint2_angle: float
    joint3_angle: float
    joint4_angle: float


@dataclass
class ToolPose:
    """
    (x,y,z) represents the position in the world frame (in m).
    theta represents the tool orientation.
    """

    x: float
    y: float
    z: float
    theta: float


def do_joint_angles_violate_joint_limits(joint_angles: JointAngles) -> bool:
    # Source for joint angle limits:
    # https://github.com/ROBOTIS-GIT/open_manipulator/blob/be2859a0506b4e941a19435c0a07562b41768a27/open_manipulator_libs/src/OpenManipulator.cpp#L34-L73
    
    if joint_angles.joint1_angle > np.pi or joint_angles.joint1_angle < -np.pi:
        return True

    if joint_angles.joint2_angle > 1.67 or joint_angles.joint2_angle < -2.05:
        return True

    if joint_angles.joint3_angle > 1.53 or joint_angles.joint3_angle < -1.67:
        return True

    if joint_angles.joint4_angle > 2.0 or joint_angles.joint4_angle < -1.8:
        return True

    return False


def get_first_valid_joint_angles(
    joint_angles: List[JointAngles]
) -> Optional[JointAngles]:
    for angles in joint_angles:
        if not do_joint_angles_violate_joint_limits(angles):
            return angles

    # No valid joint angles found
    return None
