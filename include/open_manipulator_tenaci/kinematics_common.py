from dataclasses import dataclass


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
