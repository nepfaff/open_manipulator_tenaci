from dataclasses import dataclass


@dataclass
class Waypoint:
    """
    Specifies a gripper pose and opening.
    """

    x: float
    y: float
    z: float
    theta: float
    gripper: float
