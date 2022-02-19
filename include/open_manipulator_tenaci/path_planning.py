from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

GRIPPER_OPENING_OPEN = 0.05
GRIPPER_OPENING_CLOSED = -0.05
GRIPPER_OPENING_CUBE = -0.02  # Tight grip around cube

# TODO: Determine more suitable values by experiment
CUBE_SIDE_LENGTH_M = 0.05
GRIPPER_Z_PICK_UP_CUBE = CUBE_SIDE_LENGTH_M * 0.75
GRIPPER_Z_ABOVE_CUBE = CUBE_SIDE_LENGTH_M * 1.5


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


def compute_waypoints_for_task_2a(
    starting_locations: List[Tuple[float, float]],
    finishing_locations: List[Tuple[float, float]],
) -> List[Waypoint]:
    """
    Computes a sequence of waypoints to achieve task 2a.

    The task involves picking up cubes from three starting locations and
    transfering them to three finishing locations.

    :param starting_locations: A list of (x,y) coordinates.
    :param finishing_locations: A list of (x, y) coordinates.
    :return: A list of waypoints.
    """

    waypoints = []
    for start, finish in zip(starting_locations, finishing_locations):
        x_start, y_start = start
        x_finish, y_finish = finish

        # Move to start location
        waypoints.append(
            Waypoint(
                x_start,
                y_start,
                GRIPPER_Z_ABOVE_CUBE,
                -np.pi / 2.0,
                GRIPPER_OPENING_OPEN,
            )
        )

        # Grab cube
        waypoints.append(
            Waypoint(
                x_start,
                y_start,
                GRIPPER_Z_PICK_UP_CUBE,
                -np.pi / 2.0,
                GRIPPER_OPENING_CUBE,
            )
        )

        # Pick up cube
        waypoints.append(
            Waypoint(
                x_start,
                y_start,
                GRIPPER_Z_ABOVE_CUBE,
                -np.pi / 2.0,
                GRIPPER_OPENING_CUBE,
            )
        )

        # Move to finish location
        waypoints.append(
            Waypoint(
                x_finish,
                y_finish,
                GRIPPER_Z_ABOVE_CUBE,
                -np.pi / 2.0,
                GRIPPER_OPENING_CUBE,
            )
        )

        # Place down cube
        waypoints.append(
            Waypoint(
                x_finish,
                y_finish,
                GRIPPER_Z_PICK_UP_CUBE,
                -np.pi / 2.0,
                GRIPPER_OPENING_OPEN,
            )
        )

        # Move gripper up
        waypoints.append(
            Waypoint(
                x_finish,
                y_finish,
                GRIPPER_Z_ABOVE_CUBE,
                -np.pi / 2.0,
                GRIPPER_OPENING_OPEN,
            )
        )

    return waypoints

