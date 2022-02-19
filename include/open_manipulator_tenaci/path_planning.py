from dataclasses import dataclass
from re import X
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


def waypoints_for_pick_facing_down_place_facing_straight(
    x: float, y: float
) -> List[Waypoint]:
    """
    Waypoints for picking up a cube with the gripper facing down
    and placing it back down at the same location with the gripper
    facing straight.

    :param x: X coordinate of the cube.
    :param y: Y coordinate of the cube.
    """

    waypoints = []

    # Move to cube location
    waypoints.append(
        Waypoint(x, y, GRIPPER_Z_ABOVE_CUBE, -np.pi / 2.0, GRIPPER_OPENING_OPEN)
    )

    # Grab cube
    waypoints.append(
        Waypoint(x, y, GRIPPER_Z_PICK_UP_CUBE, -np.pi / 2.0, GRIPPER_OPENING_CUBE)
    )

    # Pick up cube
    waypoints.append(
        Waypoint(x, y, GRIPPER_Z_PICK_UP_CUBE, -np.pi / 2.0, GRIPPER_OPENING_CUBE)
    )

    # Bring gripper to save height to prevent touching floor when executing next waypoint
    # NOTE: Not sure if this always works => Test with all possible cube locations
    waypoints.append(Waypoint(x, y, 0.25, np.pi / 3.0, GRIPPER_OPENING_CUBE))

    # Rotate gripper and move back to cube location
    waypoints.append(Waypoint(x, y, GRIPPER_Z_ABOVE_CUBE, 0.0, GRIPPER_OPENING_CUBE))

    # Place down cube
    waypoints.append(Waypoint(x, y, GRIPPER_Z_PICK_UP_CUBE, 0.0, GRIPPER_OPENING_OPEN))

    # Move gripper up
    waypoints.append(Waypoint(x, y, GRIPPER_Z_ABOVE_CUBE, 0.0, GRIPPER_OPENING_OPEN))

    return waypoints


def waypoints_for_pick_facing_straight_place_facing_down(
    x: float, y: float
) -> List[Waypoint]:
    """
    Waypoints for picking up a cube with the gripper facing straight
    and placing it back down at the same location with the gripper
    facing down.

    :param x: X coordinate of the cube.
    :param y: Y coordinate of the cube.
    """
    waypoints = []

    # Move to cube location
    waypoints.append(Waypoint(x, y, GRIPPER_Z_ABOVE_CUBE, 0.0, GRIPPER_OPENING_OPEN))

    # Grab cube
    waypoints.append(Waypoint(x, y, GRIPPER_Z_PICK_UP_CUBE, 0.0, GRIPPER_OPENING_CUBE))

    # Pick up cube
    # Big z-position to prevent tuching the floor during rotation
    waypoints.append(Waypoint(x, y, 0.2, 0.0, GRIPPER_OPENING_CUBE))

    # Rotate gripper and move back to cube location
    waypoints.append(
        Waypoint(x, y, GRIPPER_Z_ABOVE_CUBE, -np.pi / 2.0, GRIPPER_OPENING_CUBE)
    )

    # Place down cube
    waypoints.append(
        Waypoint(x, y, GRIPPER_Z_PICK_UP_CUBE, -np.pi / 2.0, GRIPPER_OPENING_OPEN)
    )

    # Move gripper up
    waypoints.append(
        Waypoint(x, y, GRIPPER_Z_ABOVE_CUBE, -np.pi / 2.0, GRIPPER_OPENING_OPEN)
    )

    return waypoints


def compute_waypoints_for_task_2b(
    cube_locations: List[Tuple[float, float, str]]
) -> List[Waypoint]:
    """
    Computes a sequence of waypoints to achieve task 2b.

    The task involves picking up cubes, rotating them, and
    placing them back down in the same location.
    The cubes start with the red face to the front (away from the robot),
    back (towards the robot), or down (towards the ground).
    They should end with the red face at the top.

    :param cube_locations: A list of (x,y, direction). Direction is
        one of "front", "back", or "down".
    :return: A list of waypoints.
    """
    # TODO: Improve by using task space motion planning or otherwise
    # The current approach does not consider intermediate poses between waypoints
    # which leads to collisions with the floor

    for location in cube_locations:
        x, y, direction = location

        if direction == "front":
            waypoints = waypoints_for_pick_facing_down_place_facing_straight(x, y)
        elif direction == "back":
            waypoints = waypoints_for_pick_facing_straight_place_facing_down(x, y)
        elif direction == "down":
            waypoints = [
                *waypoints_for_pick_facing_down_place_facing_straight(x, y),
                *waypoints_for_pick_facing_down_place_facing_straight(x, y),
            ]
        else:
            assert False, f"Direction is invalid: {direction}"

    return waypoints
