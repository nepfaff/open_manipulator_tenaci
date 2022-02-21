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


def waypoints_for_moving_between_start_finish(
    x_start: float,
    y_start: float,
    x_finish: float,
    y_finish: float,
    gripper_z_pick_up_cube: float = GRIPPER_Z_PICK_UP_CUBE,
    gripper_z_above_cube: float = GRIPPER_Z_ABOVE_CUBE,
) -> List[Waypoint]:
    """
    Waypoints for picking up a cube at the start location and
    placing it down at the finish location.

    :param gripper_z_pick_up_cube: Custom z-postion for gripping
        cubes while placing them down. Not used for picking up cubes.
    :param gripper_z_above_cube: Custom z-postion for save gripper
        position above cubes while placing them down. Not used for
        picking up cubes.
    """

    waypoints = []

    # Move to start location
    waypoints.append(
        Waypoint(
            x_start, y_start, GRIPPER_Z_ABOVE_CUBE, -np.pi / 2.0, GRIPPER_OPENING_OPEN
        )
    )

    # Grab cube
    waypoints.append(
        Waypoint(
            x_start, y_start, GRIPPER_Z_PICK_UP_CUBE, -np.pi / 2.0, GRIPPER_OPENING_CUBE
        )
    )

    # Pick up cube
    waypoints.append(
        Waypoint(
            x_start, y_start, GRIPPER_Z_ABOVE_CUBE, -np.pi / 2.0, GRIPPER_OPENING_CUBE
        )
    )

    # Move to finish location
    waypoints.append(
        Waypoint(
            x_finish, y_finish, gripper_z_above_cube, -np.pi / 2.0, GRIPPER_OPENING_CUBE
        )
    )

    # Place down cube
    waypoints.append(
        Waypoint(
            x_finish,
            y_finish,
            gripper_z_pick_up_cube,
            -np.pi / 2.0,
            GRIPPER_OPENING_OPEN,
        )
    )

    # Move gripper up
    waypoints.append(
        Waypoint(
            x_finish, y_finish, gripper_z_above_cube, -np.pi / 2.0, GRIPPER_OPENING_OPEN
        )
    )

    return waypoints


def waypoints_for_pick_facing_down_place_facing_straight(
    x_start: float,
    y_start: float,
    x_finish: float,
    y_finish: float,
    gripper_z_pick_up_cube: float = GRIPPER_Z_PICK_UP_CUBE,
    gripper_z_above_cube: float = GRIPPER_Z_ABOVE_CUBE,
) -> List[Waypoint]:
    """
    Waypoints for picking up a cube with the gripper facing down
    and placing it down with the gripper facing straight.

    :param gripper_z_pick_up_cube: Custom z-postion for gripping
        cubes while placing them down. Not used for picking up cubes.
    :param gripper_z_above_cube: Custom z-postion for save gripper
        position above cubes while placing them down. Not used for
        picking up cubes.
    """

    waypoints = []

    # Move to cube location
    waypoints.append(
        Waypoint(
            x_start, y_start, GRIPPER_Z_ABOVE_CUBE, -np.pi / 2.0, GRIPPER_OPENING_OPEN
        )
    )

    # Grab cube
    waypoints.append(
        Waypoint(
            x_start, y_start, GRIPPER_Z_PICK_UP_CUBE, -np.pi / 2.0, GRIPPER_OPENING_CUBE
        )
    )

    # Pick up cube
    waypoints.append(
        Waypoint(
            x_start, y_start, GRIPPER_Z_ABOVE_CUBE, -np.pi / 2.0, GRIPPER_OPENING_CUBE
        )
    )

    # Rotate gripper and move to finish location
    waypoints.append(
        Waypoint(x_finish, y_finish, gripper_z_above_cube, 0.0, GRIPPER_OPENING_CUBE)
    )

    # Place down cube
    waypoints.append(
        Waypoint(x_finish, y_finish, gripper_z_pick_up_cube, 0.0, GRIPPER_OPENING_OPEN)
    )

    # Move gripper up
    waypoints.append(
        Waypoint(x_finish, y_finish, gripper_z_above_cube, 0.0, GRIPPER_OPENING_OPEN)
    )

    return waypoints


def waypoints_for_pick_facing_straight_place_facing_down(
    x_start: float,
    y_start: float,
    x_finish: float,
    y_finish: float,
    gripper_z_pick_up_cube: float = GRIPPER_Z_PICK_UP_CUBE,
    gripper_z_above_cube: float = GRIPPER_Z_ABOVE_CUBE,
) -> List[Waypoint]:
    """
    Waypoints for picking up a cube with the gripper facing straight
    and placing it down with the gripperfacing down.

    :param gripper_z_pick_up_cube: Custom z-postion for gripping
        cubes while placing them down. Not used for picking up cubes.
    :param gripper_z_above_cube: Custom z-postion for save gripper
        position above cubes while placing them down. Not used for
        picking up cubes.
    """

    waypoints = []

    # Move to cube location
    waypoints.append(
        Waypoint(x_start, y_start, GRIPPER_Z_ABOVE_CUBE, 0.0, GRIPPER_OPENING_OPEN)
    )

    # Grab cube
    waypoints.append(
        Waypoint(x_start, y_start, GRIPPER_Z_PICK_UP_CUBE, 0.0, GRIPPER_OPENING_CUBE)
    )

    # Pick up cube
    waypoints.append(
        Waypoint(x_start, y_start, GRIPPER_Z_ABOVE_CUBE, 0.0, GRIPPER_OPENING_CUBE)
    )

    # Rotate gripper and move to finish location
    waypoints.append(
        Waypoint(
            x_finish, y_finish, gripper_z_above_cube, -np.pi / 2.0, GRIPPER_OPENING_CUBE
        )
    )

    # Place down cube
    waypoints.append(
        Waypoint(
            x_finish,
            y_finish,
            gripper_z_pick_up_cube,
            -np.pi / 2.0,
            GRIPPER_OPENING_OPEN,
        )
    )

    # Move gripper up
    waypoints.append(
        Waypoint(
            x_finish, y_finish, gripper_z_above_cube, -np.pi / 2.0, GRIPPER_OPENING_OPEN
        )
    )

    return waypoints


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

        waypoints.extend(
            waypoints_for_moving_between_start_finish(
                x_start, y_start, x_finish, y_finish
            )
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

    waypoints = []

    for location in cube_locations:
        x, y, direction = location

        if direction == "front":
            waypoints.extend(
                waypoints_for_pick_facing_down_place_facing_straight(x, y, x, y)
            )
        elif direction == "back":
            waypoints.extend(
                waypoints_for_pick_facing_straight_place_facing_down(x, y, x, y)
            )
        elif direction == "down":
            waypoints.extend(
                [
                    *waypoints_for_pick_facing_down_place_facing_straight(x, y, x, y),
                    *waypoints_for_pick_facing_down_place_facing_straight(x, y, x, y),
                ]
            )
        else:
            assert False, f"Direction is invalid: {direction}"

    return waypoints


def compute_waypoints_for_task_2c(
    starting_locations: List[Tuple[float, float, str]],
    finishing_locations: List[Tuple[float, float]],
) -> List[Waypoint]:
    """
    Computes a sequence of waypoints to achieve task 2c.

    The task involves picking up cubes from three starting locations,
    potentially rotating them and stacking them on one of the finishing
    locations.

    :param starting_locations: A list of (x,y, direction). Direction is
        one of "front", "back", or "down".
    :param finishing_locations: A list of (x, y) coordinates.
    :return: A list of waypoints.
    """
    # TODO: Improve by stacking cubes with gripper orientation = 0.
    # This would likely improve reach.
    # Another option is to pick a starting position that guarantees
    # the current algorithm to work. However, must check if this is
    # always possible.

    waypoints = []

    # Choose one of the finishing locations
    x_finish, y_finish = finishing_locations[0]

    # Keep track of where to place next cube on stack
    next_gripper_z_pickup_cube = GRIPPER_Z_PICK_UP_CUBE
    next_gripper_z_above_cube = GRIPPER_Z_ABOVE_CUBE

    for start in starting_locations:
        x_start, y_start, direction = start

        if direction == "front":
            # No rotation required
            waypoints.extend(
                waypoints_for_moving_between_start_finish(
                    x_start,
                    y_start,
                    x_finish,
                    y_finish,
                    next_gripper_z_pickup_cube,
                    next_gripper_z_above_cube,
                )
            )
        elif direction == "back":
            # Rotate on and place back down at start location
            waypoints.extend(
                waypoints_for_pick_facing_down_place_facing_straight(
                    x_start,
                    y_start,
                    x_start,
                    y_start,
                    next_gripper_z_pickup_cube,
                    next_gripper_z_above_cube,
                )
            )

            # Rotate again and move to finish location
            waypoints.extend(
                waypoints_for_pick_facing_down_place_facing_straight(
                    x_start,
                    y_start,
                    x_finish,
                    y_finish,
                    next_gripper_z_pickup_cube,
                    next_gripper_z_above_cube,
                )
            )
        elif direction == "down":
            waypoints.extend(
                waypoints_for_pick_facing_down_place_facing_straight(
                    x_start,
                    y_start,
                    x_finish,
                    y_finish,
                    next_gripper_z_pickup_cube,
                    next_gripper_z_above_cube,
                )
            )
        else:
            assert False, f"Direction is invalid: {direction}"

        next_gripper_z_pickup_cube += GRIPPER_Z_PICK_UP_CUBE
        next_gripper_z_above_cube += GRIPPER_Z_PICK_UP_CUBE

    return waypoints
