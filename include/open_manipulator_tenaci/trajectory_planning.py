from typing import List
from dataclasses import dataclass

import numpy as np

from open_manipulator_tenaci.path_planning import Waypoint
from open_manipulator_tenaci.kinematics_common import (
    JointAngles,
    ToolPose,
    get_first_valid_joint_angles,
)
from open_manipulator_tenaci.inverse_kinematics import analytical_inverse_kinematics


@dataclass
class SetPoint:
    """
    Used to represent tool trajectories.
    """

    joint_angles: JointAngles
    time: float


def compute_task_space_trajectory_setpoints(
    start_waypoint: Waypoint,
    end_waypoint: Waypoint,
    time_for_trajectory: float,
    sample_period: float,
) -> List[SetPoint]:
    """
    Returns samples of a cubic task space interpolation between two waypoints.

    :param start_waypoint: The start waypoint.
    :param end_waypoint: The destination waypoint.
    :param time_for_trajectory: Time in seconds in which the destination waypoint
        should be reached.
    :param sample_period: Sample period for sampling the cubic task space interpolation.
    :return: A list of set points. Each set point represents a sample of the task space
        trajectory in joint space representation. The first set point has time = 0 while
        the last set point has time = time_for_trajectory.
    """

    def waypoint_to_vector(w: Waypoint) -> np.ndarray:
        return np.array([w.x, w.y, w.z, w.theta])

    # Cubic polynomial
    waypoint_delta = waypoint_to_vector(end_waypoint) - waypoint_to_vector(
        start_waypoint
    )
    a0 = waypoint_to_vector(start_waypoint)
    a1 = 0
    a2 = 3 / (time_for_trajectory ** 2) * waypoint_delta
    a3 = -2 / (time_for_trajectory ** 3) * waypoint_delta

    # Sample polynomial and convert to joint space set point
    set_points: List[SetPoint] = []
    for t in np.arange(0.0, time_for_trajectory + sample_period, sample_period):
        tool_pose = a0 + a1 * t + a2 * (t ** 2) + a3 * (t ** 3)

        ik_solutions = analytical_inverse_kinematics(
            ToolPose(tool_pose[0], tool_pose[1], tool_pose[2], tool_pose[3])
        )
        valid_joint_angles = get_first_valid_joint_angles(ik_solutions)
        if valid_joint_angles is None:
            # Skip this set point
            continue

        set_points.append(SetPoint(valid_joint_angles, t))

    return set_points
