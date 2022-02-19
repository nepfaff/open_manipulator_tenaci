#!/usr/bin/env python
from typing import List
import time

import rospy
from std_msgs.msg import Float64
import numpy as np

from open_manipulator_tenaci.inverse_kinematics import analytical_inverse_kinematics
from open_manipulator_tenaci.kinematics_common import get_first_valid_joint_angles
from open_manipulator_tenaci.path_planning import Waypoint


def main():
    # Start pose
    start_pose = Waypoint(x=0.0, y=0.274, z=0.2048, theta=0.0, gripper=0.05)

    # Waypoints (must include 'start_pose')
    current_waypoint = 0
    waypoints: List[Waypoint] = [
        start_pose,
        Waypoint(x=-0.148, y=0.0, z=0.079, theta=-np.pi / 2.0, gripper=-0.05),
        Waypoint(x=0.0, y=0.274, z=0.2048, theta=0.0, gripper=-0.05),
    ]

    rospy.init_node("waypoint_follower")

    waypoint_publishing_rate = rospy.Rate(0.5)  # hz
    gripper_command_delay_seconds = 2.0
    time_to_wait_after_reaching_start_seconds = 2.0

    # Joint publishers (joint1 is lowest joint)
    joint1_publisher = rospy.Publisher(
        "/joint1_position/command", Float64, queue_size=10
    )
    joint2_publisher = rospy.Publisher(
        "/joint2_position/command", Float64, queue_size=10
    )
    joint3_publisher = rospy.Publisher(
        "/joint3_position/command", Float64, queue_size=10
    )
    joint4_publisher = rospy.Publisher(
        "/joint4_position/command", Float64, queue_size=10
    )

    # Gripper opening publisher
    gripper_publisher = rospy.Publisher(
        "/gripper_position/command", Float64, queue_size=10
    )

    while not rospy.is_shutdown():
        if current_waypoint == len(waypoints):
            print("Final waypoint reached")
            return

        waypoint = waypoints[current_waypoint]

        # Compute inverse kinematics
        ik_solutions = analytical_inverse_kinematics(waypoint)
        ik_solution = get_first_valid_joint_angles(ik_solutions)

        if current_waypoint == 0:
            print("Moving to starting position")
        else:
            print(f"Starting waypoint {current_waypoint}")

        # Publish joint angles to Gazebo topics
        joint1_publisher.publish(ik_solution.joint1_angle)
        joint2_publisher.publish(ik_solution.joint2_angle)
        joint3_publisher.publish(ik_solution.joint3_angle)
        joint4_publisher.publish(ik_solution.joint4_angle)

        time.sleep(gripper_command_delay_seconds)

        # Publish gripper opening angles to Gazebo topics
        gripper_publisher.publish(waypoint.gripper)

        if current_waypoint == 0:
            print("Pausing before starting waypoint sequence")
            time.sleep(time_to_wait_after_reaching_start_seconds)

        current_waypoint += 1

        waypoint_publishing_rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
