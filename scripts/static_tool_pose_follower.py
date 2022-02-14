#!/usr/bin/env python
from typing import List

import rospy
from std_msgs.msg import Float64
import numpy as np

from open_manipulator_tenaci.inverse_kinematics import analytical_inverse_kinematics
from open_manipulator_tenaci.kinematics_common import (
    JointAngles,
    ToolPose,
    do_joint_angles_violate_joint_limits,
)
from open_manipulator_tenaci.forward_kinematics import forward_kinematics


def main():
    # Change these (send to robot)
    tool_pose = ToolPose(x=0.0, y=0.148, z=0.079, theta=-np.pi / 2.0)

    # Gripper opening
    gripper_angle = 0.05

    rospy.init_node("static_tool_pose_follower")

    # Publishing rate
    rate = rospy.Rate(1)  # 1hz

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
        # Compute inverse kinematics
        ik_solutions = analytical_inverse_kinematics(tool_pose)

        # Choose one of the possible solutions
        valid_ik_sols: List[JointAngles] = []
        for sol in ik_solutions:
            if not do_joint_angles_violate_joint_limits(sol):
                valid_ik_sols.append(sol)

        print(f"Found {len(valid_ik_sols)} valid IK solutions")
        ik_solution = valid_ik_sols[0]

        # Print joint and tool pose
        forward_kinematics(ik_solution, True)

        # Publish joint angles to Gazebo topics
        joint1_publisher.publish(ik_solution.joint1_angle)
        joint2_publisher.publish(ik_solution.joint2_angle)
        joint3_publisher.publish(ik_solution.joint3_angle)
        joint4_publisher.publish(ik_solution.joint4_angle)

        # Publish gripper opening angles to Gazebo topics
        gripper_publisher.publish(gripper_angle)

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
