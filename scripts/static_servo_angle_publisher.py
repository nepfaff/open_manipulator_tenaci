#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import numpy as np

from open_manipulator_tenaci.dh import tf_from_dh


def main():
    # Change these (send to robot)
    # All angles are in radians
    joint1_angle = np.pi
    joint2_angle = -1.717
    joint3_angle = 0
    joint4_angle = -1.424

    gripper_angle = 0.05
    gripper_sub_angle = 0.0

    rospy.init_node("static_servo_angle_publisher")

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
    # Not sure what this does
    gripper_sub_publisher = rospy.Publisher(
        "/gripper_sub_posigtion/command", Float64, queue_size=10
    )

    while not rospy.is_shutdown():
        # Compute forward kinematics
        T01 = tf_from_dh(0, 0, 0, joint1_angle + np.pi / 2.0)
        T12 = tf_from_dh(0.0, 0, 0.077, 0)
        T23 = tf_from_dh(np.pi / 2.0, 0.0, 0.0, -joint2_angle + np.pi / 2.0 - 0.1853)
        T34 = tf_from_dh(0, 0.13, 0, -joint3_angle - np.pi / 2.0 + 0.1853)
        T45 = tf_from_dh(0, 0.124, 0, -joint4_angle)
        T5G = tf_from_dh(0, 0.126, 0, 0)
        tool_pose = T01 @ T12 @ T23 @ T34 @ T45 @ T5G @ np.array([0, 0, 0, 1])

        # Print joint and gripper (tool) poses
        print("-------------------------------")
        print("Joint 1 pose:", T01 @ np.array([0, 0, 0, 1]))
        print("Joint 2 pose:", T01 @ T12 @ T23 @ np.array([0, 0, 0, 1]))
        print("Joint 3 pose:", T01 @ T12 @ T23 @ T34 @ np.array([0, 0, 0, 1]))
        print("Joint 4 pose:", T01 @ T12 @ T23 @ T34 @ T45 @ np.array([0, 0, 0, 1]))
        print("Tool pose:", tool_pose[:3])
        print("-------------------------------")

        # Publish joint angles to Gazebo topics
        joint1_publisher.publish(joint1_angle)
        joint2_publisher.publish(joint2_angle)
        joint3_publisher.publish(joint3_angle)
        joint4_publisher.publish(joint4_angle)

        # Publish gripper opening angles to Gazebo topics
        gripper_publisher.publish(gripper_angle)
        gripper_sub_publisher.publish(gripper_sub_angle)

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
