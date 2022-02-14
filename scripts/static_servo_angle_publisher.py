#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import numpy as np

from open_manipulator_tenaci.forward_kinematics import forward_kinematics
from open_manipulator_tenaci.kinematics_common import JointAngles


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
        # Print joint and tool pose
        forward_kinematics(
            JointAngles(joint1_angle, joint2_angle, joint3_angle, joint4_angle), True
        )

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
