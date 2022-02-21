# open_manipulator_tenaci

ROS Noetic package for interacting with the OpenManipulator-X Gazebo simulation.
Mainly used as a helping tool for developing a low-level MATLAB robotic manipulation system based on the OpenManipulator-X robot.

## Installing Dependencies

1. Install Ubuntu >= 20.04
2. Install ROS Noetic
3. Install [OpenManipulator-X ROS packages](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/) (select Noetic version)
4. Clone this repository into the ROS Noetic workspace `/src` directory
5. Run `catkin_make install` from the ROS Noetic workspace

## Usage

1. Open a new terminal and navigate to your ROS Noetic workspace
2. Run `source devel/setup.bash` from the ROS Noetic workspace (this must be repeated whenever opening a new terminal, could add to `~/.bashrc` for convenience)
3. Launch Gazebo: `roslaunch open_manipulator_gazebo open_manipulator_gazebo.launch`
4. Start Gazebo using the play button (see [docs](http://gazebosim.org/tutorials?tut=guided_b2&cat=) for more info)
5. Run a ROS node from this (`open_manipulator_tenaci`) package, for example: `rosrun open_manipulator_tenaci static_servo_angle_publisher.py` where `static_servo_angle_publisher.py` should be replaced with the desired ROS node

## Development

1. Add a new ROS node to `open_manipulator_tenaci/scripts`
2. Add a `catkin_install_python(...)` clause to `CMakeLists.txt`
3. Run `catkin_make install` from the ROS Noetic workspace (this must only be run once for Python nodes and after every change for C++ nodes)
4. The new ROS node is now ready to use

## Available ROS nodes

- `static_servo_angle_publisher.py`
  - Publishes static angles in radians for each of the 5 servos to the relevant Gazebo topics. Also prints the corresponding tool pose using forward kinematics. Changing the published angles requires modifying the source code.
- `static_tool_pose_follower.py`
  - Publishes joint angles corresponding to a static pose. Uses analytical inverse kinematics. Also prints the corresponding tool pose using forward kinematics. Changing the pose requires modifying the source code.
- `waypoint_follower.py`
  - Follows a static sequence of waypoints. Changing the waypoints requires changing the source code.
- `waypoint_follower_with_task_space_interpolation.py`
  - Follows a static sequence of waypoints by first converting them into set points using cubic task space trajectories. Changing the waypoints requires changing the source code.
  