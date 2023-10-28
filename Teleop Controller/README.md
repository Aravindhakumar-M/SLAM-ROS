# Teleoperation Node for ROS

This Python script provides a simple teleoperation node for controlling a robot using the ROS (Robot Operating System) framework. It allows you to control the robot's motion using keyboard input. The direction commands are published as `Twist` messages to the `/wheel` topic, allowing for easy robot control.

## Prerequisites

- Python 3
- ROS (Robot Operating System) environment
- Python `rospy` package
- Python `std_msgs` package

## Usage

1. Make sure you have a working ROS environment set up.

2. Place this Python script in your ROS package directory.

3. Run the script:

   ```bash
   rosrun your_ros_package_name teleop_node.py
   ```

## Use the arrow keys on your keyboard to control the robot's motion:
- Right arrow key: Move right
- Left arrow key: Move left
- Up arrow key: Move forward
- Down arrow key: Move backward

## Node Details
The script uses the curses library to capture keyboard input.
It initializes the ROS node as "teleop_node" and publishes direction commands as Int16MultiArray to the /wheel topic.
