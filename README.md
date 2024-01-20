# SLAM-ROS

## Overview:
A comprehensive repository based on ROS Noetic, featuring custom-written nodes for Simultaneous Localization and Mapping (SLAM). This project provides a set of Python3-based nodes designed to enhance SLAM functionality within ROS Noetic environments.

## Custom Scripts:
1. **Join States to TF:** Enables seamless integration of joint states into the Transform (TF) tree for improved mapping accuracy.
2. **Joint States to Odometry:** Converts joint states information into Odometry data, enhancing the rover's localization capabilities.
3. **Laser to Map:** Transforms laser scan data into a map representation, a crucial step in building an accurate environment model.
4. **Path Publisher:** Publishes path information, contributing to the creation and optimization of the robot's navigation path.
5. **Path to Velocity:** Converts path data into velocity commands, aiding in dynamic path following during navigation.
6. **Teleop Controller:** Enables teleoperation control, allowing users to manually navigate the robot for testing and exploration.
7. **Twist to Controllers:** Converts Twist messages into control commands for various robot controllers, enhancing flexibility in robotic motion.

## Usage:
1. Navigate into the ros package.
1. Download the repository: `git clone https://github.com/Aravindhakumar-M/SLAM-ROS.git`
2. Navigate to the project directory: `cd <workspace name>`
3. Build the project: `catkin_make`
4. Source the environment: `source devel/setup.bash`
5. Execute the desired Python script: `rosrun  script_name.py`

# Acknowledgments
Credit to the creators of ROS and its libraries.   
Extending gratitude to Anya Robotics Pvt Ltd for their valuable contributions and support.
