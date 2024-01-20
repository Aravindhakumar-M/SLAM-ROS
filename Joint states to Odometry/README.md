# Joint State to Odometry Publisher

This Python script is an example of a ROS (Robot Operating System) node that subscribes to joint state information and publishes odometry data. It's commonly used in robotics applications to estimate the position and orientation of a mobile robot based on wheel encoder information.

## Prerequisites

- ROS (Robot Operating System): This script is intended to be run in a ROS environment. Make sure you have ROS noetic installed and configured.

## How to Use

1. Place the `odom_pub.py` file in your ROS workspace or package.

2. Make sure the script is executable by running the following command:

   ```shell
   chmod +x odom_pub.py
   ```
3. Launch your ROS environment and run the script with the following command:
   ```shell
   rosrun <package_name> odom_pub.py
   ```
   
The script will start listening to the joint_states topic and publishing odometry data to the odom topic.

## Understanding the Code
- The script uses the rospy library to interact with ROS.

- It subscribes to the joint_states topic to receive information about the position of the robot's wheels.

- It calculates the robot's position and orientation based on the wheel encoder data and the distance between the wheels.

- The calculated odometry data is published on the odom topic.

## Customization
- You can customize the wheel_base variable in the script to match your robot's specifications. The wheel_base is the distance between the wheels of the robot.

- You can modify the topic names and frame IDs to match your robot's configuration.

# Acknowledgments
Credit to the creators of ROS and its libraries.   
Extending gratitude to Anya Robotics Pvt Ltd for their valuable contributions and support.
