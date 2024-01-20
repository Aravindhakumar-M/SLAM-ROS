# Robot Wheel Velocity Controller

This Python script is designed to control the velocities of a robot's wheels based on path information received from the `/path` topic. It is intended to be used in a ROS (Robot Operating System) environment.

## Prerequisites

- ROS (Robot Operating System): Ensure that you have ROS installed and configured on your system to run this script.

## How to Use

1. Place the `path_to_vel.py` file in your ROS workspace or package.

2. Make the script executable by running the following command:

   ```shell
   chmod +x path_to_vel.py
   ```
3. Launch your ROS environment and run the script with the following command:

   ```shell
   rosrun your_package_name path_to_vel.py
   ```

The script will subscribe to the /path topic, process the path information, and control the robot's wheel velocities accordingly.

## Understanding the Code
- The script subscribes to the /path topic to receive path information.

- It calculates the desired linear and angular velocities based on the received path.

- The linear and angular velocities are then transformed into individual wheel velocities.

- The individual wheel velocities are published to the /cont1, /cont2, /cont3, and /cont4 topics, controlling the robot's wheels.

## Customization
- You can adjust the max_linear_vel and max_angular_vel values to limit the robot's maximum linear and angular velocities.

- Modify the topic names and frame IDs to match your robot's configuration.

# Acknowledgments
Credit to the creators of ROS and its libraries.   
Extending gratitude to Anya Robotics Pvt Ltd for their valuable contributions and support.
