# Wheel Encoder to TF Transform Publisher

This Python script is designed to calculate and publish transformations based on wheel encoder data received from the `/joint_states` topic in a ROS (Robot Operating System) environment. It provides a way to estimate the robot's position and orientation.

## Prerequisites

- ROS (Robot Operating System): Ensure that you have ROS Noetic installed and configured on your system to run this script.

## How to Use

1. Place the `tf_broadcaster.py` file in your ROS workspace or package.

2. Make the script executable by running the following command:

   ```shell
   chmod +x tf_broadcaster.py
   ```
3. Launch your ROS environment and run the script with the following command:
   ```shell
   rosrun your_package_name tf_broadcaster.py
   ```

The script will subscribe to the /joint_states topic, process the wheel encoder data, and publish transformations between the odom and world_link frames.

## Understanding the Code
- The script subscribes to the /joint_states topic to receive wheel encoder data, which includes the positions and velocities of the robot's wheels.

- It calculates the change in orientation and the distance traveled by the robot based on the wheel encoder data.

- It updates the robot's position and orientation over time and publishes these changes as transformations between the odom and world_link frames.

## Customization
- You can adjust the wheel parameters such as wheel radius and wheel separation to match your robot's specifications.

- Modify the frame names and topic names as needed to align with your robot's configuration.

# Acknowledgments
Credit to the creators of ROS and its libraries.   
Extending gratitude to Anya Robotics Pvt Ltd for their valuable contributions and support.
