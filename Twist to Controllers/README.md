# Twist data to Controller commands Publisher

This Python script is designed to control multiple motors using ROS (Robot Operating System). It subscribes to the `/cmd_vel` topic and calculates control commands for four motors based on the received Twist messages. The control commands are then published to specific topics for each motor.

## Prerequisites

Before using this code, you should have the following installed and set up:

1. ROS (Robot Operating System)
2. Python 3
3. ROS packages to handle standard messages

## How It Works

The script contains the following main components:

- `py_node(data)`: This callback function receives Twist messages containing linear (`x`) and angular (`z`) velocity data. It calculates control commands for four motors based on the input and publishes these commands to separate topics for each motor.

- `cmdvel()`: This function sets up a subscriber to the `/cmd_vel` topic and starts listening for incoming Twist messages.

## Usage

1. Make sure ROS is properly set up and running on your system.

2. Place this Python script in your ROS package directory.

3. Open a terminal and run the following command to start the ROS node:

   ```bash
   rosrun <your_package_name> cont_pub.py
   ```

The node will listen for Twist messages on the /cmd_vel topic and control the motors accordingly.
