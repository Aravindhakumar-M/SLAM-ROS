# ROS Map Maker

This Python script is designed to create a map based on laser scan data and odometry information in a ROS (Robot Operating System) environment. The script listens to laser scan data and odometry data, and uses this information to build an occupancy grid map. Additionally, it tracks the robot's path.

## Prerequisites

Before using this code, make sure you have the following prerequisites installed:

- ROS (Robot Operating System)
- Python 2.7 or 3.x
- NumPy library
- ROS packages for LaserScan, Odometry, and other relevant topics

## Usage

1. Place this script in a ROS package directory.

2. Build and compile your ROS workspace if required.

3. Launch the ROS master and relevant robot-related nodes.

4. Execute the script using Python:

   ```bash
   rosrun <package_name> map_maker.py
   ```

## Functionality
1. The script listens to laser scan data and odometry data.

2. It calculates the robot's position and orientation in the world coordinate system.

3. The laser scan data is used to determine obstacles and populate the occupancy grid map.

4. The map data is published as a ROS OccupancyGrid message.

5. The robot's path is tracked and published as a series of PoseStamped messages.

6. The map is built with a specified resolution and dimensions.

7. The map's origin is set based on the robot's position.

## Published Topics
`/map` The occupancy grid map is published on this topic as OccupancyGrid messages.

`/path` The robot's path is published on this topic as a series of PoseStamped messages.

# Acknowledgments
Credit to the creators of ROS and its libraries.   
Extending gratitude to Anya Robotics Pvt Ltd for their valuable contributions and support.
